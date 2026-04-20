# Sigyn Futures Work Plan

**Created:** 2026-03-25  
**Status:** Living document — update freely as decisions change.  
**Companion document:** `REFACTORING_PLAN.md` and `TODO.md` cover technical debt and hardware work. This document covers the *forward-looking capabilities* work.

---

## 1. Background: What Sigyn Is

Sigyn is a house-patrolling robot built by Michael Wimble. It operates in a residential home and is meant to autonomously patrol, detect anomalies, and eventually provide rich situational awareness about the home.

**Hardware platform:**
- Mobile base with RoboClaw motor controller (two drive motors, connected via Teensy 4.1 boards)
- Gripper assembly with elevator (second Teensy 4.1 board)
- LD LiDAR mounted approximately 5 feet off the floor (very high compared to typical robots — this is important for localization; it sees wall/ceiling intersections, doorframe tops, and high features that are extremely stable landmarks)
- OAK-D stereo camera (provides RGB, stereo depth, and onboard ML inference via DepthAI)
- VL53L0X time-of-flight sensors (8 units, short-range obstacle detection — currently partially disabled, re-enabling is in `TODO.md`)
- BNO055 IMU
- INA226 current sensor (battery monitoring)

**Software platform:**
- Ubuntu 24.04 + ROS 2 Jazzy
- Gazebo Harmonic for simulation
- Nav2 navigation stack
- BehaviorTree.CPP v4 for behavior trees
- Current SLAM: slam_toolbox or Cartographer (manually driven mapping sessions)
- Current localization: AMCL + EKF (robot_localization)
- Object detection: YOLOv8 via OAK-D (running in separate workspace `~/sigyn_oakd_detection_ws`)

**Key repos (all under `wimblerobotics/` on GitHub):**
- `Sigyn` — main monorepo (bringup, rviz config, udev rules)
- `sigyn_behavior_trees` — BT nodes and XML trees (extraction in progress as of 2026-02)
- `sigyn_interfaces` — ROS 2 message/action/service definitions
- `sigyn_description` — URDFs and Gazebo models
- `sigyn_to_teensy` — hardware bridge node (translates ROS topics to Teensy serial protocol)
- `sigyn_teensy_boards` — Teensy 4.1 firmware (PlatformIO, not colcon-built)
- `sigyn_oakd_detection` — OAK-D YOLO detection node (separate workspace)
- `wr_ldlidar` — customized LiDAR driver

**Build notes:**
```bash
# Main workspace
cd ~/sigyn_ws && colcon build --symlink-install

# OAK-D workspace (separate)
cd ~/sigyn_oakd_detection_ws && colcon build --symlink-install

# Source both for full environment
source ~/sigyn_ws/install/setup.bash
source ~/sigyn_oakd_detection_ws/install/setup.bash
```

---

## 2. The Vision (What This Document Plans For)

The current Sigyn stack is functional but largely reactive and manually configured. The futures work described here transforms Sigyn into a robot that can:

1. **Autonomously build and maintain a high-quality map of the house** without manual driving
2. **Understand what is in the map semantically** — not just occupied/free, but "wall", "chair leg", "door", "power socket", "appliance", etc.
3. **Localize robustly** — including after wheel slip, after power-up in an unknown location, and in areas with sparse AMCL features
4. **Detect meaningful changes** during patrols — open doors, fallen objects, unexpected people, unusual postures
5. **Maintain a persistent feature database** that survives map rebuilds and gains richness over time

These are described as four capability layers, each independently deliverable.

---

## 3. Capability Layer 1: Autonomous Map Building

### The Problem

Currently, mapping requires manually driving the robot while watching SLAM visualization. This is time-consuming, produces variable quality depending on driving skill, and needs to be repeated periodically.

### The Approach: Frontier-Based Exploration

A **frontier** is a free cell in the occupancy grid that is adjacent to an unknown cell. The robot's strategy is:

1. Detect all frontier cells in the costmap
2. Cluster nearby frontiers into candidate goals
3. Select the best frontier (nearest centroid works better than largest in house environments — avoids long backtracking)
4. Use Nav2 `NavigateToPose` to reach it
5. As SLAM fills in area, new frontiers appear — repeat
6. Terminate when no reachable frontiers remain

**The termination condition:** A frontier is considered unreachable if Nav2's path planner (`compute_path_to_pose`) fails or produces an implausibly long path. Maintain a blacklist of repeatedly unreachable frontiers so the robot doesn't thrash on them. When all frontiers are blacklisted or absent, exploration is complete. There is no need to prove cells are *physically* unreachable — only *navigation-unreachable*, which is tractable.

**Existing code to start from:** `explore_lite` has a maintained ROS 2 port. Read its source as a reference. You may want to wrap it in a BT action node rather than run it as a standalone node, for integration with the rest of Sigyn's behavior tree.

**Complication: doorways.** The Nav2 inflation radius that makes normal navigation safe often makes narrow doorways appear unplannable. For exploration mode, temporarily reduce the inflation radius or use a footprint that is slightly smaller than the real robot. Add this as a configurable parameter.

**For map *quality* (periodic re-mapping every few months):** After initial coverage, do a second systematic pass with a lawnmower or spiral pattern. This causes loop closure to fire many more times, which pulls accumulated drift out of the SLAM trajectory graph. Each loop closure globally corrects all previous poses. Multiple passes also average out LIDAR scan noise — a cell seen 20 times from different angles converges to ground truth faster than one seen twice.

**Architecture recommendation:** This maps naturally onto a BT. Top level:
```
RepeatUntilDone
  SequenceStar
    ComputeFrontiers          ← custom BT node, reads costmap
    SelectBestFrontier        ← custom BT node, picks nearest cluster centroid
    NavigateToPose            ← existing Nav2 BT node
    CheckExplorationComplete  ← custom BT node
```

### Suggested Package: RTAB-Map Instead of slam_toolbox

**Strong recommendation:** Switch to RTAB-Map for the periodic full-house remapping sessions. RTAB-Map is worth evaluating because:

- It does RGB-D + LiDAR SLAM with aggressive loop closure
- It maintains a database of visual and geometric features per session
- **Multi-session mapping:** Successive mapping runs feed back into the same database, improving quality over time rather than starting from scratch each time
- **Relocalization from the database:** When the robot powers up, RTAB-Map can perform global place recognition against its database and produce an initial pose without AMCL warm-up. This is directly useful for Layer 3.
- It has a maintained ROS 2 node and integrates with Nav2

RTAB-Map can coexist with slam_toolbox — you could use RTAB-Map for full re-mapping sessions and slam_toolbox for live localization during normal patrol.

---

## 4. Capability Layer 2: Semantic Feature Database

### The Problem

The occupancy grid knows "something is here" but not what. Navigation treats a chair leg the same as a wall. The patrol mission has no institutional memory of what features normally exist and where.

### The Target: A Persistent Feature Database

A SQLite database (or structured YAML/JSON sidecar) stored alongside the map file. Schema per entry:

| Field | Description |
|---|---|
| `id` | UUID |
| `map_version` | Which map generation this was derived from |
| `feature_type` | Enum: WALL, WALL_SEGMENT, DOOR, DOOR_FRAME, WINDOW, GLASS_WALL, TABLE_TOP, TABLE_LEG, CHAIR, COUCH, APPLIANCE, CABINET, POWER_SOCKET, LIGHT_SWITCH, FLOOR_CORD, FLOOR_MAT, UNKNOWN_OBSTACLE, etc. |
| `geometry` | 2D polygon or point in map coordinates (wkt or GeoJSON) |
| `height_min_m` | Bottom of occupied voxels |
| `height_max_m` | Top of occupied voxels |
| `color_rgb` | Average color from camera at this location |
| `confidence` | 0.0–1.0 |
| `first_observed` | Timestamp |
| `last_confirmed` | Timestamp of last patrol that confirmed this feature |
| `nickname` | Optional human-assigned name ("my bed", "dryer door", etc.) |
| `is_permanent` | Bool — permanent features (walls) vs. furniture vs. transient objects |

A ROS 2 service interface allows other nodes (BT nodes, navigation stack) to query the database:
- `GetFeatureAtPosition` — what is at (x, y)?
- `GetFeatureByNickname` — where is "guest bed"?
- `GetFeaturesByType` — where are all DOOR features?
- `UpdateFeature` — update last_confirmed, geometry, confidence

### The Processing Pipeline

**Step 1: Geometric classification (no ML required)**

Using PCL (Point Cloud Library) and an Octomap:
- Large vertical planar clusters (RANSAC plane fitting) → WALL candidates
- Bounded vertical planar clusters of typical door width (~0.8m) → DOOR candidates  
- Horizontal planes at 0.7–0.9m → TABLE_TOP / COUNTER candidates
- Thin vertical clusters (diameter < 0.1m, height > 0.3m) → TABLE_LEG / CHAIR_LEG
- Wide blobs at floor level (0–0.5m) → COUCH_BASE / CABINET_BASE

This geometry alone classifies most permanent structural features reliably and without training data.

**Step 2: YOLO projection**

OAK-D YOLO detections carry a 3D position from the stereo depth. Project those positions onto the map and associate them with the nearest geometric cluster from Step 1. The detector label enriches the geometric cluster's `feature_type`.

**Step 3: Wall outlet / light switch detection**

These are small, flat, and at known heights. A specialized classifier (or a fine-tuned YOLO class) looking at the RGB stream at known wall locations is more reliable than general object detection for these. Train with images collected during patrols.

**Step 4: Temporal integration**

Run continuously during patrols, not just during mapping. On each patrol pass, re-confirm known features and update `last_confirmed`. If a feature has not been confirmed in N patrols, reduce its confidence. This naturally handles moved furniture over time.

### Octomap Note

An Octomap (3D volumetric map on an octree data structure) is the right tool for Step 1. It is *not* what you might have intuitively expected: it does not give "8 directional distance readings." Instead, it is a tree that recursively subdivides 3D space into 8 equal child cubes, storing occupancy probability at each leaf. It efficiently represents 3D occupied space (empty regions are pruned, not stored).

The key query for semantic classification is bounding-box leaf iteration:
```cpp
for (auto it = tree->begin_leafs_bbx(min_pt, max_pt);
     it != tree->end_leafs_bbx(); ++it) {
    if (tree->isNodeOccupied(*it)) {
        double z = it.getCoordinate().z();  // height of this voxel
    }
}
```

This lets you ask "what is the vertical extent of occupation in this 2D column?" — which is the key discriminator between feature types.

**ROS 2 package:** `octomap_server2` (community-maintained). Note that the broader Nav2 ecosystem uses Voxel Layer and NVBLOX for 3D costmap work, so Octomap is used here only as a feature extraction tool, not as a Nav2 layer.

---

## 5. Capability Layer 3: Robust Localization

### The Problem

AMCL localization breaks in several important scenarios:
- **Wheel slip** (e.g., running into a wall): odometry diverges, AMCL takes time to recover
- **Power-up at unknown location**: AMCL requires a reasonable initial pose estimate
- **Feature-sparse areas**: Long corridors, blank walls — AMCL particle filter has high uncertainty

### The Approach: Multi-Source EKF with LiDAR Feature Matching

The robot_localization EKF already fuses wheel odometry and IMU. The strategy is to add a third source: a **LiDAR-based geometric localizer** that produces a `nav_msgs/Odometry` input to the existing EKF.

When the LiDAR-based source has high confidence and the AMCL covariance is high (indicating AMCL uncertainty), the EKF weights favor the LiDAR source. In normal conditions, AMCL is the reliable fallback.

**Why Sigyn's LiDAR height is an advantage:**

The LD LiDAR is mounted approximately 5 feet off the floor. At this height it sees:
- Wall/ceiling intersection edges (extremely stable, permanent)
- Doorframe tops (distinctive, fixed geometry)
- Upper cabinet faces
- TV/monitor faces (glass, distinctive reflectance pattern)
- High appliance profiles

These features are far more stable than floor-level features (chairs get moved, boxes appear). A feature database built from these high features will be highly reliable for localization.

**KISS-ICP:**

KISS-ICP (Keep It Small and Simple — Iterative Closest Point) is modern, fast, and accurate LiDAR odometry. It takes consecutive LiDAR scans and estimates relative motion. Better than wheel odometry for detecting wheel slip (if the robot slips, the LiDAR scan match provides a corrected motion estimate). Feed its output as an additional odometry source to the EKF.

GitHub: `tiagomonteirinho/kiss-icp` — has a ROS 2 wrapper.

**ScanContext / Place Recognition for Power-Up Localization:**

ScanContext++ converts a single LiDAR scan into a compact 2D descriptor (a ring-key encoding of height and intensity distributions). It can query a database of prior descriptors to find "I've been near here before" in milliseconds. 

At power-up:
1. ScanContext++ queries its database against the current scan → returns top-K candidates with map poses
2. ICP refines the best candidate against the stored map point cloud → produces a precise initial pose
3. If the ICP fitness score is above a threshold, inject the pose into AMCL as the initial pose → AMCL converges immediately

This solves the power-up localization problem without requiring a human to give the robot its location.

GitHub: `gisbi-kim/SC-A-LOAM` or standalone `ScanContext` library — check for ROS 2 availability before starting (the ROS 2 ecosystem for this has been moving fast).

**Confidence Monitor Node:**

A small (~100-line) ROS 2 node monitors the AMCL covariance trace and the ICP fitness score, and publishes a weighted pose estimate or a source selector. Logic:
- If ScanContext+ICP confidence > threshold AND AMCL covariance trace > threshold → use LiDAR pose
- If both sources agree within a threshold → reinforce with EKF fusion
- If AMCL covariance is low (AMCL is confident) → trust AMCL

This node is the arbitration layer. It does not replace the EKF — it feeds the EKF with appropriately weighted sources.

---

## 6. Capability Layer 4: Change Detection and Anomaly Reporting

### The Problem

Nav2's costmap system was designed for navigation, not for noticing that the dryer door is open or that someone fell. The static map is used for planning but live sensor disagreement with it (e.g., a moved chair) causes navigation hesitation rather than a semantically meaningful event.

### Approach: Patrol-to-Reference Comparison

During each patrol, the robot:
1. Collects LiDAR scan data along the patrol route
2. Registers it against the reference scan from the semantic feature database (ICP or scan-to-map matching)
3. Computes per-cell difference between current observation and reference
4. Clusters difference regions
5. Filters noise (small isolated clusters, consistent with prior patrol noise floor)
6. Associates surviving clusters with known features: "this cluster is at the location of the door to outside → door state changed"

**Event types and thresholds:**
- Large planar region where there was previously free space → door closed/opened, or new obstacle
- New floor-level cluster that wasn't there last patrol → object fell, new floor hazard
- Known large furniture missing from its location → furniture moved
- Unknown humanoid-shaped cluster → person present

### People Detection and Pose Analysis

**YOLOv8-pose** (not just YOLOv8) does skeleton pose estimation and person detection in a single inference pass. It returns joint positions in image space. With the OAK-D stereo depth, you can get 3D joint positions.

Post-processing on the skeleton joints:
- If the spine vector (hip-to-shoulder) is within 30° of horizontal → flagged as fallen/slumped
- If head joint is below hip joint height → flagged as inverted/fallen
- If no motion update on a detected person for N seconds while in a non-sleeping-zone location → flagged as possibly unconscious

This is achievable with straightforward joint angle arithmetic on top of existing YOLO inference.

**Resident vs. Stranger:**

Face recognition using an enrollment database of known residents. Practical approach:
- Use `face_recognition` Python library (based on dlib) or a lightweight ArcFace model on the OAK-D's onboard inference
- Enrollment database: a set of face embeddings per known person stored on the robot
- At runtime: compare detected face embedding against enrolled embeddings, cosine similarity threshold
- Unrecognized face during patrol → alert event

Privacy consideration: the enrollment database and face images must be encrypted at rest and never transmitted externally. Design this from the start.

**Pet detection:**

YOLOv8 baseline COCO model already classifies dogs and cats. For Sigyn's specific home, additional fine-tuning with photos of the actual pets will improve recall significantly.

---

## 7. Component Decisions and Rationale

This section records decisions made (or tentatively made) so future AI context is aware of them.

| Decision | Choice | Rationale |
|---|---|---|
| SLAM for re-mapping | RTAB-Map (evaluate over slam_toolbox) | Multi-session maps, relocalization from database, RGB+LiDAR fusion |
| Frontier exploration | Build on `explore_lite` ROS 2 port as reference | Proven, readable; wrap in BT action node for integration |
| 3D obstacle representation | Octomap via `octomap_server2` | Used for feature extraction only, not as a Nav2 layer |
| LiDAR odometry | KISS-ICP | Modern, accurate, handles slip detection |
| Place recognition | ScanContext++ | Fast descriptor-based matching, good for power-up localization |
| Feature database | SQLite with ROS 2 service interface | Survives map rebuilds, queryable by nickname or type |
| Object detection | YOLOv8 (OAK-D onboard) + specialized classifiers | Incremental: start with COCO, add house-specific fine-tuning |
| Pose estimation | YOLOv8-pose | Same inference pass as detection, 3D from OAK-D stereo depth |
| Face recognition | `face_recognition` / ArcFace, local encrypted DB | Privacy-preserving, no external API |
| Semantic layer storage | Sidecar database alongside pgm/yaml map files | Simple, versioned with map, easy to inspect |

**Open decisions (not yet made):**
- Whether to replace AMCL entirely with a LiDAR-feature-based localizer, or keep AMCL as a fallback
- Whether the feature database is per-map-version or maintains feature identity across map rebuilds using spatial matching
- Specific YOLO classes to train for house-specific items (sockets, switches, etc.)
- Whether change detection runs continuously on every patrol or periodically (e.g., once per day)

---

## 8. Recommended Build Order

Each layer depends on the ones before it. Do not start Layer N+1 before Layer N is demonstrably working on the real robot.

### Phase 1: RTAB-Map Integration (4–6 weeks)
**What:** Replace or supplement the current manual SLAM workflow with RTAB-Map. Validate that it produces higher-quality maps with automatic loop closure. Set up multi-session mapping so successive runs accumulate into one database.

**Why first:** Every subsequent layer uses the map. Map quality and the RTAB-Map relocalization capability unblock Layer 3. All other features are only as good as the map they run on.

**Starting point:** `rtabmap_ros` package on GitHub, ROS 2 branch. Study the `rtabmap_launch` examples for LiDAR-only configuration, then add OAK-D RGB-D input.

### Phase 2: Frontier Exploration BT (2–3 weeks)
**What:** Automated exploration using frontier detection. Sigyn drives herself around the house until all reachable areas have been seen.

**Why second:** Once RTAB-Map is working, you have a SLAM back-end to feed. This makes re-mapping fully autonomous.

**Starting point:** `explore_lite` ROS 2 GitHub. Read its costmap subscriber and frontier detection code. Adapt into a BT action node rather than a standalone lifecycle node.

### Phase 3: KISS-ICP + ScanContext Localization (3–4 weeks)
**What:** Add LiDAR odometry and place recognition as additional EKF sources. Build the confidence monitor node. Solve the power-up localization problem.

**Why third:** Reliable localization is a prerequisite for reliable change detection. You can't know if something moved if you don't know precisely where the robot is.

**Starting point:** `kiss-icp` ROS 2 wrapper. For ScanContext, check the current ROS 2 ecosystem (was moving fast as of early 2026 — do a fresh search for maintained ROS 2 implementations before choosing).

### Phase 4: Geometric Feature Extractor + Database (4–6 weeks)
**What:** PCL-based plane fitting, cluster extraction, height profiling. Build the SQLite feature database and its ROS 2 service interface. Populate it from a mapping run.

**Why fourth:** Depends on a good map (Phase 1) and good localization (Phase 3). The database is the foundation for everything in Layers 2 and 4.

**Starting point:** PCL `SACSegmentation` (RANSAC plane fitting) and `EuclideanClusterExtraction`. These are well-documented with examples. The OAK-D depth stream or an Octomap provides the 3D point cloud.

### Phase 5: YOLO Semantic Annotation (2–3 weeks)
**What:** Project OAK-D YOLO detections onto map coordinates and associate with geometric clusters from Phase 4. Enrich feature database entries with YOLO labels.

**Why fifth:** Builds on Phase 4's geometric clusters. YOLO is already running; this is primarily a geometry/projection task.

**Starting point:** You already have `sigyn_oakd_detection` producing `/oakd/detections` with 3D position. The work is: (a) transform detection pose from camera frame to map frame using TF2, (b) query the feature database for clusters near that map position, (c) update the cluster's `feature_type` with the YOLO label and confidence.

### Phase 6: Change Detection Monitor (3–4 weeks)
**What:** Per-patrol comparison of current observations against the reference semantic map. Event generation for meaningful changes. Basic alert reporting.

**Why sixth:** Requires a reliable feature database (Phase 4) and reliable localization (Phase 3) to avoid false positives.

**Starting point:** No perfect existing package. Build as a new ROS 2 node that subscribes to `/map` updates and `/scan`, performs scan-to-map registration (ICP via PCL or RTAB-Map's graph), computes difference, and publishes `SigynChangeEvent` messages on a new topic.

### Phase 7: People/Pose/Identity (4–6 weeks)
**What:** YOLOv8-pose for skeleton detection, fallen/slumped posture classification, face recognition for resident vs. stranger.

**Why last:** Most dependent on everything else being stable. Also the most privacy-sensitive, so design the enrollment database carefully.

**Starting point:** YOLOv8-pose is a single model swap in `sigyn_oakd_detection` (change the model file path). Skeleton geometry post-processing is new code. Face recognition: evaluate whether the OAK-D's onboard processor can run a lightweight face embedding model, or run it on the host CPU.

---

## 9. Integration with Existing Work

### Relation to `TODO.md` / `REFACTORING_PLAN.md`

These futures items are logically separate from the tech debt tracked in those files. However, some debt items *block* this work:

- **VL53L0X re-enable** (`TODO.md`): Short-range sensors feed the local costmap. Want these working before heavy navigation-dependent testing in Phase 2.
- **Nav2 stack tuning** (`TODO.md`): Exploration-mode navigation (Phase 2) will require a separately tuned Nav2 config (smaller inflation radius, lower speeds, different planner parameters for open-ended exploration vs. point-to-point navigation).
- **sigyn_behavior_trees extraction** (`REFACTORING_PLAN.md`): Phase 2 and all subsequent BT work should go into the clean extracted repo, not into the old monorepo structure.

### New Packages These Phases Will Likely Create

| Package | Phase | Description |
|---|---|---|
| `sigyn_explorer` | Phase 2 | Frontier detection, exploration BT nodes |
| `sigyn_localization` | Phase 3 | KISS-ICP integration, ScanContext, confidence monitor |
| `sigyn_semantic_map` | Phase 4–5 | Feature database, PCL classifiers, ROS 2 service interface |
| `sigyn_change_monitor` | Phase 6 | Patrol comparison, event generation |
| `sigyn_person_monitor` | Phase 7 | Pose estimation, face recognition, alert generation |

These should be separate repos symlinked into `~/sigyn_ws/src/`, following the established pattern in this project.

---

## 10. Open Research Questions

Before starting each phase, spend time on these:

**Phase 1:**
- Has anyone successfully used RTAB-Map with the LD LiDAR model Sigyn uses? The LD LiDAR is not a typical 32-channel unit — check scan format compatibility with RTAB-Map's LiDAR SLAM front-end.
- What is the current state of `rtabmap_ros` on ROS 2 Jazzy? Check open issues.

**Phase 2:**
- Is the current ROS 2 port of `explore_lite` actively maintained? What Nav2 version does it target? (Nav2 API changed significantly between Humble and Jazzy.)

**Phase 3:**
- What is the current best-maintained ROS 2 implementation of ScanContext or a similar place-recognition system? As of early 2026 this space was evolving fast.
- Does KISS-ICP handle the LD LiDAR's scan format correctly? Test with a recorded bag before integrating.

**Phase 4:**
- What resolution Octomap is appropriate for house-scale semantic features? 5cm captures chair legs; 10cm is more memory-efficient for large rooms. Benchmark memory and CPU at both resolutions on the robot's compute platform.

**Phase 6:**
- What is the practical noise floor for scan-to-map ICP comparison (i.e., how much difference is expected between two "identical" scans of the same scene, just from measurement noise)? Need to measure this empirically with a few patrol recordings before setting change-detection thresholds.

---

## 11. Change Log

| Date | Change |
|---|---|
| 2026-03-25 | Initial document created from design conversation |

---

*This document is intentionally architectural and forward-looking. It does not duplicate the technical debt items in `REFACTORING_PLAN.md` or `TODO.md`. When picking this up in a new AI context, also read `AI_CONTEXT.md` in the Sigyn monorepo root for the current state of the codebase.*
