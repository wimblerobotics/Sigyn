Read [](file:///home/ros/sigyn_ws/src/Sigyn/Documentation/prompts/can_do_challenge_prompt.md)

This is an excellent challenge! Let me break this down into phases and ask clarifying questions before we start:

## **Questions Before Implementation:**

### **Phase 1 - Package Structure & Behavior Tree:**

1. **Package naming**: You want `can_do_challenge` - should the main node be called `can_do_challenge_node` (following perimeter_roamer_v3 pattern)?

2. **Behavior tree complexity**: Based on your description, the main tree should have these phases:
   - Setup (save starting pose, set object="CokeZeroCan")
   - ReactiveRobotSafety (continuous monitoring)
   - Navigate to expected can location (using map coordinates)
   - Search/detect can with OAK-D (visual servoing to get within 1 foot)
   - Fine positioning with Pi camera (rotate/center using gripper extender)
   - Grasp sequence (raise elevator → extend → close gripper → retract → lower)
   - Return to starting pose
   
   Should I create this as one main tree with multiple subtrees (like FMAB1.xml), or break it into more modular pieces?

3. **Known can location**: You mentioned a JSON database. Should I create:
   - A simple `can_locations.json` with one entry like: `{"CokeZeroCan": {"x": 2.0, "y": 1.5, "z": 0.75}}`?
   - A custom BT node `LoadCanLocation` that reads this JSON?

### **Phase 2 - Simulation World:**

4. **World file**: Looking at the `sim` alias, I need to find which world file it uses. Should I:
   - Clone the existing world file to `can_challenge_world.world`?
   - Add a simple table model (box primitive) at a specific location?
   - Add a cylinder for the can with a CokeZero texture?

5. **Gazebo models**: Should the table and can be:
   - Simple SDF primitives inline in the world file, OR
   - Separate model files in a `models/` directory?

### **Phase 3 - Placeholder Nodes:**

6. **Object detection placeholders**: Should I create nodes that:
   - `DetectCanWithOAKD` - returns SUCCESS if "can is visible" (hardcoded to succeed after X ticks?)
   - `DetectCanWithPiCamera` - similar placeholder?
   - Or should they actually subscribe to topics and just log that they're waiting?

7. **Gripper control**: Looking at sigyn_to_sensor_v2, should I:
   - Create action clients for gripper commands (similar to how perimeter_roamer_v3 uses Nav2 actions)?
   - Or just placeholder actions that log the commands?

### **My Proposed Plan (awaiting your approval):**

**Phase 1: Package Setup** (do first)
- Create `can_do_challenge` package with structure similar to perimeter_roamer_v3
- Create `main.xml` with complete tree structure but placeholder nodes
- Create bt_nodes.cpp with placeholder implementations
- Create `can_do_challenge_node.cpp` to load and run the tree

**Phase 2: Launch & Simulation** (do second)
- Find and clone the world file from `sim` alias
- Add table and can models
- Create `can_challenge_sim_launch.py` based on `sim.launch.py`

**Phase 3: Testing** (do third)
- Verify the tree loads and runs in simulation
- Placeholders log their execution

Should I proceed with **Phase 1 first**, or would you like me to tackle all three phases at once?