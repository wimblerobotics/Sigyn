# Sigyn Robot - Video Script for Homebrew Robotics Club
## Target Duration: 5-10 minutes
## Audience: Homebrew Robotics Club members (beginner to advanced)

---

## VIDEO OUTLINE

### **ACT 1: THE WHY** (1-1.5 minutes)
1. Hook: Show Sigyn navigating autonomously through narrow doorways
2. The Demographic Crisis
3. Personal Motivation
4. What "Aging in Place" Actually Means

### **ACT 2: THE CHALLENGE** (1-1.5 minutes)
1. Real-World Constraints
2. House Patrol Requirements
3. The Evolution from Simple to Complex

### **ACT 3: HARDWARE ENGINEERING DECISIONS** (3-4 minutes)
1. Physical Form Factor
2. Sensor Evolution (The Journey to Better Localization)
3. Custom Teensy 4.1 Architecture
4. Gripper System

### **ACT 4: SOFTWARE & INTEGRATION** (2-3 minutes)
1. ROS2 and Nav2 Foundation
2. Custom Behavior Trees
3. The Secret Sauce: Why Sigyn Moves Smoothly
4. Current Capabilities

### **ACT 5: THE FUTURE** (30-45 seconds)
1. Near-term goals
2. Long-term vision
3. Call to action

---

## FULL SCRIPT

---

### **ACT 1: THE WHY** (1-1.5 minutes)

#### **[VISUAL: Sigyn smoothly navigating through a narrow doorway, then patrolling]**

**NARRATION:**
"This is Sigyn. She's not just another hobby robot—she's my backup plan for aging.

Let me show you a chart that keeps me up at night."

#### **[VISUAL: Show the population demographics graph from README]**

**NARRATION:**
"In 2019, for the first time in human history, people over 65 outnumbered children under 5. By the time I need assistance—and I will—there simply won't be enough caregivers to go around.

I'm not wealthy. I can't count on having round-the-clock help. So I'm building my own.

I'm Mike Wimble, this is Wimble Robotics—a one-person effort to create an assistive robot that will help me live independently in my home as I age.

But here's the thing: I'm not talking about a voice assistant that tells you the weather. I'm talking about a robot that can navigate my house, detect problems, fetch objects, and physically help me when I need it.

Let me show you how I'm building it."

---

### **ACT 2: THE CHALLENGE** (1-1.5 minutes)

#### **[VISUAL: Walk-through footage of house, highlighting narrow spaces]**

**NARRATION:**
"Most tabletop robots at Homebrew Robotics Club navigate wide open spaces. Some of you have built TurtleBot-style robots that can navigate a room. But a house presents unique challenges.

Look at these doorways."

#### **[VISUAL: Overlay measurements on screen as you show each space]**
- Computer room doorway: **26 inches**
- Master bedroom: **25 inches**
- Guest bathroom: **23 inches**

**NARRATION:**
"Twenty-three inches. That's the width of my guest bathroom doorway.

This isn't an academic exercise. These are real constraints that dictate every engineering decision.

And it's not just navigation. Sigyn needs to:
- **Patrol the house** autonomously and detect changes—a door left open, something fallen on the floor, a stranger in the house
- **Monitor environmental conditions** like temperature anomalies that might indicate an open window
- **Fetch and carry objects**—eventually picking things up from the floor
- **Interact with the environment**—opening doors, operating appliances

These requirements drove me from building simple robots to building... well, let me show you what I built."

---

### **ACT 3: HARDWARE ENGINEERING DECISIONS** (3-4 minutes)

#### **Section 3.1: Form Factor** (30 seconds)

#### **[VISUAL: Show evolution from boxy Raven to cylindrical Sigyn]**

**NARRATION:**
"My previous robot, Raven, was box-shaped. It worked, but it was constantly bumping into doorframes and furniture. The corners were destructive.

Sigyn is cylindrical—18 inches in diameter. This gives me about 3.5 inches clearance on each side in the narrowest doorways. The curved body is more forgiving when I inevitably bump into things.

But the real engineering challenge wasn't the body—it was the sensors."

---

#### **Section 3.2: The Sensor Evolution** (1.5-2 minutes)

#### **[VISUAL: Graphics showing sensor layout, perhaps an exploded view or annotated photo]**

**NARRATION:**
"Let me tell you about my journey to reliable localization. This is where things get interesting.

**First iteration: Single LIDAR**
I started with one LD06 LIDAR up top. It worked okay for mapping, but localization was shaky. The robot would drift during long runs.

**Second iteration: Dual LIDAR**
I added a second LIDAR. Now I'm fusing data from both into a single `/scan` topic. Localization improved dramatically. Why? More data points, better coverage of the environment, and redundancy.

#### **[VISUAL: Show diagram of LIDAR coverage]**

**Third iteration: Add an IMU**
I added a BNO085 IMU for orientation data. This gave me high-rate gyroscope data fused with the wheel odometry and LIDAR. Localization got even better—tighter tracking, faster recovery from kidnapping.

I tried adding a second IMU thinking more is better. It wasn't. One well-placed IMU was sufficient.

**Fourth iteration: Time-of-Flight sensors**
Here's a problem: LIDAR scans in a horizontal plane. Anything below the scan height is invisible. A cat, a shoe on the floor, a low table leg—the robot would hit them.

#### **[VISUAL: Show the 8 TOF sensors positioned around the base]**

So I added **eight VL53L0X time-of-flight sensors** around the base, pointing outward and downward. Now Sigyn sees obstacles from floor level up to the LIDAR plane. These publish on a separate `/scan_tof` topic that feeds into the navigation stack.

The result? Sigyn navigates smoothly and safely, even in cluttered spaces."

---

#### **Section 3.3: Custom Teensy 4.1 Boards** (1-1.5 minutes)

#### **[VISUAL: Show photos of the three custom Teensy boards installed in Sigyn]**

**NARRATION:**
"Now, you might be wondering: how do I manage all these sensors? This is where I made a decision that separates Sigyn from typical hobby robots.

I designed **three custom Teensy 4.1 boards**. Why Teensy? Because I need **near-realtime performance** for sensor sampling and motor control.

#### **[VISUAL: Diagram showing the three boards and their responsibilities]**

**Board 1** - The workhorse:
- Interfaces with the **RoboClaw motor controller**
- Reads **eight time-of-flight sensors** at high frequency
- Publishes **odometry** data
- Monitors **temperature sensors**

**Board 2** - Power and orientation:
- Monitors **current and voltage sensors** for battery management
- Handles **IMU data** (BNO085) at high sample rates

**Board 3** - Gripper control:
- Controls the **elevator motor** (vertical movement)
- Controls the **extender motor** (horizontal reach)

Plus, there's a **Raspberry Pi 5** dedicated to:
- The **gripper close servo**
- The **gripper camera** for vision-guided manipulation

#### **[VISUAL: Show a timing diagram or oscilloscope trace showing update rates]**

Why this architecture? **Frame rates matter.**

The Teensy boards can maintain:
- **100 Hz** odometry updates
- **50 Hz** IMU data
- **20 Hz** TOF sensor scans

Compare this to typical hobby robots running everything on a single Raspberry Pi at 10-20 Hz. That higher data rate translates directly into smoother motion and better localization.

The ROS2 Jazzy system on my main computer pulls this data over serial, fuses it with LIDAR, and feeds it into Nav2. It's a clean separation of concerns: Teensy handles realtime; ROS handles reasoning."

---

#### **Section 3.4: Gripper System** (45 seconds)

#### **[VISUAL: Show gripper extending, elevating, and closing]**

**NARRATION:**
"The gripper is Sigyn's hands. It's a three-axis system:
- **Elevator:** 48 inches of vertical travel—enough to reach from 20 inches off the ground to 68 inches high. That covers everything from the freezer handle at 27 inches to the top refrigerator shelf at 62 inches.
- **Extender:** An 18-inch drawer slide gives me 36 inches of horizontal reach—plenty to grab objects from deep shelves.
- **Gripper:** A servo-controlled close mechanism with a camera for visual feedback.

#### **[VISUAL: Show reach diagram overlaid on refrigerator or other target]**

This isn't theoretical. These dimensions are based on real measurements in my house. The gripper is designed to fetch objects, open doors, and eventually, pick things up from the floor.

Right now, it's in the proof-of-concept stage, but the hardware is there."

---

### **ACT 4: SOFTWARE & INTEGRATION** (2-3 minutes)

#### **Section 4.1: ROS2 Foundation** (45 seconds)

#### **[VISUAL: Show ROS2 node graph or rqt_graph]**

**NARRATION:**
"Sigyn runs **ROS2 Jazzy** on Ubuntu. The software stack includes:
- **Nav2** for navigation and path planning
- **robot_localization** for sensor fusion (Extended Kalman Filter)
- **Custom behavior trees** using BehaviorTree.CPP
- **OAK-D camera** for object detection using YOLO

I've built custom ROS2 packages for everything from Teensy sensor bridges to house patrol logic to websocket telemetry.

But the real magic is in how it all comes together."

---

#### **Section 4.2: Behavior Trees & Patrol** (45 seconds)

#### **[VISUAL: Show behavior tree visualization or diagram]**

**NARRATION:**
"For autonomous operation, I use behavior trees—think of them as hierarchical state machines with built-in fallback logic.

Right now, I have a proof-of-concept behavior tree called **`patrol_using_waypoints_launch.py`** that demonstrates autonomous house patrol. 

It's simple: navigate to waypoint A, check for changes, navigate to waypoint B, check for changes, repeat. But it proves that all the pieces—localization, navigation, sensor integration, and decision-making—work together.

This is the foundation. The next steps are more sophisticated patrols, anomaly detection, and task execution."

---

#### **Section 4.3: The Secret Sauce** (45 seconds)

#### **[VISUAL: Side-by-side comparison of jerky hobby robot vs. smooth Sigyn motion]**

**NARRATION:**
"If you've ever watched a typical hobby robot navigate, you've seen the jerky, hesitant motion—constant rotation corrections, oscillation around waypoints.

Sigyn doesn't do that. She moves smoothly and confidently. Why?

It comes down to three things:

**1. High-performance motor control:** 
The **RoboClaw controller** runs closed-loop velocity PID at 1 kHz. The motors smoothly track commanded velocities with minimal overshoot.

**2. High-rate odometry and sensor fusion:**
Combining 100 Hz odometry, 50 Hz IMU data, and dual LIDAR at 10 Hz gives the Extended Kalman Filter rich, frequent data. Better data = better localization = less correction.

**3. Custom Nav2 tuning:**
I've spent months tuning `navigation_sim.yaml`—controller frequencies, velocity profiles, trajectory sampling, cost functions. Every parameter matters.

#### **[VISUAL: Show excerpt from navigation_sim.yaml or overlay on smooth motion footage]**

The result? Sigyn navigates at higher speeds with less oscillation than most hobby robots. I documented all of this in a technical writeup called **'Why Sigyn Moves So Smoothly'** if you want the deep dive."

---

### **ACT 5: THE FUTURE** (30-45 seconds)

#### **[VISUAL: Concept renders or aspirational footage]**

**NARRATION:**
"So where is this going?

**Near-term:**
- Reliable autonomous patrolling with anomaly detection
- Object recognition and fetching
- Opening doors and interacting with appliances
- Autonomous charging

**Long-term:**
- Picking up objects from the floor
- More sophisticated manipulation tasks
- Integration with smart home systems
- Truly useful assistance as I age

This is a marathon, not a sprint. Every component I build, every sensor I add, every line of code I write is a step toward that future.

I'm building Sigyn because I have to. Because the alternative—losing my independence as I age—isn't acceptable.

#### **[VISUAL: Return to footage of Sigyn navigating smoothly]**

If you're at Homebrew Robotics Club, maybe you're building a tabletop robot. Maybe you're working on a TurtleBot-style platform. That's great—we all start somewhere.

But I want to show you that it's possible to build something more. Something that operates in the real world, in real homes, solving real problems.

The code is open source. The designs are available. I've documented the journey.

If you want to follow along, check out the Sigyn repository on GitHub. And maybe, just maybe, we can build a future where aging in place isn't a privilege—it's a choice we can all make.

Thanks for watching."

---

## ASSET CREATION CHECKLIST

### **Video Footage Needed:**

#### **Robot in Action:**
- [ ] Sigyn navigating smoothly through narrow doorway (26" computer room)
- [ ] Sigyn navigating through various doorways (show multiple angles)
- [ ] Sigyn patrolling autonomously (following waypoints)
- [ ] Close-up of smooth motion (no jerkiness, confident turns)
- [ ] Side-by-side: jerky hobby robot vs. smooth Sigyn (if possible, or use animation)
- [ ] Full patrol demonstration (maybe time-lapse)

#### **Hardware Close-ups:**
- [ ] Overview of Sigyn (360-degree slow pan)
- [ ] Close-up: Dual LIDAR sensors (top view showing both)
- [ ] Close-up: Eight TOF sensors around base
- [ ] Close-up: Custom Teensy Board 1 (labeled)
- [ ] Close-up: Custom Teensy Board 2 (labeled)
- [ ] Close-up: Custom Teensy Board 3 in gripper (labeled)
- [ ] Close-up: Raspberry Pi 5 and servo control
- [ ] Close-up: RoboClaw motor controller
- [ ] Close-up: IMU (BNO085)
- [ ] Gripper extending (horizontal motion)
- [ ] Gripper elevating (vertical motion)
- [ ] Gripper closing/opening
- [ ] Gripper camera view

#### **House Environment:**
- [ ] Walk-through of house showing narrow passages
- [ ] Measuring tape overlay showing doorway widths
- [ ] Refrigerator with measurements overlaid (handle heights, shelf heights)
- [ ] Examples of obstacles: cat, shoes, furniture

#### **Screen Recordings:**
- [ ] RViz visualization during navigation
- [ ] rqt_graph showing ROS2 node architecture
- [ ] Behavior tree visualization (if you have BT visualization tool)
- [ ] Terminal showing sensor data rates (rostopic hz)
- [ ] Battery overlay in RViz
- [ ] LIDAR scan visualization (both sensors)
- [ ] TOF sensor visualization

### **Graphics/Animations Needed:**

#### **Diagrams:**
- [ ] Population demographics graph (from README - already have)
- [ ] Floor plan with doorway widths labeled
- [ ] Sensor coverage diagram (top-down view showing LIDAR and TOF coverage)
- [ ] Evolution diagram: Single LIDAR → Dual LIDAR → + IMU → + TOF
- [ ] Three Teensy boards architecture diagram (showing what each controls)
- [ ] Gripper reach diagram (showing 20" to 68" range overlaid on refrigerator)
- [ ] Simplified ROS2 node graph (easier to read than raw rqt_graph)
- [ ] Control loop timing diagram (showing 100Hz odometry, 50Hz IMU, etc.)
- [ ] Before/After: Boxy Raven vs. Cylindrical Sigyn

#### **Annotations/Overlays:**
- [ ] Doorway width measurements (overlay on video)
- [ ] Sensor labels (overlay on hardware footage)
- [ ] Board function callouts (overlay on Teensy footage)
- [ ] Data rate indicators (overlay showing Hz rates)
- [ ] Reach envelope visualization (overlay on gripper footage)

### **B-Roll Footage:**
- [ ] You working on the robot (soldering, assembling, testing)
- [ ] Computer screen with code
- [ ] CAD designs (Fusion 360 of Teensy boards or gripper)
- [ ] Your workspace/lab

### **Photos to Gather:**
- [ ] High-resolution photos of all Teensy boards
- [ ] High-resolution photo of full robot (studio-lit if possible)
- [ ] Components laid out (for evolution story)

### **Audio:**
- [ ] Record narration (use script above)
- [ ] Record in quiet environment with good microphone
- [ ] Background music (subtle, non-intrusive) - suggest searching for "technology ambient" or "documentary background music"

### **Title Cards/Text Overlays:**
- [ ] Opening title: "Sigyn: Building an Assistive Robot for Aging in Place"
- [ ] Your name/affiliation: "Mike Wimble | Wimble Robotics"
- [ ] Section headers:
  - "The Why"
  - "The Challenge"
  - "Hardware Engineering"
  - "Software & Integration"
  - "The Future"
- [ ] Key statistics overlays:
  - "26 inches - narrowest doorway"
  - "100 Hz odometry updates"
  - "8 time-of-flight sensors"
  - "48 inches vertical reach"
- [ ] Closing card: GitHub URL and contact info

### **Optional Advanced Visuals:**
- [ ] 3D animation of sensor coverage (if you have skills/tools)
- [ ] Animated behavior tree execution
- [ ] Data visualization: sensor fusion in realtime
- [ ] Comparison chart: typical hobby robot vs. Sigyn specs

---

## PRODUCTION NOTES

### **Pacing:**
- Keep cuts dynamic—don't linger too long on any single shot
- Aim for 3-5 second shots for B-roll, longer for demonstrations
- Use smooth transitions (crossfades, not jarring cuts)

### **Storytelling Tips:**
- **Act 1:** Emotional hook—make them care
- **Act 2:** Establish the problem—make them understand
- **Act 3 & 4:** Show your solutions—make them impressed
- **Act 5:** Inspire them—make them want to build

### **Technical Presentation:**
- Avoid jargon overload for beginners
- Provide enough detail for advanced members to appreciate the engineering
- Use visuals to explain complex concepts
- Show, don't just tell (footage > talking head)

### **Editing Software Suggestions:**
- **DaVinci Resolve** (free, professional-grade)
- **OpenShot** (free, simpler)
- **Kdenlive** (free, Linux-friendly)
- **iMovie** (Mac, beginner-friendly)

### **Aspect Ratio:**
- 16:9 for YouTube/general viewing
- Consider 1:1 or 9:16 for social media clips

---

## SHORTENED VERSION (5-MINUTE TARGET)

If you need to cut down to 5 minutes:

1. **Reduce Act 1 to 45 seconds:** Demographics → motivation → "I'm building my backup plan"
2. **Reduce Act 2 to 45 seconds:** Quick montage of narrow doorways with measurements, list requirements
3. **Act 3 becomes 2 minutes:** Focus on sensor evolution and Teensy architecture; abbreviate form factor and gripper
4. **Act 4 becomes 1 minute:** Brief ROS2 mention, show patrol running, 30 seconds on smooth motion
5. **Act 5 becomes 30 seconds:** Quick future vision, call to action

Total: ~5 minutes

---

## EXTENDED VERSION (10-MINUTE TARGET)

If you want to go longer:

1. **Add technical deep-dives:**
   - Show actual `navigation_sim.yaml` parameters and explain what they do
   - Demonstrate sensor fusion in RViz with overlays
   - Show behavior tree XML and walk through logic
   
2. **Add more demonstrations:**
   - Show failed navigation attempts and how you debugged them
   - Demonstrate TOF sensors detecting low obstacles
   - Show gripper attempting to reach objects
   
3. **Add development story:**
   - Show evolution from Raven to Sigyn
   - Discuss failures and lessons learned
   - Show CAD design process

---

## SCRIPT TIMING BREAKDOWN

| Section | Content | Target Time |
|---------|---------|-------------|
| Act 1 | The Why | 1-1.5 min |
| Act 2 | The Challenge | 1-1.5 min |
| Act 3.1 | Form Factor | 0.5 min |
| Act 3.2 | Sensor Evolution | 1.5-2 min |
| Act 3.3 | Teensy Architecture | 1-1.5 min |
| Act 3.4 | Gripper | 0.75 min |
| Act 4.1 | ROS2 Foundation | 0.75 min |
| Act 4.2 | Behavior Trees | 0.75 min |
| Act 4.3 | Smooth Motion | 0.75 min |
| Act 5 | The Future | 0.5-0.75 min |
| **TOTAL** | | **8-11 minutes** |

Adjust timing based on your comfort level and available footage.

---

## FINAL CHECKLIST

### **Pre-Production:**
- [ ] Review script and timing
- [ ] Create shot list from asset checklist
- [ ] Gather all existing photos/footage
- [ ] Identify gaps in footage
- [ ] Plan shooting schedule for missing footage
- [ ] Prepare graphics/diagrams (use draw.io, Inkscape, or PowerPoint)

### **Production:**
- [ ] Shoot all robot footage
- [ ] Shoot all hardware close-ups
- [ ] Record all screen captures
- [ ] Record narration
- [ ] Create all graphics/animations

### **Post-Production:**
- [ ] Import all assets into editing software
- [ ] Create rough cut (assembly edit)
- [ ] Fine-tune timing and pacing
- [ ] Add graphics and overlays
- [ ] Mix audio (narration + music)
- [ ] Color correction (if needed)
- [ ] Export and review
- [ ] Get feedback from 1-2 trusted people
- [ ] Make final tweaks
- [ ] Export final version

### **Distribution:**
- [ ] Upload to YouTube
- [ ] Write compelling title: "Building Sigyn: An Assistive Robot for Aging in Place | Advanced ROS2 Mobile Manipulation"
- [ ] Write detailed description with links
- [ ] Add timestamps in description for each act
- [ ] Tag appropriately (ROS2, robotics, assistive technology, mobile manipulation, DIY)
- [ ] Share with Homebrew Robotics Club
- [ ] Post to relevant communities (r/ROS, r/robotics, ROS Discourse)

---

Good luck with your video! This is a compelling story that will resonate with both beginners and advanced roboticists. The key is showing your engineering thought process and the real-world constraints that drive your decisions.
