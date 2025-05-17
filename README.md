# 🤖 Human Arm Kinematic Modeling and Teleoperation

This repository presents a comprehensive robotic modeling project developed as part of ENPM662 - Introduction to Robot Modeling. The goal was to simulate the upper portion of a human arm (elbow to fingers) using a 17 DOF kinematic structure in Gazebo and validate it using symbolic kinematic modeling tools, forward/inverse kinematics, and teleoperation control.

---

## 🧠 Project Overview
* Goal: To replicate human arm functionality in a virtual robot using realistic DH parameters and simulate motion in Gazebo.

* Scope: The system includes 6 DOF from elbow to fingertip and further breakdown across parallel finger structures, each with revolute joints.

* Applications: This can be extended to prosthetics, telemanipulation, or pick-and-place robots.

---

## 🔧 Robot Design and Structure
### 📌 Key Specifications
- Total DOF: 17 (includes fingers modeled as parallel manipulators)

- Kinematic Chain Modeled: Elbow to hand

- Joints: All joints are revolute

- Thumb joint orientation modeled separately from other fingers

- Modeled using SolidWorks and exported to Gazebo via ```SW2URDF```

### 📐 DH Parameters Summary
**For Bigger Fingers**
|Link|*a<sub>i</sub>*|	*α<sub>i</sub>*| *d<sub>i</sub>*|*θ<sub>i</sub>*|
|:---:|:---:|:---:|:---:|:---:|
|0-1|3.00|0|0|0|
|1-2|10.25|0|0|*θ<sub>1</sub>*|
|2-3|0|-90|0|*$θ<sub>2</sub>*-90|
|3-4|0|90|5.02|*θ<sub>3</sub>*|
|4-5|0.75|0|~0.38|*θ<sub>4</sub>*+90|
|5-6|0.75|0|0|*θ<sub>5</sub>*|
|6-7|1.01|0|0|*θ<sub>6</sub>*|

**For Thumb**
|Link|*a<sub>i</sub>*|	*α<sub>i</sub>*| *d<sub>i</sub>*|*θ<sub>i</sub>*|
|:---:|:---:|:---:|:---:|:---:|
|0-1|3.00|0|0|0|
|1-2|10.25|0|0|*θ<sub>1</sub>*|
|2-3|0.85|90|0|*$θ<sub>2</sub>*|
|3-4|2.22|-90|1.7|*θ<sub>3</sub>*-90|
|4-5|0.47|90|0|*θ<sub>4</sub>*+90|
|5-6|0.78|90|0|*θ<sub>5</sub>*|

## 🔬 Kinematic Modeling
### 🧮 Forward Kinematics
- Derived via standard DH convention.

- Validated with RoboAnalyzer for 4 joint configurations.

- All transformation matrices verified to match the CAD geometry and coordinate frames.

### 🔁 Inverse Kinematics
- Solved using Jacobian-based inverse velocity kinematics.

- Used numerical methods from SymPy for validation.

- Limited by physical workspace of anatomical fidelity.

---

## 🎮 Control and Teleoperation
* Teleop node built using ROS Python package.

* Keys mapped to specific joints for testing constraints and anatomical realism.

* Special logic:

  * Passive grip release on no input (mimics relaxed hand).

  * Grouped joint control for simplified teleoperation.

---

## 🛠️ Simulation Setup
* Gazebo: Model simulated in physics-enabled world.

* RViz: Basic static visualization.

* Xacro/URDF: Used for model definition and integration.

* SolidWorks Export: All meshes exported using SW2URDF.

### 🧩 Visualization Demo
Watch the arm perform rock-paper-scissors with randomized choices.\
🎥 [Simulation Video](https://drive.google.com/file/d/1i169GcTWNQIHsVtYh0mpbomPb4Xoh0bB/view)

---

## ⚙️ System Architecture Diagram

```
Input: Keyboard-based teleoperation
        ↓
ROS Controller Node
        ↓
Joint Commands → Gazebo Sim
        ↓
Human Arm URDF (17 DOF)
        ↓
Visual + Motion Feedback in Gazebo/RViz
```

---

## 📈 Challenges Faced
|Challenge|	Resolution|
|:---:|:---:|
|Floating model and physics glitches|	Added mass & adjusted inertia|
|Limited workspace	|Prioritized anatomical realism over full flexibility|
|Xacro validation errors|	Manual file validation + debugging|
|Wrist visualization confusion	|Redefined axis visualization in CAD|

---

## 🚀 Future Work
* Add feedback sensors and haptic interface → for prosthetic applications

* Improve range of motion → add additional base DOFs

* Use Myo armband or EMG for intuitive mirrored control
