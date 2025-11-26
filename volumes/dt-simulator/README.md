# PyBullet Scene Simulation and Object State Export

This project uses **PyBullet** to simulate a simple robotic environment consisting of a **fixed Panda robotic arm** and **three stacked cube objects**, and exports key data such as positions, sizes, and grasp points of all dynamic objects in the scene to a structured JSON file. This JSON file will also be used by the Anchoring project as runtime data for the digital twin, to be inserted into TypeDB for subsequent semantic generation and reasoning tasks.

---

## Key Features

* Loads the following into a PyBullet GUI:

  * A fixed **Panda robotic arm**
  * **Three colored cube objects** stacked vertically
  * A **ground plane**

* Automatically identifies all dynamic objects and extracts:

  * **Base position**
  * **Bounding box dimensions**
  * **Grasp point position** (center of the cube's top surface)

* Also extracts:

  * **Gripper center position**
  * **Remaining reach distance** of the Panda arm

* Outputs all collected data into a structured JSON file: `box_positions.json`

---

## How to Use

### 1. Launch the simulation (with GUI)

```bash
python3 DT_Simulation.py
```

This opens a PyBullet GUI with the robot and cubes loaded.

### 2. Start the JSON updater (in a separate terminal)

```bash
python3 update_json.py
```

This script connects to the running PyBullet instance and updates the output file every 3 seconds.

---

## Output Format (JSON)

Example output in `box_positions.json`:

```json
[
    {
        "dt_id": "A",
        "position": [
            0.002,
            0.001,
            0.5
        ],
        "bounding_box": {
            "width": 0.201,
            "length": 0.201,
            "height": 0.2
        },
        "grasp_point": {
            "position": [
                0.002,
                0.001,
                0.6
            ]
        }
    },
    {
        "dt_id": "agent",
        "gripper": {
            "center": [
                -0.661,
                -0.001,
                0.821
            ],
            "remaining_reach": 0.079
        }
    }
]
```
...(It will also include the other two cubes, B and C)

---

## Notes

* The cubes are fully dynamic and can be dragged or manipulated in the GUI.
* The `grasp_point` is calculated as the center of the cube’s top surface using its AABB (axis-aligned bounding box).
* The Panda gripper position is extracted from link 11 of the robot.
* **To avoid configuration conflicts during simulation, it is recommended to create and run the project within an isolated virtual environment (venv).**

---
