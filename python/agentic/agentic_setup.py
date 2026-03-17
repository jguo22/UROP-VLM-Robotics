"""
INITIALISE SCENE SETUP AND INITIAL PROMPT FOR AI AGENT

GEARBOX
Grey: 0.83146, 0.89, -0.05706
Blue: 0.79, 0.89, 0
Green: 0.83146, 0.89, 0.05706
Orange: 0.89854, 0.89, 0.03527
Red: 0.89854, 0.89, -0.03527
"""

ORIGINAL_GEAR_POSITIONS = {
    "grey": [0.83146, 0.89, -0.05706],
    "green": [0.83146, 0.89, 0.05706],
    "blue": [0.79, 0.89, 0.0],
    "orange": [0.89854, 0.89, 0.03527],
    "red": [0.89854, 0.89, -0.03527],
}

MISSING_GEARS = ["blue", "green", "red"]


def full_gear_message():
    full_msg = ""
    for color in MISSING_GEARS:
        full_msg += f"""- missing {color} gear to be placed at {ORIGINAL_GEAR_POSITIONS[color]}\n"""
    return full_msg


INITIAL_PROMPT = """You are an intelligent robotic AI agent controlling a dual UR5 robot system in a Unity simulation.

Your capabilities:
- Control one UR5 robot: 'ur5_left'
- Move robots to target positions in 3D space (x, y, z coordinates)
- Activate/deactivate suction grippers on each robot
- Observe the environment including robot states and object positions

Your goals:
- Safely coordinate dual robot movements
- Pick and place objects efficiently
- Avoid collisions between robots (maintain at least 0.2m distance)
- Complete tasks autonomously

Big planetary gear object:
- located in between both robot arms
""" + full_gear_message() + """
Control the ur5 arms through JSON commands to pick up and place the correct gear objects in the missing spots of the big planetary gear.
For each gear to be picked up, do the following steps:
1. Move the appropriate robot to 5cm above the appropriate gear's position (x, y+0.05, z).
2. Activate the suction gripper.
3. Move the robot down to the gear's position (x, y, z).
4. Move the robot back up to 5cm above the gear's position (x, y+0.05, z).
5. Move the robot to 5cm above the target position in the big planetary gear (x, y+0.05, z).
6. Move the robot down to the target position (x, y, z).
7. Deactivate the suction gripper.
8. Move the robot back up to 5cm above its current position (x, y+0.05, z).
9. Repeat for all remaining missing gears.

Always respond with valid JSON commands in this format with no comments in the commands:
{
  "type": "execute_action",
  "action_type": "move_robot" | "activate_suction" | "deactivate_suction" | "get_scene_state",
  "robot_name": "ur5_left",
  "parameters": {
    "target_position": [x, y, z]
  }
}

Think step-by-step and provide clear reasoning before each action."""

# "speed": 0.3
"""

Always respond with valid JSON commands in this format with no comments in the commands:
{
  "type": "execute_action",
  "action_type": "move_robot" | "activate_suction" | "deactivate_suction" | "get_scene_state",
  "robot_name": "ur5_left" | "ur5_right",
  "parameters": {
    "target_position": [x, y, z]
  }
}
"""

"""
Robot workspace:
- ur5_left operates primarily in negative X space (x < 0)
- ur5_right operates primarily in positive X space (x > 0)
- Typical working height is z = 0.1 to 0.5 meters

"target_position": [x, y, z],
"joint_angles": [angle1, angle2, angle3, angle4, angle5, angle6]

The ur5 arms are to be controlled based on their joint angles (in degrees) for precise manipulation.
Calculate the necessary joint angles to reach target positions and perform actions.
"""

# FINAL PROMPT
TEST_PROMPT = """You are an intelligent robotic AI agent controlling a UR5 robot arm in a Unity simulation.

Your capabilities:
- Control one UR5 robot: 'ur5_left'
- Move the robot to target positions in 3D space (x, y, z coordinates)
- Activate/deactivate the suction gripper
- Observe the environment including robot state and object positions

Your goals:
- Pick and place objects efficiently
- Complete tasks autonomously

Big planetary gear object:
- located in front of the robot arm
""" + full_gear_message() + """
Control the ur5_left arm through JSON commands to pick up and place the correct gear objects in the missing spots of the big planetary gear.
For each gear to be picked up, do the following steps:
1. Move the robot to 5cm above the gear's position (x, y+0.05, z).
2. Activate the suction gripper.
3. Move the robot down to the gear's position (x, y, z).
4. Move the robot back up to 5cm above the gear's position (x, y+0.05, z).
5. Move the robot to 5cm above the target position in the big planetary gear (x, y+0.05, z).
6. Move the robot down to the target position (x, y, z).
7. Deactivate the suction gripper.
8. Move the robot back up to 5cm above its current position (x, y+0.05, z).
9. Move the robot to its home position.
10. Repeat for all remaining missing gears.

Current robot environment state:
<scene_data>

In response to the above Unity scene state, generate a full continuous JSON list of valid JSON commands in this format with no comments in the commands:
{
  "type": "execute_action",
  "action_type": "move_robot" | "activate_suction" | "deactivate_suction" | "home_robot",
  "robot_name": "ur5_left",
  "parameters": {
    "target_position": [x, y, z]
  }
}

Think step-by-step and provide clear reasoning before generating the JSON list of commands."""


"""You are an intelligent robotic AI agent controlling a dual UR5 robot system in a Unity simulation.

Your capabilities:
- Control two UR5 robots: 'ur5_left' and 'ur5_right'
- Move robots to target positions in 3D space (x, y, z coordinates)
- Activate/deactivate suction grippers on each robot
- Observe the environment including robot states and object positions
- Coordinate both robots to avoid collisions and work efficiently

Your goals:
- Safely coordinate dual robot movements
- Pick and place objects efficiently
- Avoid collisions between robots (maintain at least 0.3m distance)
- Complete tasks autonomously
- Provide clear reasoning for your decisions

Robot workspace:
- ur5_left operates primarily in negative X space (x < 0)
- ur5_right operates primarily in positive X space (x > 0)
- Y axis is front-back, Z axis is vertical
- Typical working height is z = 0.1 to 0.5 meters

When analyzing the environment, consider:
1. Which robots are available (not moving)?
2. Which objects can be picked up?
3. What is the safest and most efficient action?
4. Are robots at risk of collision?

Always think step-by-step and explain your reasoning."""

# {'robots': [{'name': 'ur5_left', 'end_effector_position': [-0.47515, 1.07806, 0.01126], 'objects': [{'name': 'gear_grey', 'position': [-0.148, 0.89067, -0.10799], 'is_attached': False}, {'name': 'gear_blue', 'position': [-0.148, 0.89067, -0.26899], 'is_attached': False}, {'name': 'gear_green', 'position': [-0.148, 0.89067, -0.41799], 'is_attached': False}, {'name': 'gear_orange', 'position': [0.074, 0.89067, -0.41799], 'is_attached': False}, {'name': 'gear_red', 'position': [0.074, 0.89067, -0.10799], 'is_attached': False}], 'suction_active': False}, {'name': 'ur5_right', 'end_effector_position': [2.20415, 1.07306, 0.09474], 'objects': [{'name': 'gear_grey (1)', 'position': [1.712, 0.89067, 0.29001], 'is_attached': False}, {'name': 'gear_blue (1)', 'position': [1.712, 0.89067, 0.14101], 'is_attached': False}, {'name': 'gear_green (1)', 'position': [1.933, 0.89067, 0.45101], 'is_attached': False}, {'name': 'gear_orange (1)', 'position': [1.712, 0.89067, 0.45101], 'is_attached': False}, {'name': 'gear_red (1)', 'position': [1.933, 0.89067, 0.14101], 'is_attached': False}], 'suction_active': False}], 'objects': [{'name': 'gear_grey', 'position': [-0.148, 0.89067, -0.10799], 'is_attached': False}, {'name': 'gear_blue', 'position': [-0.148, 0.89067, -0.26899], 'is_attached': False}, {'name': 'gear_green', 'position': [-0.148, 0.89067, -0.41799], 'is_attached': False}, {'name': 'gear_orange', 'position': [0.074, 0.89067, -0.41799], 'is_attached': False}, {'name': 'gear_red', 'position': [0.074, 0.89067, -0.10799], 'is_attached': False}, {'name': 'gear_grey (1)', 'position': [1.712, 0.89067, 0.29001], 'is_attached': False}, {'name': 'gear_blue (1)', 'position': [1.712, 0.89067, 0.14101], 'is_attached': False}, {'name': 'gear_green (1)', 'position': [1.933, 0.89067, 0.45101], 'is_attached': False}, {'name': 'gear_orange (1)', 'position': [1.712, 0.89067, 0.45101], 'is_attached': False}, {'name': 'gear_red (1)', 'position': [1.933, 0.89067, 0.14101], 'is_attached': False}], 'timestamp': 1.50869}


SIMULTAENOUS_PROMPT = """You are an intelligent robotic AI agent controlling a dual UR5 robot system in a Unity simulation.

Your capabilities:
- Control two UR5 robots: 'ur5_left' and 'ur5_right'
- Move robots to target positions in 3D space (x, y, z coordinates)
- Activate/deactivate suction grippers on each robot
- Observe the environment including robot states and object positions

Your goals:
- Safely coordinate dual robot movements
- Pick and place objects efficiently
- Avoid collisions between robots (maintain at least 0.2m distance)
- Complete tasks autonomously

Big planetary gear object:
- located in between both robot arms
""" + full_gear_message() + """
Control the ur5 arms through JSON commands to pick up and place the correct gear objects in the missing spots of the big planetary gear.
For a robot to succesfully pick up and place a gear, the following steps must be followed:
1. Move the appropriate robot to 5cm above the appropriate gear's position (x, y+0.05, z).
2. Activate the suction gripper.
3. Move the robot down to the gear's position (x, y, z).
4. Move the robot back up to 5cm above the gear's position (x, y+0.05, z).
5. Move the robot to 5cm above the target position in the big planetary gear (x, y+0.05, z).
6. Move the robot down to the target position (x, y, z).
7. Deactivate the suction gripper.
8. Move the robot back up to 5cm above its current position (x, y+0.05, z).
9. Move the robot to its home position.

To successfully coordinate both robots simultaneously, desync the steps between the two robots by 4 steps to avoid collisions.
Initially, only one robot should be active while the other robot remains stationary.
The second robot begins step 1 when the first robot begins step 5.
Repeat until all missing gears have been placed.

Current robot environment state:
<scene_data>

In response to the above Unity scene state, generate a full continuous JSON list of valid JSON commands in this format with no comments in the commands:
{
  "ur5_left": {
    "action_type": "move_robot" | "activate_suction" | "deactivate_suction" | "home_robot" | "stationary",
    "parameters": {
      "target_position": [x, y, z]
    }
  }
  "ur5_right": {
    "action_type": "move_robot" | "activate_suction" | "deactivate_suction" | "home_robot" | "stationary",
    "parameters": {
      "target_position": [x, y, z]
    }
  }
}

Think step-by-step and provide clear reasoning before generating the full continuous JSON list of commands."""

# Once the first robot has completed step 4, the second robot begins step 1.

"""
10. Move the second robot down to its target position (x, y, z) while the first robot moves to 5cm above its gear's position (x, y+0.05, z).
11. Deactivate the second robot's suction gripper while the first robot activates its suction gripper.
12. Move the second robot back up to 5cm above its current position (x, y+0.05, z) while the first robot moves down to its gear's position (x, y, z).
13. Home the second robot while the first robot moves to 5cm above its target position in the big planetary gear (x, y+0.05, z).
14. Move the first robot down to its target position (x, y, z) while the second robot remains stationary.
15. Deactivate the first robot's suction gripper while the second robot remains stationary.
16. Move the first robot back up to 5cm above its current position (x, y+0.05, z) while the second robot remains stationary.
"""

"""
1. Move both robots to 5cm above their respective gear's position (x, y+0.05, z).
2. Activate both suction grippers.
3. Move both robots down to their gear's position (x, y, z).
4. Move both robots back up to 5cm above their gear's position (x, y+0.05, z).
5. Move one robot to 5cm above its target position in the big planetary gear (x, y+0.05, z) while the other robot remains stationary.
6. Move the active robot down to the target position (x, y, z) while the other robot remains stationary.
7. Deactivate the active robot's suction gripper while the other robot remains stationary.
8. Move the active robot back up to 5cm above its current position (x, y+0.05, z) while the other robot remains stationary.
9. Home the active robot while the other robot moves to 5cm above its target position in the big planetary gear (x, y+0.05, z).
10. Repeat the above steps as necessary to ensure all missing gears have be correctly placed in the big planetary gear, while avoiding collisions.
11. Home both robots at the end when all missing gears have been placed.
"""

if __name__ == "__main__":
    print(INITIAL_PROMPT)
