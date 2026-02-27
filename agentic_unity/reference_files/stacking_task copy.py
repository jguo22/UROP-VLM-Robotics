import pybullet as p
import numpy as np
import time
import copy

def run_stacking_task(robot):
    """Run a task to stack the colored cubes on top of each other"""
    print("\nInitiating Cube Stacking Task...")
    
    # First, analyze the scene to locate objects
    robot.analyze_scene()
    
    # Find the platform
    platform_pos = None
    for obj_type, objects in robot.scene_objects.items():
        if 'platform' in obj_type and objects:
            platform_pos = objects[0]['position']
            break
    
    if not platform_pos:
        print("Error: Could not find a platform to stack cubes on")
        return
    
    # Find all the cubes
    cubes = {}
    for obj_type, objects in robot.scene_objects.items():
        if 'cube' in obj_type and objects:
            color = obj_type.split('_')[0]  # Extract color from the object type
            cubes[color] = {
                'position': objects[0]['position'],
                'id': objects[0]['id']
            }
    
    if not cubes:
        print("Error: No cubes found in the scene")
        return
    
    # Define the stacking order
    stacking_order = ['green', 'blue', 'red', 'yellow']
    
    # Filter out colors that aren't in the scene
    stacking_order = [color for color in stacking_order if color in cubes]
    
    print(f"Stacking cubes in order: {stacking_order}")
    
    # Define the stacking location (on the platform)
    stacking_location = [platform_pos[0], platform_pos[1], platform_pos[2] + 0.01]  # Slightly above platform
    print(f"Stacking location: {stacking_location}")
    
    # Improve physics stability for stacking
    previous_solver_iterations = p.getPhysicsEngineParameters()['numSolverIterations']
    p.setPhysicsEngineParameter(numSolverIterations=100)  # More iterations for better stability
    
    # Stack the cubes one by one
    for i, color in enumerate(stacking_order):
        print(f"\nStacking {color} cube ({i+1}/{len(stacking_order)})")
        
        # Get the cube position and ID
        cube_pos = cubes[color]['position']
        cube_id = cubes[color]['id']
        
        print(f"Detected {color} cube at: {cube_pos}")
        
        # Calculate the placement height based on the number of cubes already stacked
        placement_height = stacking_location[2] + (i * 0.05)  # Each cube is about 4-5cm tall
        
        # Define waypoints for picking and placing this cube with careful placement
        waypoints = [
            # Move above the cube
            (cube_pos[0], cube_pos[1], 0.2),
            # First approach - move closer to the cube
            (cube_pos[0], cube_pos[1], cube_pos[2] + 0.1),
            # Move down to the cube - position the suction cup to actually touch the cube
            (cube_pos[0], cube_pos[1], cube_pos[2] - 0.01),  # Go slightly below the top of the cube
            # Activate suction cup
            None,
            # Lift the cube slowly
            (cube_pos[0], cube_pos[1], cube_pos[2] + 0.05),
            (cube_pos[0], cube_pos[1], cube_pos[2] + 0.1),
            (cube_pos[0], cube_pos[1], 0.2),
            # Move to the stacking location (above)
            (stacking_location[0], stacking_location[1], 0.2),
            # Lower to just above the stacking position (careful approach)
            (stacking_location[0], stacking_location[1], placement_height + 0.1),
            # Lower to the final placement position
            (stacking_location[0], stacking_location[1], placement_height + 0.02),
            # Release the cube
            None,
        ]
        
        # Execute the waypoints for this cube
        robot.cube_attached = False
        
        for j, waypoint in enumerate(waypoints):
            if waypoint is None:
                if j == 3:  # Close gripper
                    print(f"Closing gripper on {color} cube")
                    # Visually close the gripper
                    robot.toggle_gripper()
                    
                    # Wait for the gripper to close
                    for _ in range(20):
                        p.stepSimulation()
                        time.sleep(0.005)  # Reduced delay for smoother motion
                    
                    # Check if the cube is close enough to the suction tip to be grabbed
                    tip_pos, tip_orn = p.getBasePositionAndOrientation(robot.suction_tip)
                    cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                    
                    # Calculate distance from cube to the tip
                    distance = np.linalg.norm(np.array(cube_pos) - np.array(tip_pos))
                    print(f"Distance from tip to cube: {distance:.4f}")
                    
                    # Only grab the cube if it's close enough (25cm threshold)
                    if distance <= 0.25:  # Increased to 25cm threshold
                        print(f"Grabbing {color} cube with suction tip")
                        # Store the cube ID we're working with
                        robot.grabbed_cube_id = cube_id
                        robot.cube_attached = True
                    else:
                        print(f"Cube too far to grab. Move closer to the cube.")
                        # Skip this cube if it's too far
                        robot.cube_attached = False
                        continue
                    
                    # Show a visual effect for the suction
                    p.changeVisualShape(robot.suction_cup, -1, rgbaColor=[0.9, 0.2, 0.2, 1])
                    
                    # Make the cube kinematic (not affected by physics)
                    p.changeDynamics(cube_id, -1, mass=0.0, linearDamping=0.8, angularDamping=0.8)
                    
                    # Disable collisions between the cube and other objects while it's attached
                    # This prevents unexpected physics interactions
                    for i in range(p.getNumBodies()):
                        if i != cube_id and i != robot.arm_id and i != robot.suction_cup:
                            p.setCollisionFilterPair(cube_id, i, -1, -1, 0)
                    
                    # Wait a bit for physics to settle
                    for _ in range(50):  # Further increased settle time for smoother motion
                        p.stepSimulation()
                        time.sleep(0.005)  # Reduced delay for smoother motion
                
                elif j == 10:  # Open gripper - careful placement
                    print(f"Carefully placing {color} cube")
                    # Visually open the gripper
                    robot.toggle_gripper()
                    
                    # Wait for the gripper to open
                    for _ in range(20):
                        p.stepSimulation()
                        time.sleep(0.005)  # Reduced delay for smoother motion
                    
                    # IMPORTANT: This is where we actually place the cube in its final position
                    # This is the key step that was missing before
                    final_pos = [stacking_location[0], stacking_location[1], placement_height]
                    p.resetBasePositionAndOrientation(cube_id, final_pos, p.getQuaternionFromEuler([0, 0, 0]))
                    print(f"Placed {color} cube at {final_pos}")
                    
                    # Restore the cube's physics properties gradually
                    p.changeDynamics(cube_id, -1, mass=0.1, linearDamping=0.9, angularDamping=0.9)
                    
                    # Re-enable collisions with other objects
                    for i in range(p.getNumBodies()):
                        if i != cube_id and i != robot.arm_id:
                            p.setCollisionFilterPair(cube_id, i, -1, -1, 1)
                    
                    # Update our scene data
                    for obj_type, objects in robot.scene_objects.items():
                        if color in obj_type and 'cube' in obj_type and objects:
                            objects[0]['position'] = final_pos
                    
                    # Mark the cube as released
                    robot.cube_attached = False
                    robot.grabbed_cube_id = None
                    
                    # Clear the stored offsets
                    if hasattr(robot, 'cube_offset_x'):
                        delattr(robot, 'cube_offset_x')
                    if hasattr(robot, 'cube_offset_y'):
                        delattr(robot, 'cube_offset_y')
                    
                    # Wait for physics to settle with monitoring
                    for _ in range(50):  # Further increased settle time for smoother motion
                        p.stepSimulation()
                        time.sleep(0.005)  # Reduced delay for smoother motion
                        
                        # Check if the cube is stable
                        cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                        euler = p.getEulerFromQuaternion(cube_orn)
                        
                        # If cube is tilted, correct it
                        if abs(euler[0]) > 0.05 or abs(euler[1]) > 0.05:
                            p.resetBasePositionAndOrientation(
                                cube_id, 
                                [cube_pos[0], cube_pos[1], placement_height],
                                p.getQuaternionFromEuler([0, 0, euler[2]])  # Keep yaw, reset pitch and roll
                            )
            else:
                # Move to the waypoint
                print(f"Moving to waypoint {j+1}/{len(waypoints)}: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]")
                
                # Convert 3D position to our 2D control + height
                robot.position = np.array([waypoint[0], waypoint[1]], dtype=np.float32)
                
                # Find the best height setting for the target z
                target_height_index = min(range(len(robot.rotation_options)), 
                                        key=lambda i: abs(0.3 - i*0.05 - waypoint[2]))
                robot.rotation_index = target_height_index
                robot.rotation = robot.rotation_options[robot.rotation_index]
                
                # Move the arm with collision avoidance
                robot.move_arm_to_position(robot.position, robot.rotation, check_collisions=True)
                
                # If we have a cube attached, move it with the arm
                if robot.cube_attached and hasattr(robot, 'grabbed_cube_id') and robot.grabbed_cube_id is not None:
                    # Get the suction cup position
                    suction_pos, suction_orn = p.getBasePositionAndOrientation(robot.suction_cup)
                    
                    # Get current cube position to calculate offset
                    current_cube_pos, current_cube_orn = p.getBasePositionAndOrientation(cube_id)
                    
                    # Calculate the initial offset between cube and suction cup
                    # This will help maintain the cube's relative position
                    offset_x = current_cube_pos[0] - suction_pos[0]
                    offset_y = current_cube_pos[1] - suction_pos[1]
                    
                    # Store these offsets for later use
                    robot.cube_offset_x = offset_x
                    robot.cube_offset_y = offset_y
                    
                    # Calculate the position for the cube (slightly below the suction cup)
                    new_cube_pos = [
                        suction_pos[0] + offset_x, 
                        suction_pos[1] + offset_y,
                        suction_pos[2] - 0.05
                    ]
                    
                    # DIRECTLY set the cube position
                    p.resetBasePositionAndOrientation(robot.grabbed_cube_id, new_cube_pos, suction_orn)
                
                # Wait for the arm to reach the waypoint
                for _ in range(20):  # Increased update frequency for smoother motion
                    p.stepSimulation()
                    time.sleep(0.005)  # Reduced delay for smoother motion
                    
                    # If we have a cube attached, keep updating its position
                    if robot.cube_attached and hasattr(robot, 'grabbed_cube_id') and robot.grabbed_cube_id is not None:
                        # Get the suction cup position
                        suction_pos, suction_orn = p.getBasePositionAndOrientation(robot.suction_cup)
                        
                        # Use the stored offsets to maintain the cube's relative position
                        # This prevents the cube from drifting toward the center axis
                        if hasattr(robot, 'cube_offset_x') and hasattr(robot, 'cube_offset_y'):
                            offset_x = robot.cube_offset_x
                            offset_y = robot.cube_offset_y
                        else:
                            # Fallback in case offsets weren't stored
                            offset_x = 0
                            offset_y = 0
                        
                        # Apply the offset to the current suction cup position
                        new_cube_pos = [
                            suction_pos[0] + offset_x,
                            suction_pos[1] + offset_y,
                            suction_pos[2] - 0.05  # Fixed offset below suction cup
                        ]
                        
                        # DIRECTLY set the cube position
                        p.resetBasePositionAndOrientation(robot.grabbed_cube_id, new_cube_pos, suction_orn)
    
    # Update scene analysis to get the new positions
    robot.analyze_scene()
    
    # Move back to a neutral position
    neutral_pos = [0.5, 0.0, 0.2]
    print("\nMoving to neutral position")
    robot.position = np.array([neutral_pos[0], neutral_pos[1]], dtype=np.float32)
    target_height_index = min(range(len(robot.rotation_options)), 
                            key=lambda i: abs(0.3 - i*0.05 - neutral_pos[2]))
    robot.rotation_index = target_height_index
    robot.rotation = robot.rotation_options[robot.rotation_index]
    robot.move_arm_to_position(robot.position, robot.rotation, check_collisions=True)
    
    # Restore original physics parameters
    p.setPhysicsEngineParameter(numSolverIterations=previous_solver_iterations)
    
    # Store the original positions of cubes for destacking later
    robot.original_cube_positions = {}
    for color, cube_info in cubes.items():
        robot.original_cube_positions[color] = cube_info['position']
    
    print("\nStacking task completed successfully!")
    

def run_destacking_task(robot):
    """Run a task to destack the colored cubes and return them to their original positions"""
    print("\nInitiating Cube Destacking Task...")
    
    # First, analyze the scene to locate objects
    robot.analyze_scene()
    
    try:
        # Find the platform
        platform_pos = None
        for obj_type, objects in robot.scene_objects.items():
            if 'platform' in obj_type and objects:
                platform_pos = objects[0]['position']
                break
        
        if not platform_pos:
            print("Error: Could not find a platform")
            return
        
        # Find all the cubes in the stack
        cubes = {}
        for obj_type, objects in robot.scene_objects.items():
            if 'cube' in obj_type and objects:
                color = obj_type.split('_')[0]  # Extract color from the object type
                cubes[color] = {
                    'position': objects[0]['position'],
                    'id': objects[0]['id']
                }
        
        if not cubes:
            print("Error: No cubes found in the scene")
            return
        
        # Define the destacking order (reverse of stacking order)
        destacking_order = ['yellow', 'red', 'blue', 'green']
        
        # Filter out colors that aren't in the scene
        destacking_order = [color for color in destacking_order if color in cubes]
        
        print(f"Destacking cubes in order: {destacking_order}")
        
        # Check if we have the original positions stored
        if not hasattr(robot, 'original_cube_positions') or not robot.original_cube_positions:
            print("Warning: Original cube positions not found. Will place cubes in predefined positions.")
            # Define some default positions around the platform
            robot.original_cube_positions = {
                'green': [platform_pos[0] - 0.15, platform_pos[1] - 0.15, platform_pos[2] + 0.025],
                'blue': [platform_pos[0] + 0.15, platform_pos[1] - 0.15, platform_pos[2] + 0.025],
                'red': [platform_pos[0] - 0.15, platform_pos[1] + 0.15, platform_pos[2] + 0.025],
                'yellow': [platform_pos[0] + 0.15, platform_pos[1] + 0.15, platform_pos[2] + 0.025]
            }
        
        # Improve physics stability for destacking
        previous_solver_iterations = p.getPhysicsEngineParameters()['numSolverIterations']
        p.setPhysicsEngineParameter(numSolverIterations=100)  # More iterations for better stability
        
        # Destack the cubes one by one
        for i, color in enumerate(destacking_order):
            print(f"\nDestacking {color} cube ({i+1}/{len(destacking_order)})")
            
            # Get the cube position and ID
            cube_pos = cubes[color]['position']
            cube_id = cubes[color]['id']
            
            print(f"Detected {color} cube at: {cube_pos}")
            
            # Get the original position to place the cube
            original_pos = robot.original_cube_positions.get(color, [platform_pos[0] + 0.2 + i*0.1, platform_pos[1], platform_pos[2] + 0.025])
            
            # Define waypoints for picking and placing this cube
            waypoints = [
                # Move above the stack
                (cube_pos[0], cube_pos[1], 0.2),
                # Approach the top cube
                (cube_pos[0], cube_pos[1], cube_pos[2] + 0.1),
                # Move down to the cube
                (cube_pos[0], cube_pos[1], cube_pos[2] - 0.01),
                # Activate suction cup
                None,
                # Lift the cube slowly
                (cube_pos[0], cube_pos[1], cube_pos[2] + 0.05),
                (cube_pos[0], cube_pos[1], cube_pos[2] + 0.1),
                (cube_pos[0], cube_pos[1], 0.2),
                # Move to the original position (above)
                (original_pos[0], original_pos[1], 0.2),
                # Lower to just above the placement position
                (original_pos[0], original_pos[1], original_pos[2] + 0.1),
                # Lower to the final placement position
                (original_pos[0], original_pos[1], original_pos[2] + 0.02),
                # Release the cube
                None,
            ]
            
            # Execute the waypoints for this cube
            robot.cube_attached = False
            
            for j, waypoint in enumerate(waypoints):
                if waypoint is None:
                    if j == 3:  # Close gripper
                        print(f"Closing gripper on {color} cube")
                        # Visually close the gripper
                        robot.toggle_gripper()
                        
                        # Wait for the gripper to close
                        for _ in range(20):
                            p.stepSimulation()
                            time.sleep(0.005)  # Reduced delay for smoother motion
                        
                        # Check if the cube is close enough to the suction tip to be grabbed
                        tip_pos, tip_orn = p.getBasePositionAndOrientation(robot.suction_tip)
                        cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                        
                        # Calculate distance from cube to the tip
                        distance = np.linalg.norm(np.array(cube_pos) - np.array(tip_pos))
                        print(f"Distance from tip to cube: {distance:.4f}")
                        
                        # Only grab the cube if it's close enough (25cm threshold)
                        if distance <= 0.25:  # Increased to 25cm threshold
                            print(f"Grabbing {color} cube with suction tip")
                            # Store the cube ID we're working with
                            robot.grabbed_cube_id = cube_id
                            robot.cube_attached = True
                        else:
                            print(f"Cube too far to grab. Move closer to the cube.")
                            # Skip this cube if it's too far
                            robot.cube_attached = False
                            continue
                        
                        # Show a visual effect for the suction
                        p.changeVisualShape(robot.suction_cup, -1, rgbaColor=[0.9, 0.2, 0.2, 1])
                        
                        # Make the cube kinematic (not affected by physics)
                        p.changeDynamics(cube_id, -1, mass=0.0, linearDamping=0.8, angularDamping=0.8)
                        
                        # Disable collisions between the cube and other objects while it's attached
                        # This prevents unexpected physics interactions
                        for i in range(p.getNumBodies()):
                            if i != cube_id and i != robot.arm_id and i != robot.suction_cup:
                                p.setCollisionFilterPair(cube_id, i, -1, -1, 0)
                        
                        # Get current cube position to calculate offset
                        current_cube_pos, current_cube_orn = p.getBasePositionAndOrientation(cube_id)
                        
                        # Calculate the initial offset between cube and suction cup
                        # This will help maintain the cube's relative position
                        suction_pos, suction_orn = p.getBasePositionAndOrientation(robot.suction_cup)
                        offset_x = current_cube_pos[0] - suction_pos[0]
                        offset_y = current_cube_pos[1] - suction_pos[1]
                        
                        # Store these offsets for later use
                        robot.cube_offset_x = offset_x
                        robot.cube_offset_y = offset_y
                        
                        # Wait a bit for physics to settle
                        for _ in range(50):  # Further increased settle time for smoother motion
                            p.stepSimulation()
                            time.sleep(0.005)  # Reduced delay for smoother motion
                    
                    elif j == 10:  # Open gripper - careful placement
                        print(f"Carefully placing {color} cube at original position")
                        # Visually open the gripper
                        robot.toggle_gripper()
                        
                        # Wait for the gripper to open
                        for _ in range(20):
                            p.stepSimulation()
                            time.sleep(0.005)  # Reduced delay for smoother motion
                        
                        # Place the cube in its original position
                        final_pos = original_pos
                        p.resetBasePositionAndOrientation(cube_id, final_pos, p.getQuaternionFromEuler([0, 0, 0]))
                        print(f"Placed {color} cube at {final_pos}")
                        
                        # Restore the cube's physics properties gradually
                        p.changeDynamics(cube_id, -1, mass=0.1, linearDamping=0.9, angularDamping=0.9)
                        
                        # Re-enable collisions with other objects
                        for i in range(p.getNumBodies()):
                            if i != cube_id and i != robot.arm_id:
                                p.setCollisionFilterPair(cube_id, i, -1, -1, 1)
                        
                        # Update our scene data
                        for obj_type, objects in robot.scene_objects.items():
                            if color in obj_type and 'cube' in obj_type and objects:
                                objects[0]['position'] = final_pos
                        
                        # Mark the cube as released
                        robot.cube_attached = False
                        robot.grabbed_cube_id = None
                        
                        # Clear the stored offsets
                        if hasattr(robot, 'cube_offset_x'):
                            delattr(robot, 'cube_offset_x')
                        if hasattr(robot, 'cube_offset_y'):
                            delattr(robot, 'cube_offset_y')
                        
                        # Wait for physics to settle with monitoring
                        for _ in range(50):  # Further increased settle time for smoother motion
                            p.stepSimulation()
                            time.sleep(0.005)  # Reduced delay for smoother motion
                            
                            # Check if the cube is stable
                            cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                            euler = p.getEulerFromQuaternion(cube_orn)
                            
                            # If cube is tilted, correct it
                            if abs(euler[0]) > 0.05 or abs(euler[1]) > 0.05:
                                p.resetBasePositionAndOrientation(
                                    cube_id, 
                                    [cube_pos[0], cube_pos[1], original_pos[2]],
                                    p.getQuaternionFromEuler([0, 0, euler[2]])  # Keep yaw, reset pitch and roll
                                )
                else:
                    # Move to the waypoint
                    print(f"Moving to waypoint {j+1}/{len(waypoints)}: [{waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}]")
                    
                    # Convert 3D position to our 2D control + height
                    robot.position = np.array([waypoint[0], waypoint[1]], dtype=np.float32)
                    
                    # Find the best height setting for the target z
                    target_height_index = min(range(len(robot.rotation_options)), 
                                          key=lambda i: abs(0.3 - i*0.05 - waypoint[2]))
                    robot.rotation_index = target_height_index
                    robot.rotation = robot.rotation_options[robot.rotation_index]
                    
                    # Move the arm with collision avoidance
                    robot.move_arm_to_position(robot.position, robot.rotation, check_collisions=True)
                    
                    # If we have a cube attached, move it with the arm
                    if robot.cube_attached and hasattr(robot, 'grabbed_cube_id') and robot.grabbed_cube_id is not None:
                        # Get the suction cup position
                        suction_pos, suction_orn = p.getBasePositionAndOrientation(robot.suction_cup)
                        
                        # Use the stored offsets to maintain the cube's relative position
                        if hasattr(robot, 'cube_offset_x') and hasattr(robot, 'cube_offset_y'):
                            offset_x = robot.cube_offset_x
                            offset_y = robot.cube_offset_y
                        else:
                            # Fallback in case offsets weren't stored
                            offset_x = 0
                            offset_y = 0
                        
                        # Apply the offset to the current suction cup position
                        new_cube_pos = [
                            suction_pos[0] + offset_x,
                            suction_pos[1] + offset_y,
                            suction_pos[2] - 0.05  # Fixed offset below suction cup
                        ]
                        
                        # DIRECTLY set the cube position
                        p.resetBasePositionAndOrientation(robot.grabbed_cube_id, new_cube_pos, suction_orn)
                    
                    # Wait for the arm to reach the waypoint
                    for _ in range(20):  # Increased update frequency for smoother motion
                        p.stepSimulation()
                        time.sleep(0.005)  # Reduced delay for smoother motion
                        
                        # If we have a cube attached, keep updating its position
                        if robot.cube_attached and hasattr(robot, 'grabbed_cube_id') and robot.grabbed_cube_id is not None:
                            # Get the suction cup position
                            suction_pos, suction_orn = p.getBasePositionAndOrientation(robot.suction_cup)
                            
                            # Use the stored offsets to maintain the cube's relative position
                            if hasattr(robot, 'cube_offset_x') and hasattr(robot, 'cube_offset_y'):
                                offset_x = robot.cube_offset_x
                                offset_y = robot.cube_offset_y
                            else:
                                # Fallback in case offsets weren't stored
                                offset_x = 0
                                offset_y = 0
                            
                            # Apply the offset to the current suction cup position
                            new_cube_pos = [
                                suction_pos[0] + offset_x,
                                suction_pos[1] + offset_y,
                                suction_pos[2] - 0.05  # Fixed offset below suction cup
                            ]
                            
                            # DIRECTLY set the cube position
                            p.resetBasePositionAndOrientation(robot.grabbed_cube_id, new_cube_pos, suction_orn)
            
            # Update scene analysis after each cube is moved
            robot.analyze_scene()
        
        # Move back to a neutral position
        neutral_pos = [0.5, 0.0, 0.2]
        print("\nMoving to neutral position")
        robot.position = np.array([neutral_pos[0], neutral_pos[1]], dtype=np.float32)
        target_height_index = min(range(len(robot.rotation_options)), 
                               key=lambda i: abs(0.3 - i*0.05 - neutral_pos[2]))
        robot.rotation_index = target_height_index
        robot.rotation = robot.rotation_options[robot.rotation_index]
        robot.move_arm_to_position(robot.position, robot.rotation, check_collisions=True)
        
        # Restore original physics parameters
        p.setPhysicsEngineParameter(numSolverIterations=previous_solver_iterations)
        
        print("\nDestacking task completed successfully!")
        
    except Exception as e:
        print(f"Error executing destacking task: {e}")
        import traceback
        traceback.print_exc()
