#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import numpy as np
import time
import cv2
import math
import argparse
import os
from factory_environment import FactoryEnvironment
from gearbox_assembly import run_gearbox_assembly, run_disassembly_task
from collections import defaultdict

class UR5ArmSimulation:
    def __init__(self, render_mode="GUI"):
        # Initialize PyBullet
        if render_mode == "GUI":
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        # Setup simulation environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Enable real-time simulation for more accurate physics
        p.setRealTimeSimulation(0)  # 0 = non-real-time (we'll step manually)
        p.setPhysicsEngineParameter(numSolverIterations=50)  # Increased for better stability
        p.setPhysicsEngineParameter(enableConeFriction=1)
        
        # Set up collision detection
        p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
        p.setPhysicsEngineParameter(allowedCcdPenetration=0.0)
        
        # Load ground plane with proper collision properties
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Setup factory environment with textures
        textures_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../textures")
        self.factory_env = FactoryEnvironment(textures_dir)
        
        # Find the assets directory for the UR5 arm
        # First check in the VimaBench directory
        vima_bench_dir = "/home/dth20_04_6/workspace/MRL2025/VimaBench"
        assets_root = os.path.join(vima_bench_dir, "vima_bench", "tasks", "assets")
        
        if not os.path.exists(assets_root):
            # Try to find it in the current directory structure
            current_dir = os.path.dirname(os.path.abspath(__file__))
            assets_root = os.path.join(current_dir, "../..", "VimaBench", "vima_bench", "tasks", "assets")
        
        if not os.path.exists(assets_root):
            # Fall back to pybullet_data
            print("Could not find UR5 assets, using default PyBullet assets")
            assets_root = pybullet_data.getDataPath()
            ur5_urdf_path = "kuka_iiwa/model.urdf"
            workspace_urdf_path = None
        else:
            print(f"Found assets at {assets_root}")
            ur5_urdf_path = os.path.join(assets_root, "ur5/ur5.urdf")
            workspace_urdf_path = os.path.join(assets_root, "ur5/workspace.urdf")
        
        # Load the UR5 robot arm
        if os.path.exists(ur5_urdf_path):
            self.arm_id = p.loadURDF(ur5_urdf_path, [0, 0, 0], useFixedBase=True)
            print(f"Loaded UR5 arm from {ur5_urdf_path}")
        else:
            # Fall back to a default arm if UR5 is not available
            self.arm_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
            print("Falling back to default KUKA arm")
        
        # Load the workspace if available
        if workspace_urdf_path and os.path.exists(workspace_urdf_path):
            self.workspace_id = p.loadURDF(workspace_urdf_path, [0, 0, 0], useFixedBase=True)
            print(f"Loaded workspace from {workspace_urdf_path}")
        
        # Get number of joints
        self.num_joints = p.getNumJoints(self.arm_id)
        self.end_effector_index = self.num_joints - 1  # Last joint is the end effector
        
        # Add a gripper to the end of the arm
        self.setup_gripper()
        
        # Load a square platform and multiple colored cubes
        self.platform_id = None
        self.cubes = {}
        self.load_platform_and_cubes(assets_root)
        
        # Get number of joints
        self.num_joints = p.getNumJoints(self.arm_id)
        print(f"Loaded robot arm with {self.num_joints} joints")
        
        # Print joint info
        self.joint_info = {}
        for i in range(self.num_joints):
            info = p.getJointInfo(self.arm_id, i)
            print(f"Joint {i}: {info[1].decode('utf-8')}")
            self.joint_info[i] = {
                'name': info[1].decode('utf-8'),
                'type': info[2],
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }
        
        # Setup camera for visualization
        self.camera_distance = 1.0
        self.camera_yaw = 50
        self.camera_pitch = -35
        self.camera_target_position = [0, 0, 0.5]
        
        # Enable debug visualization for collision detection
        self.debug_mode = False
        
        # Initialize control variables
        self.reset_arm_position()
        
        # Initialize gripper and cube variables
        self.cube_constraint_id = None
        self.cube_attached = False
        
        # Setup UI for manual control
        self.setup_control_ui()
        
        # Initialize cube control variables
        self.is_cube_selected = False
        self.cube_height = 0.02  # Initial height of the cube
        
        # Scene recognition variables
        self.scene_objects = {}
        self.scene_analyzed = False
        
        # Path planning variables
        self.path_points = []
        self.current_path_index = 0
        self.is_executing_path = False
        self.collision_margin = 0.02  # Margin for collision detection in meters
        
        # Enable collision detection
        self.enable_collision_detection()
        
        # Apply factory environment textures and create walls
        self.setup_factory_environment()
    
    def analyze_scene(self):
        """Analyze the scene to detect objects and their positions"""
        print("\nAnalyzing scene...")
        
        # Initialize scene objects dictionary
        self.scene_objects = {}
        
        # Detect the platform
        if hasattr(self, 'platform_id'):
            platform_pos, platform_orn = p.getBasePositionAndOrientation(self.platform_id)
            self.scene_objects['blue_platform'] = [{
                'id': self.platform_id,
                'position': platform_pos,
                'orientation': platform_orn
            }]
            print(f"blue_platform 1: Position {platform_pos}")
        
        # Detect all the colored cubes
        if hasattr(self, 'cubes'):
            for color, cube_info in self.cubes.items():
                cube_id = cube_info["id"]
                cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                
                # Add to scene objects
                key = f"{color}_cube"
                if key not in self.scene_objects:
                    self.scene_objects[key] = []
                
                self.scene_objects[key].append({
                    'id': cube_id,
                    'position': cube_pos,
                    'orientation': cube_orn
                })
                
                print(f"{key} 1: Position {cube_pos}")
        
        print("Scene Analysis Summary:")
        for obj_type, instances in self.scene_objects.items():
            for i, instance in enumerate(instances):
                print(f"{obj_type} {i+1}: Position {instance['position']}")
        
        # Capture an overhead view of the scene for visualization
        self.capture_scene_image()
        
        return self.scene_objects
    
    def capture_scene_image(self):
        """Capture a top-down view of the scene for visualization"""
        # Save current camera settings
        current_dist = self.camera_distance
        current_yaw = self.camera_yaw
        current_pitch = self.camera_pitch
        current_target = self.camera_target_position
        
        # Set camera to top-down view
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[0.5, 0, 0],
            distance=1.2,
            yaw=0,
            pitch=-90,
            roll=0,
            upAxisIndex=2
        )
        
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60,
            aspect=1.0,
            nearVal=0.1,
            farVal=100.0
        )
        
        # Capture image
        width, height = 320, 320
        img_arr = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Convert to RGB
        rgb = img_arr[2]
        rgb_array = np.array(rgb, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (height, width, 4))
        rgb_array = rgb_array[:, :, :3]  # Remove alpha channel
        
        # Create a copy for drawing
        scene_img = rgb_array.copy()
        
        # Draw object markers on the image
        for obj_type, objects in self.scene_objects.items():
            for obj in objects:
                # Project 3D position to 2D image coordinates
                pos = obj['position']
                
                # Improved projection calculation
                # The workspace is roughly from [0.2, -0.3, 0] to [0.8, 0.3, 0.3]
                x_min, x_max = 0.2, 0.8
                y_min, y_max = -0.3, 0.3
                
                # Map x from [x_min, x_max] to [0, width]
                x = int((pos[0] - x_min) * width / (x_max - x_min))
                # Map y from [y_min, y_max] to [height, 0] (y is inverted in image)
                y = int((y_max - pos[1]) * height / (y_max - y_min))
                
                # Ensure coordinates are within image bounds
                x = max(0, min(width - 1, x))
                y = max(0, min(height - 1, y))
                
                # Determine color based on object type
                if 'red' in obj_type:
                    color = (0, 0, 255)  # BGR format (red)
                elif 'green' in obj_type:
                    color = (0, 255, 0)  # BGR format (green)
                elif 'blue' in obj_type and 'bowl' not in obj_type:
                    color = (255, 0, 0)  # BGR format (blue)
                elif 'yellow' in obj_type:
                    color = (0, 255, 255)  # BGR format (yellow)
                elif 'bowl' in obj_type:
                    color = (255, 0, 0)  # BGR format (blue for bowl)
                else:
                    color = (255, 255, 255)  # BGR format (white)
                
                # Draw marker
                cv2.circle(scene_img, (x, y), 10, color, -1)
                
                # Draw label with contrasting outline for better visibility
                text = obj_type
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                text_x = x - text_size[0] // 2
                text_y = y - 15
                
                # Ensure text is within image bounds
                text_x = max(5, min(width - text_size[0] - 5, text_x))
                text_y = max(15, text_y)
                
                # Draw text with outline
                cv2.putText(scene_img, text, (text_x-1, text_y-1), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.putText(scene_img, text, (text_x, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Add title and timestamp
        cv2.putText(scene_img, "Scene Analysis - Overhead View", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        cv2.putText(scene_img, f"Time: {timestamp}", (10, height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add grid lines for reference
        grid_color = (100, 100, 100)
        grid_step = 40  # pixels
        for i in range(0, width, grid_step):
            cv2.line(scene_img, (i, 0), (i, height), grid_color, 1)
        for j in range(0, height, grid_step):
            cv2.line(scene_img, (0, j), (width, j), grid_color, 1)
        
        # Add coordinate system reference
        cv2.putText(scene_img, "X+", (width-30, height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(scene_img, "Y+", (width//2, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Display the scene image
        cv2.imshow("Scene Analysis", scene_img)
        
        # Restore camera settings
        self.camera_distance = current_dist
        self.camera_yaw = current_yaw
        self.camera_pitch = current_pitch
        self.camera_target_position = current_target
    
    def load_platform_and_cubes(self, assets_root):
        """Load a square platform and multiple colored cubes into the scene"""
        try:
            # Create a square platform for stacking instead of using the bowl
            platform_size = [0.15, 0.15, 0.01]  # Size of the platform (width, length, height)
            platform_position = [0.6, 0.0, 0.005]  # Position of the platform
            platform_color = [0.2, 0.2, 0.8, 1]  # Blue color
            
            # Create a visual shape for the platform
            platform_visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[platform_size[0]/2, platform_size[1]/2, platform_size[2]/2],
                rgbaColor=platform_color
            )
            
            # Create a collision shape for the platform
            platform_collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[platform_size[0]/2, platform_size[1]/2, platform_size[2]/2]
            )
            
            # Create the platform as a multibody
            self.platform_id = p.createMultiBody(
                baseMass=0,  # Mass of 0 makes it static
                baseCollisionShapeIndex=platform_collision_shape,
                baseVisualShapeIndex=platform_visual_shape,
                basePosition=platform_position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
            )
            
            print(f"Created square platform at position {platform_position}")
            
            # Increase solver iterations for better physics stability
            p.setPhysicsEngineParameter(numSolverIterations=100)
            
            # Set cube properties for better physics
            cube_mass = 0.05  # Reduced mass for better stability (50 grams)
            cube_lateral_friction = 0.9  # High friction to prevent sliding
            cube_spinning_friction = 0.3  # High spinning friction to prevent rotation
            cube_rolling_friction = 0.3  # High rolling friction to prevent rolling
            cube_restitution = 0.01  # Very low bounciness
            
            # Load the cube URDF
            cube_urdf_path = os.path.join(assets_root, "block/block.urdf")
            if not os.path.exists(cube_urdf_path):
                print(f"Could not find cube URDF at {cube_urdf_path}")
                return
                
            # Create multiple cubes with different colors and positions
            self.cubes = {}  # Dictionary to store cube IDs and their colors
            
            # Define cube colors and positions
            cube_configs = [
                {"color": "red", "position": [0.5, 0.0, 0.05], "rgba": [1, 0, 0, 1]},
                {"color": "green", "position": [0.5, 0.15, 0.05], "rgba": [0, 1, 0, 1]},
                {"color": "blue", "position": [0.5, -0.15, 0.05], "rgba": [0, 0, 1, 1]},
                {"color": "yellow", "position": [0.35, 0.0, 0.05], "rgba": [1, 1, 0, 1]}
            ]
            
            # Load each cube
            for config in cube_configs:
                # Add small random offset for more natural placement
                rand_x = np.random.uniform(-0.01, 0.01)
                rand_y = np.random.uniform(-0.01, 0.01)
                position = [
                    config["position"][0] + rand_x,
                    config["position"][1] + rand_y,
                    config["position"][2]
                ]
                
                # Load the cube
                cube_id = p.loadURDF(
                    cube_urdf_path,
                    position,
                    p.getQuaternionFromEuler([0, 0, 0])  # Perfect alignment
                )
                
                # Change the color of the cube
                p.changeVisualShape(cube_id, -1, rgbaColor=config["rgba"])
                
                # Set physical properties for the cube
                p.changeDynamics(
                    cube_id, 
                    -1,  # Base link
                    mass=cube_mass,
                    lateralFriction=cube_lateral_friction,
                    spinningFriction=cube_spinning_friction,
                    rollingFriction=cube_rolling_friction,
                    restitution=cube_restitution,
                    linearDamping=0.5,  # Higher damping to reduce oscillations
                    angularDamping=0.8,  # Higher angular damping to prevent spinning
                    contactStiffness=20000,  # Stiffer contacts for stability
                    contactDamping=200,  # Higher contact damping
                    frictionAnchor=1  # Enable friction anchors for more stable friction
                )
                
                # Store the cube in our dictionary
                self.cubes[config["color"]] = {
                    "id": cube_id,
                    "color": config["color"],
                    "position": position
                }
                
                print(f"Loaded {config['color']} cube at position {position}")
                
            # Store the red cube ID separately for backward compatibility
            self.cube_id = self.cubes["red"]["id"]
            
            # IMPORTANT: Ensure proper collision handling between all cubes
            print("Setting up proper collision handling between cubes...")
            for color1, cube_info1 in self.cubes.items():
                for color2, cube_info2 in self.cubes.items():
                    if color1 != color2:
                        # Enable collision between this pair of cubes
                        p.setCollisionFilterPair(
                            cube_info1["id"], cube_info2["id"], 
                            -1, -1, 
                            1  # 1 = enable collision
                        )
                        print(f"Enabled collision between {color1} and {color2} cubes")
            
            # Let the cubes settle with gravity
            for _ in range(100):  # Increased settling time
                p.stepSimulation()
                time.sleep(0.01)
                
                # Check and correct any cubes that might be on their edges
                for color, cube_info in self.cubes.items():
                    cube_id = cube_info["id"]
                    pos, orn = p.getBasePositionAndOrientation(cube_id)
                    euler = p.getEulerFromQuaternion(orn)
                    
                    # If cube is not flat (has significant roll or pitch)
                    if abs(euler[0]) > 0.1 or abs(euler[1]) > 0.1:
                        # Apply a small random impulse to tip it over onto a face
                        rand_force = [np.random.uniform(-0.1, 0.1), np.random.uniform(-0.1, 0.1), -0.5]
                        p.applyExternalForce(
                            objectUniqueId=cube_id,
                            linkIndex=-1,
                            forceObj=rand_force,
                            posObj=[pos[0], pos[1], pos[2] + 0.025],
                            flags=p.WORLD_FRAME
                        )
                
        except Exception as e:
            print(f"Error loading platform and cubes: {e}")
    
    def run_stacking_task(self):
        """Run a task to stack the colored cubes on top of each other"""
        print("\nInitiating Cube Stacking Task...")
        
        try:
            # Import the stacking task module
            import stacking_task
            
            # Run the stacking task
            stacking_task.run_stacking_task(self)
            
        except Exception as e:
            print(f"Error executing stacking task: {e}")
            import traceback
            traceback.print_exc()
    
    def run_destacking_task(self):
        """Run a task to destack the colored cubes and return them to their original positions"""
        print("\nInitiating Cube Destacking Task...")
        
        try:
            # Import the stacking task module
            import stacking_task
            
            # Run the destacking task
            stacking_task.run_destacking_task(self)
            
        except Exception as e:
            print(f"Error executing destacking task: {e}")
            import traceback
            traceback.print_exc()
            
    
    def run_simulation(self):
        """Main simulation loop"""
        print("\nUR5 Robotic Arm Control")
        print("Use the control window buttons or keyboard to move the robot arm")
        print("UP/DOWN/LEFT/RIGHT: Move in XY plane (arrow keys or W/A/S/D)")
        print("HEIGHT+/HEIGHT-: Fine control of arm height (Page Up/Down keys)")
        print("ROTATION: Cycle through different arm orientations (R key)")
        print("STEP+/STEP-: Adjust movement step size (+ and - keys)")
        print("GRIP: Toggle gripper (G key)")
        print("STABILIZE: Reset to a stable position (X key)")
        print("RESET: Reset arm to default position (Space key)")
        print("\nCube Controls:")
        print("CUBE_UP/DOWN/LEFT/RIGHT: Move cube in XY plane (I/K/J/L keys)")
        print("CUBE_FORWARD/BACKWARD: Move cube up/down (U/O keys)")
        print("\nCamera Controls:")
        print("CAM_ZOOM_IN/OUT: Adjust camera zoom ([ and ] keys)")
        print("CAM_YAW_LEFT/RIGHT: Rotate camera left/right (< and > keys)")
        print("CAM_PITCH_UP/DOWN: Adjust camera pitch (^ and v keys)")
        print("\nTask Controls:")
        print("RUN_VIMA: Execute pick and place task (V key)")
        print("STACK: Execute cube stacking task (S key)")
        print("ANALYZE: Perform scene analysis (Z key)")
        print("\nDebug Controls:")
        print("B: Toggle debug mode with collision detection (currently OFF)")
        print("\nPress 'Q' to quit")
        
        # Initial scene analysis
        self.analyze_scene()
        
        # Frame counter for periodic scene analysis
        frame_counter = 0
        scene_analysis_interval = 30  # Analyze scene every 30 frames
        
        running = True
        while running:
            # Update physics simulation
            p.stepSimulation()
            
            # Update camera view
            p.resetDebugVisualizerCamera(
                cameraDistance=self.camera_distance,
                cameraYaw=self.camera_yaw,
                cameraPitch=self.camera_pitch,
                cameraTargetPosition=self.camera_target_position
            )
            
            # Update gripper position
            self.update_gripper_position()
            
            # Periodically update scene analysis
            frame_counter += 1
            if frame_counter >= scene_analysis_interval:
                self.analyze_scene()
                frame_counter = 0
            
            # Update UI
            self.update_status_display()
            
            # Process keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                running = False
            elif key == ord('v'):
                self.run_vima_task()
            elif key == ord('s'):
                self.run_stacking_task()
            elif key == ord('d'):
                self.run_destacking_task()
            elif key == ord('z'):
                self.analyze_scene()
            elif key == ord('b'):
                self.debug_mode = not self.debug_mode
                print(f"Debug mode: {'ON' if self.debug_mode else 'OFF'}")
            
            # Process button clicks
            if self.param['clicked'] is not None:
                clicked = self.param['clicked']
                
                # Update position based on button click
                if clicked == 'UP':
                    self.position[1] += self.step_size
                elif clicked == 'DOWN':
                    self.position[1] -= self.step_size
                elif clicked == 'LEFT':
                    self.position[0] -= self.step_size
                elif clicked == 'RIGHT':
                    self.position[0] += self.step_size
                elif clicked == 'HEIGHT+':
                    self.rotation_index = max(self.rotation_index - 1, 0)
                    self.rotation = self.rotation_options[self.rotation_index]
                    print(f"Setting height to level {self.rotation_index}/6")
                elif clicked == 'HEIGHT-':
                    self.rotation_index = min(self.rotation_index + 1, len(self.rotation_options) - 1)
                    self.rotation = self.rotation_options[self.rotation_index]
                    print(f"Setting height to level {self.rotation_index}/6")
                elif clicked == 'ROTATION':
                    # Cycle through rotation options
                    self.rotation_index = (self.rotation_index + 1) % len(self.rotation_options)
                    self.rotation = self.rotation_options[self.rotation_index]
                    print(f"Cycling rotation to option {self.rotation_index}/6")
                elif clicked == 'STEP+':
                    self.step_size = min(self.step_size + 0.01, 0.2)
                    print(f"Increased step size to {self.step_size:.3f}")
                elif clicked == 'STEP-':
                    self.step_size = max(self.step_size - 0.01, 0.01)
                    print(f"Decreased step size to {self.step_size:.3f}")
                elif clicked == 'GRIP':
                    # Toggle gripper
                    self.toggle_gripper()
                elif clicked == 'STABILIZE':
                    # Reset to the stable arm position
                    self.reset_arm_position()
                    print("Stabilizing arm position...")
                elif clicked == 'RESET':
                    # Reset the arm to the default position
                    self.reset_arm_position()
                    self.position = np.array([0.5, 0.0], dtype=np.float32)
                    self.rotation_index = 2
                    self.rotation = self.rotation_options[self.rotation_index]
                    print("Reset arm to default position")
                # Cube control buttons
                elif clicked.startswith('CUBE_'):
                    direction = clicked.replace('CUBE_', '')
                    self.move_cube(direction)
                # Camera control buttons
                elif clicked.startswith('CAM_'):
                    action = clicked.replace('CAM_', '')
                    self.adjust_camera(action)
                # Task execution buttons
                elif clicked == 'RUN_VIMA':
                    self.run_vima_task()
                elif clicked == 'STACK':
                    print("Starting stacking task from button press...")
                    self.run_stacking_task()
                elif clicked == 'DESTACK':
                    print("Starting destacking task from button press...")
                    self.run_destacking_task()
                
                # Clamp position to action space boundaries
                self.position[0] = np.clip(self.position[0], self.x_min, self.x_max)
                self.position[1] = np.clip(self.position[1], self.y_min, self.y_max)
                
                # Move the arm to the new position with collision avoidance
                self.move_arm_to_position(self.position, self.rotation, check_collisions=True)
                
                # Print current position
                ee_pos = self.get_end_effector_position()
                print(f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}]")
                print(f"End effector: [{ee_pos[0]:.2f}, {ee_pos[1]:.2f}, {ee_pos[2]:.2f}]")
                
                # Reset clicked button
                self.param['clicked'] = None
            
            # Small delay to prevent CPU overuse
            time.sleep(0.01)
        
        # Clean up
        cv2.destroyAllWindows()
        p.disconnect()

    def run_vima_task(self):
        """Run the VIMA model to complete a scene arrangement task"""
        print("\nInitiating VIMA task execution...")
        
        # First, analyze the scene to locate objects
        self.analyze_scene()
        
        try:
            # This is a placeholder for the actual VIMA model integration
            # In a real implementation, you would:
            # 1. Import the VIMA model
            # 2. Set up the task parameters
            # 3. Run the model to generate actions
            # 4. Execute the actions with the robotic arm
            
            # For demonstration, we'll simulate a simple pick and place task
            print("VIMA Task: Pick up the cube and place it in the basket")
            
            # Get current positions from scene analysis
            if 'red_cube' not in self.scene_objects or not self.scene_objects['red_cube']:
                print("Error: Red cube not found in scene analysis")
                return
                
            if 'blue_platform' not in self.scene_objects or not self.scene_objects['blue_platform']:
                print("Error: Blue platform not found in scene analysis")
                return
            
            cube_info = self.scene_objects['red_cube'][0]
            platform_info = self.scene_objects['blue_platform'][0]
            
            cube_pos = cube_info['position']
            platform_pos = platform_info['position']
            
            print(f"Detected cube at: {cube_pos}")
            print(f"Detected platform at: {platform_pos}")
            
            # Create a VIMA-like prompt
            vima_prompt = {
                "task": "pick_and_place",
                "objects": {
                    "cube": {"position": cube_pos, "color": "red"},
                    "platform": {"position": platform_pos, "color": "blue"}
                },
                "instruction": "Pick up the red cube and place it on the blue platform"
            }
            
            print(f"VIMA Prompt: {vima_prompt}")
            
            # Define waypoints for the task based on detected positions
            waypoints = [
                # Move above the cube
                (cube_pos[0], cube_pos[1], 0.2),
                # Move down to the cube
                (cube_pos[0], cube_pos[1], cube_pos[2] + 0.03),  # Get closer to the cube
                # Close gripper (simulated by changing cube position)
                None,
                # Lift the cube
                (cube_pos[0], cube_pos[1], 0.2),
                # Move to the platform position
                (platform_pos[0], platform_pos[1], 0.2),
                # Lower into the platform
                (platform_pos[0], platform_pos[1], platform_pos[2] + 0.05),
                # Open gripper (simulated by releasing the cube)
                None,
                # Move up
                (platform_pos[0], platform_pos[1], 0.2),
                # Return to a neutral position
                (0.5, 0.0, 0.2)
            ]
            
            # Execute the waypoints with collision avoidance
            print("Executing VIMA-generated plan with collision avoidance...")
            
            # Create a constraint for attaching the cube to the gripper
            self.cube_constraint_id = None
            self.cube_attached = False
            
            for i, waypoint in enumerate(waypoints):
                if waypoint is None:
                    if i == 2:  # Close gripper
                        print("Closing gripper")
                        # Visually close the gripper
                        self.toggle_gripper()
                        
                        # Wait for the gripper to close
                        for _ in range(20):
                            p.stepSimulation()
                            time.sleep(0.01)
                        
                        # Only create a constraint if we don't already have one
                        if not self.cube_attached and self.cube_constraint_id is None:
                            # Create a constraint to attach the cube to the suction cup
                            suction_pos, suction_orn = p.getBasePositionAndOrientation(self.suction_cup)
                            cube_pos, cube_orn = p.getBasePositionAndOrientation(self.cube_id)
                            
                            # Check if the cube is close enough to grab
                            distance = np.linalg.norm(np.array(suction_pos) - np.array(cube_pos))
                            
                            if distance < 0.15:  # If close enough
                                # Make the constraint
                                self.cube_constraint_id = p.createConstraint(
                                    parentBodyUniqueId=self.suction_cup,
                                    parentLinkIndex=-1,
                                    childBodyUniqueId=self.cube_id,
                                    childLinkIndex=-1,
                                    jointType=p.JOINT_FIXED,
                                    jointAxis=[0, 0, 0],
                                    parentFramePosition=[0, 0, 0],
                                    childFramePosition=[0, 0, 0]
                                )
                                self.cube_attached = True
                                print("Cube grabbed with suction cup")
                                
                                # Show a visual effect for the suction
                                p.changeVisualShape(self.suction_cup, -1, rgbaColor=[0.9, 0.2, 0.2, 1])  # Red when active
                            else:
                                print("Failed to grab cube - too far away")
                        
                        # Wait a bit for physics to settle
                        for _ in range(20):
                            p.stepSimulation()
                            time.sleep(0.01)
                    elif i == 6:  # Open gripper
                        print("Opening gripper")
                        # Visually open the gripper
                        self.toggle_gripper()
                        
                        # Reset suction cup color
                        p.changeVisualShape(self.suction_cup, -1, rgbaColor=[0.2, 0.2, 0.2, 1])
                        
                        # Wait for the gripper to open
                        for _ in range(20):
                            p.stepSimulation()
                            time.sleep(0.01)
                        
                        # Release the cube by removing the constraint
                        if self.cube_constraint_id is not None:
                            # Get the current position of the cube before removing constraint
                            cube_pos, cube_orn = p.getBasePositionAndOrientation(self.cube_id)
                            
                            # Remove the constraint
                            p.removeConstraint(self.cube_constraint_id)
                            self.cube_constraint_id = None
                            self.cube_attached = False
                            print("Cube released")
                            
                            # Apply a small impulse to help it fall into the platform
                            p.applyExternalForce(
                                objectUniqueId=self.cube_id,
                                linkIndex=-1,
                                forceObj=[0, 0, -1],  # Small downward force
                                posObj=cube_pos,
                                flags=p.WORLD_FRAME
                            )
                        
                        # Wait for the cube to fall into the platform naturally
                        for _ in range(50):
                            p.stepSimulation()
                            time.sleep(0.01)
                else:
                    # Move arm to the waypoint with collision avoidance
                    x, y, z = waypoint
                    print(f"Moving to waypoint {i+1}/{len(waypoints)}: [{x:.2f}, {y:.2f}, {z:.2f}]")
                    
                    # Convert 3D position to our 2D control + height index
                    self.position = np.array([x, y], dtype=np.float32)
                    
                    # Find the best height setting for the target z
                    target_height_index = min(range(len(self.rotation_options)), 
                                          key=lambda i: abs(0.3 - i*0.05 - z))
                    self.rotation_index = target_height_index
                    self.rotation = self.rotation_options[self.rotation_index]
                    
                    # Move the arm with collision avoidance
                    success = self.move_arm_to_position(self.position, self.rotation, check_collisions=True)
                    
                    if not success:
                        print("Warning: Could not reach waypoint due to collision. Trying alternative approach...")
                        # Try a higher position first
                        temp_height_index = max(0, self.rotation_index - 1)
                        temp_rotation = self.rotation_options[temp_height_index]
                        success = self.move_arm_to_position(self.position, temp_rotation, check_collisions=True)
                        
                        if success:
                            # Now try the original height
                            self.rotation_index = target_height_index
                            self.rotation = self.rotation_options[self.rotation_index]
                            success = self.move_arm_to_position(self.position, self.rotation, check_collisions=True)
                    
                    # Wait a bit to simulate movement time - use smaller steps for smoother motion
                    steps = 30  # More steps for smoother motion
                    for _ in range(steps):
                        p.stepSimulation()
                        time.sleep(0.01)
            
            print("VIMA task completed successfully!")
            
            # Update scene analysis after task completion
            self.analyze_scene()
            
        except Exception as e:
            print(f"Error executing VIMA task: {e}")
            import traceback
            traceback.print_exc()
    
    def setup_control_ui(self):
        """Create a window with control buttons for manual control"""
        self.window_name = "UR5 Arm Controls"
        cv2.namedWindow(self.window_name)
        
        # Create a black image for the control panel - increased height to accommodate gearbox buttons
        self.control_panel = np.zeros((750, 300, 3), np.uint8)
        
        # Define button positions and labels
        button_height = 30
        button_width = 80
        button_margin = 10
        
        # Movement buttons
        self.buttons = {
            'UP': (110, 50, button_width, button_height),
            'DOWN': (110, 130, button_width, button_height),
            'LEFT': (20, 90, button_width, button_height),
            'RIGHT': (200, 90, button_width, button_height),
            'HEIGHT+': (20, 190, button_width, button_height),
            'HEIGHT-': (110, 190, button_width, button_height),
            'ROTATION': (200, 190, button_width, button_height),
            'STEP+': (20, 250, button_width, button_height),
            'STEP-': (110, 250, button_width, button_height),
            'GRIP': (200, 250, button_width, button_height),
            'STABILIZE': (20, 310, button_width, button_height),
            'RESET': (110, 310, button_width, button_height),
            
            # Cube control buttons
            'CUBE_UP': (110, 370, button_width, button_height),
            'CUBE_DOWN': (110, 430, button_width, button_height),
            'CUBE_LEFT': (20, 400, button_width, button_height),
            'CUBE_RIGHT': (200, 400, button_width, button_height),
            'CUBE_FORWARD': (20, 460, button_width, button_height),
            'CUBE_BACKWARD': (110, 460, button_width, button_height),
            
            # Camera control buttons
            'CAM_ZOOM_IN': (20, 520, button_width, button_height),
            'CAM_ZOOM_OUT': (110, 520, button_width, button_height),
            'CAM_YAW_LEFT': (20, 560, button_width, button_height),
            'CAM_YAW_RIGHT': (110, 560, button_width, button_height),
            'CAM_PITCH_UP': (200, 520, button_width, button_height),
            'CAM_PITCH_DOWN': (200, 560, button_width, button_height),
            
            # Task execution buttons
            'RUN_VIMA': (20, 620, button_width, button_height),
            'STACK': (110, 620, button_width, button_height),
            'DESTACK': (200, 620, button_width, button_height),
            
            # Gearbox assembly buttons
            'ASSEMBLE': (20, 670, button_width, button_height),
            'DISASSEMBLE': (110, 670, button_width, button_height),
        }
        
        # Draw buttons
        for label, (x, y, w, h) in self.buttons.items():
            cv2.rectangle(self.control_panel, (x, y), (x+w, y+h), (100, 100, 100), -1)
            cv2.putText(self.control_panel, label, (x+10, y+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow(self.window_name, self.control_panel)
        
        # Setup mouse callback
        self.param = {'buttons': self.buttons, 'clicked': None}
        cv2.setMouseCallback(self.window_name, self.get_clicked_button, self.param)
        
        # Initialize control parameters - similar to fight_interference.py
        self.position = np.array([0.5, 0.0], dtype=np.float32)  # x: [0.25, 0.75], y: [-0.5, 0.5]
        
        # Define multiple rotation options for different arm heights/orientations
        self.rotation_options = [
            # All of these quaternions are normalized to unit length for stability
            np.array([0.9239, 0.0, 0.3827, 0.0], dtype=np.float32),  # Highest
            np.array([0.8660, 0.0, 0.5000, 0.0], dtype=np.float32),  # High
            np.array([0.7071, 0.0, 0.7071, 0.0], dtype=np.float32),  # Medium-high
            np.array([0.5000, 0.0, 0.8660, 0.0], dtype=np.float32),  # Medium
            np.array([0.3827, 0.0, 0.9239, 0.0], dtype=np.float32),  # Medium-low
            np.array([0.2588, 0.0, 0.9659, 0.0], dtype=np.float32),  # Low
            np.array([0.1305, 0.0, 0.9914, 0.0], dtype=np.float32),  # Lowest
        ]
        self.rotation_index = 2  # Start with a stable medium-high position
        self.rotation = self.rotation_options[self.rotation_index]
        
        # Control parameters
        self.step_size = 0.05  # Step size for movement
        self.gripper_state = False  # Closed = False, Open = True
        
        # Action space boundaries
        self.x_min, self.y_min = 0.25, -0.5
        self.x_max, self.y_max = 0.75, 0.5
        
        # Print action space boundaries
        print(f"Action space boundaries:")
        print(f"X: [{self.x_min}, {self.x_max}]")
        print(f"Y: [{self.y_min}, {self.y_max}]")

    def get_clicked_button(self, event, x, y, flags, param):
        """Handle mouse clicks on control buttons"""
        if event == cv2.EVENT_LBUTTONDOWN:
            buttons = param['buttons']
            for label, (bx, by, bw, bh) in buttons.items():
                if bx <= x <= bx+bw and by <= y <= by+bh:
                    param['clicked'] = label
                    return

    def get_end_effector_position(self):
        """Get the current position of the end effector"""
        # For UR5, the end effector is typically the last link
        end_effector_index = self.num_joints - 1
        state = p.getLinkState(self.arm_id, end_effector_index)
        return state[0]  # Position (x, y, z)
    
    def move_arm_to_position(self, position, rotation, check_collisions=True):
        """Move the arm to the specified position and rotation using inverse kinematics"""
        # For UR5, we need to use inverse kinematics to move to a position
        end_effector_index = 7  # wrist_3_joint for UR5
        
        # Convert quaternion to Euler angles for better control
        # This is a simplification - in a real scenario you might want to use the quaternion directly
        orientation = p.getQuaternionFromEuler([0, rotation[2], 0])
        
        # Calculate the target position in 3D space
        # Z coordinate is fixed based on the rotation index (height)
        z_height = 0.2 + (0.3 * (1.0 - (self.rotation_index / 6.0)))  # Higher index = lower height
        target_position = [position[0], position[1], z_height]
        
        # If path planning is enabled, plan a collision-free path
        if check_collisions and self.debug_mode:  # Only use path planning in debug mode
            # Get current joint positions
            current_joint_positions = [p.getJointState(self.arm_id, i)[0] for i in range(self.num_joints)]
            
            # Use inverse kinematics to get target joint positions
            target_joint_positions = p.calculateInverseKinematics(
                self.arm_id,
                end_effector_index,
                target_position,
                orientation,
                maxNumIterations=100
            )
            
            # Plan a path from current to target joint positions
            path = self.plan_joint_path(current_joint_positions, target_joint_positions)
            
            # Execute the path
            if path:
                self.execute_joint_path(path)
                return True
            else:
                print("Warning: Could not find collision-free path to target position")
                # Fall back to direct movement
        
        # If no collision checking or path planning failed, use direct IK
        joint_positions = p.calculateInverseKinematics(
            self.arm_id,
            end_effector_index,
            target_position,
            orientation,
            maxNumIterations=100
        )
        
        # Set the joint positions - only for the actual controllable joints (2-7 for UR5)
        # UR5 has 6 controllable joints (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
        controllable_joints = [2, 3, 4, 5, 6, 7]  # These are the actual movable joints for UR5
        
        for i, joint_idx in enumerate(controllable_joints):
            if joint_idx < self.num_joints and self.joint_info[joint_idx]['type'] == p.JOINT_REVOLUTE:
                # Make sure we don't go out of bounds in the joint_positions array
                if i < len(joint_positions):
                    p.setJointMotorControl2(
                        bodyIndex=self.arm_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=joint_positions[i],
                        force=self.joint_info[joint_idx]['max_force'],
                        maxVelocity=self.joint_info[joint_idx]['max_velocity']
                    )
        
        # Step simulation a few times to allow the arm to move
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.01)
        
        return True
    
    def plan_joint_path(self, start_joints, target_joints, max_iterations=50):
        """Plan a collision-free path in joint space using a simple RRT-like algorithm"""
        print("Planning collision-free path...")
        
        # Only consider the controllable joints (2-7 for UR5)
        controllable_joints = [2, 3, 4, 5, 6, 7]
        
        # Extract the relevant joint positions
        start_config = []
        target_config = []
        
        for joint_idx in controllable_joints:
            if joint_idx < len(start_joints):
                start_config.append(start_joints[joint_idx])
            if joint_idx < len(target_joints):
                target_config.append(target_joints[joint_idx])
        
        # Ensure we have the same number of joints
        if len(start_config) != len(target_config):
            print(f"Error: Joint configuration dimension mismatch: {len(start_config)} vs {len(target_config)}")
            # Fall back to direct movement
            return None
        
        # Save the current joint positions
        original_positions = [p.getJointState(self.arm_id, i)[0] for i in range(self.num_joints)]
        
        # Initialize the path with the start configuration
        path = [start_config]
        
        # Try to connect directly to the target
        if self.check_segment(start_config, target_config, controllable_joints):
            path.append(target_config)
            
            # Restore original positions
            for i, pos in zip(range(self.num_joints), original_positions):
                p.resetJointState(self.arm_id, i, pos)
            
            print("Direct path found!")
            return path
        
        # Simple RRT-like algorithm
        for _ in range(max_iterations):
            # With some probability, try to extend toward the goal
            if np.random.random() < 0.2:
                sample = target_config
            else:
                # Generate a random configuration
                sample = []
                for i, joint_idx in enumerate(controllable_joints):
                    if joint_idx < self.num_joints:
                        lower = self.joint_info[joint_idx]['lower_limit']
                        upper = self.joint_info[joint_idx]['upper_limit']
                        if lower > upper:  # Handle invalid limits
                            lower, upper = -3.14, 3.14
                        sample.append(np.random.uniform(lower, upper))
            
            # Find the nearest node in the path
            distances = [self.joint_distance(config, sample) for config in path]
            nearest_idx = np.argmin(distances)
            nearest = path[nearest_idx]
            
            # Generate a new configuration by stepping toward the sample
            step_size = 0.2  # Adjust as needed
            direction = np.array(sample) - np.array(nearest)
            distance = np.linalg.norm(direction)
            if distance > step_size:
                direction = direction / distance * step_size
            new_config = list(np.array(nearest) + direction)
            
            # Check if the segment is collision-free
            if self.check_segment(nearest, new_config, controllable_joints):
                path.append(new_config)
                
                # Try to connect to the target from the new node
                if self.check_segment(new_config, target_config, controllable_joints):
                    path.append(target_config)
                    
                    # Restore original positions
                    for i, pos in zip(range(self.num_joints), original_positions):
                        p.resetJointState(self.arm_id, i, pos)
                    
                    print(f"Path found with {len(path)} waypoints")
                    return path
        
        # Restore original positions
        for i, pos in zip(range(self.num_joints), original_positions):
            p.resetJointState(self.arm_id, i, pos)
        
        print("Path planning failed after maximum iterations")
        return None
    
    def check_segment(self, config1, config2, controllable_joints, steps=5):
        """Check if a path segment between two configurations is collision-free"""
        for t in np.linspace(0, 1, steps):
            # Interpolate between configurations
            config = [c1 * (1 - t) + c2 * t for c1, c2 in zip(config1, config2)]
            
            # Set the joint positions
            for i, joint_idx in enumerate(controllable_joints):
                if i < len(config) and joint_idx < self.num_joints:
                    p.resetJointState(self.arm_id, joint_idx, config[i])
            
            # Step simulation to update the physics
            p.stepSimulation()
            
            # Check for collisions
            if self.check_collision():
                return False
        
        return True
    
    def joint_distance(self, config1, config2):
        """Calculate the Euclidean distance between two joint configurations"""
        return np.linalg.norm(np.array(config1) - np.array(config2))
    
    def execute_joint_path(self, path, speed=0.5):
        """Execute a planned joint path"""
        controllable_joints = [2, 3, 4, 5, 6, 7]  # These are the actual movable joints for UR5
        
        for i, config in enumerate(path):
            print(f"Executing waypoint {i+1}/{len(path)}")
            
            # Set the joint positions
            for j, joint_idx in enumerate(controllable_joints):
                if j < len(config) and joint_idx < self.num_joints:
                    p.setJointMotorControl2(
                        bodyIndex=self.arm_id,
                        jointIndex=joint_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=config[j],
                        force=self.joint_info[joint_idx]['max_force'],
                        maxVelocity=self.joint_info[joint_idx]['max_velocity'] * speed
                    )
            
            # Wait for the arm to reach the position
            for _ in range(int(20 / speed)):  # Adjust based on speed
                p.stepSimulation()
                time.sleep(0.01)
                
                # If cube is attached, move it with the end effector
                if hasattr(self, 'cube_attached') and self.cube_attached:
                    ee_pos = self.get_end_effector_position()
                    p.resetBasePositionAndOrientation(
                        self.cube_id, 
                        [ee_pos[0], ee_pos[1], ee_pos[2] - 0.05], 
                        [0, 0, 0, 1]
                    )
    
    def move_cube(self, direction):
        """Move the cube in the specified direction"""
        if self.cube_id is None:
            print("No cube to move")
            return
        
        # Get current position and orientation
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.cube_id)
        
        # Calculate new position based on direction
        new_pos = list(cube_pos)
        
        if direction == "UP":
            new_pos[1] += self.step_size
        elif direction == "DOWN":
            new_pos[1] -= self.step_size
        elif direction == "LEFT":
            new_pos[0] -= self.step_size
        elif direction == "RIGHT":
            new_pos[0] += self.step_size
        elif direction == "FORWARD":
            new_pos[2] += self.step_size
        elif direction == "BACKWARD":
            new_pos[2] = max(0.02, new_pos[2] - self.step_size)  # Don't go below the table
        
        # Set the new position
        p.resetBasePositionAndOrientation(self.cube_id, new_pos, cube_orn)
        print(f"Moved cube to [{new_pos[0]:.2f}, {new_pos[1]:.2f}, {new_pos[2]:.2f}]")
    
    def adjust_camera(self, action):
        """Adjust the camera view based on the action"""
        if action == "ZOOM_IN":
            self.camera_distance = max(0.5, self.camera_distance - 0.1)
        elif action == "ZOOM_OUT":
            self.camera_distance = min(3.0, self.camera_distance + 0.1)
        elif action == "YAW_LEFT":
            self.camera_yaw = (self.camera_yaw - 5) % 360
        elif action == "YAW_RIGHT":
            self.camera_yaw = (self.camera_yaw + 5) % 360
        elif action == "PITCH_UP":
            self.camera_pitch = min(0, self.camera_pitch + 5)
        elif action == "PITCH_DOWN":
            self.camera_pitch = max(-90, self.camera_pitch - 5)
            
        print(f"Camera adjusted: Dist={self.camera_distance:.1f}, Yaw={self.camera_yaw:.1f}, Pitch={self.camera_pitch:.1f}")

    def enable_collision_detection(self):
        """Enable collision detection between all objects"""
        # Enable collision detection between all objects
        for i in range(p.getNumBodies()):
            for j in range(i + 1, p.getNumBodies()):
                p.setCollisionFilterPair(i, j, -1, -1, 1)
        
        if self.platform_id is not None:
            for i in range(self.num_joints):
                p.setCollisionFilterPair(self.arm_id, self.platform_id, i, -1, 1)
        
        # Enable collision detection with the ground plane
        for i in range(self.num_joints):
            p.setCollisionFilterPair(self.arm_id, self.plane_id, i, -1, 1)
        
        print("Collision detection enabled")
        
    def setup_factory_environment(self):
        """Apply factory textures and create factory environment"""
        print("\nSetting up factory environment...")
        
        # Load textures
        if self.factory_env.load_textures():
            # Apply floor texture
            self.factory_env.apply_texture_to_plane(self.plane_id)
            
            # Apply platform texture if platform exists
            if hasattr(self, 'platform_id') and self.platform_id is not None:
                self.factory_env.apply_texture_to_platform(self.platform_id)
            
            # Apply robot texture to the arm
            self.factory_env.apply_robot_texture(self.arm_id)
            
            # Create factory walls
            self.factory_env.create_factory_walls()
            
            # Add factory lighting
            self.factory_env.add_factory_lights()
            
            # Add some industrial props
            self.add_industrial_props()
            
            print("Factory environment setup complete!")
        else:
            print("Failed to load factory textures. Using default environment.")
            
        # Step simulation a few times to let everything settle
        for _ in range(10):
            p.stepSimulation()
            
    def add_industrial_props(self):
        """Add industrial props to the environment to make it look like a factory"""
        # Add a workbench/table next to the robot
        table_size = [0.4, 0.6, 0.02]  # Size of the table (width, length, height)
        table_legs_height = 0.4  # Height of the table legs
        table_position = [0.0, 0.6, table_legs_height/2]  # Position of the table
        
        # Create table top
        table_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[table_size[0]/2, table_size[1]/2, table_size[2]/2],
            rgbaColor=[0.6, 0.6, 0.6, 1.0]  # Gray color
        )
        
        table_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[table_size[0]/2, table_size[1]/2, table_size[2]/2]
        )
        
        table_id = p.createMultiBody(
            baseMass=0,  # Static
            baseCollisionShapeIndex=table_collision,
            baseVisualShapeIndex=table_visual,
            basePosition=[table_position[0], table_position[1], table_legs_height + table_size[2]/2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Apply metal texture to the table if available
        if hasattr(self.factory_env, 'texture_ids') and "metal" in self.factory_env.texture_ids:
            p.changeVisualShape(table_id, -1, textureUniqueId=self.factory_env.texture_ids["metal"])
        
        # Create table legs (four cylinders)
        leg_radius = 0.02
        leg_positions = [
            [table_position[0] - table_size[0]/2 + leg_radius, table_position[1] - table_size[1]/2 + leg_radius, 0],
            [table_position[0] - table_size[0]/2 + leg_radius, table_position[1] + table_size[1]/2 - leg_radius, 0],
            [table_position[0] + table_size[0]/2 - leg_radius, table_position[1] - table_size[1]/2 + leg_radius, 0],
            [table_position[0] + table_size[0]/2 - leg_radius, table_position[1] + table_size[1]/2 - leg_radius, 0]
        ]
        
        for leg_pos in leg_positions:
            leg_visual = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=leg_radius,
                length=table_legs_height,
                rgbaColor=[0.4, 0.4, 0.4, 1.0]  # Darker gray
            )
            
            leg_collision = p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=leg_radius,
                height=table_legs_height
            )
            
            leg_id = p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=leg_collision,
                baseVisualShapeIndex=leg_visual,
                basePosition=[leg_pos[0], leg_pos[1], table_legs_height/2],
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
            )
            
            # Add to factory environment objects list
            if hasattr(self.factory_env, 'object_ids'):
                self.factory_env.object_ids.append(leg_id)
        
        # Add some tools on the table
        tool_positions = [
            [table_position[0] - 0.1, table_position[1] - 0.1, table_legs_height + table_size[2] + 0.02],
            [table_position[0] + 0.1, table_position[1] + 0.1, table_legs_height + table_size[2] + 0.01]
        ]
        
        # Create a wrench
        wrench_size = [0.15, 0.04, 0.01]
        wrench_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[wrench_size[0]/2, wrench_size[1]/2, wrench_size[2]/2],
            rgbaColor=[0.1, 0.1, 0.1, 1.0]  # Dark gray
        )
        
        wrench_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[wrench_size[0]/2, wrench_size[1]/2, wrench_size[2]/2]
        )
        
        wrench_id = p.createMultiBody(
            baseMass=0.2,  # Light mass
            baseCollisionShapeIndex=wrench_collision,
            baseVisualShapeIndex=wrench_visual,
            basePosition=tool_positions[0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi/4])
        )
        
        # Add to factory environment objects list
        if hasattr(self.factory_env, 'object_ids'):
            self.factory_env.object_ids.append(wrench_id)
            self.factory_env.object_ids.append(table_id)
        
        print("Added industrial props to the environment")
        return True
    
    def check_collision(self):
        """Check if the robot arm is in collision with any object"""
        # Get all contact points
        contact_points = p.getContactPoints()
        
        # Filter contacts involving the robot arm
        arm_contacts = [cp for cp in contact_points if cp[1] == self.arm_id or cp[2] == self.arm_id]
        
        # Exclude contacts between consecutive links (they're always in contact)
        filtered_contacts = []
        for cp in arm_contacts:
            if cp[1] == self.arm_id and cp[2] == self.arm_id:
                link1, link2 = cp[3], cp[4]
                if abs(link1 - link2) > 1:  # Non-consecutive links
                    filtered_contacts.append(cp)
            else:
                filtered_contacts.append(cp)
        
        if self.debug_mode and filtered_contacts:
            print(f"Collision detected: {len(filtered_contacts)} contact points")
            for cp in filtered_contacts[:3]:  # Show only first 3 contacts
                print(f"  Contact between {cp[1]}:{cp[3]} and {cp[2]}:{cp[4]} at {cp[5]}")
        
        return len(filtered_contacts) > 0

    def setup_gripper(self):
        """Create and set up a visible suction cup for the UR5 arm"""
        # IMPORTANT: For UR5, we need to find the exact end of the last joint (wrist_3_link)
        # This is where the red circle is in the image
        
        # Get the state of the last joint (wrist_3_link)
        last_joint_index = 6  # This is the wrist_3_link for UR5
        last_joint_state = p.getLinkState(self.arm_id, last_joint_index)
        last_joint_pos = last_joint_state[0]  # Position of the joint
        last_joint_orn = last_joint_state[1]  # Orientation of the joint
        
        print(f"Last joint position: {last_joint_pos}")
        print(f"Last joint index: {last_joint_index}")
        
        # Create a visual shape for the suction cup - blue for visibility but thinner and much shorter
        suction_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.012,   # 1.2cm radius - thinner
            length=0.04,    # 4cm height - much shorter to prevent impaling
            rgbaColor=[0.2, 0.3, 0.8, 1.0]  # Blue color
        )
        
        # Create a collision shape for the suction cup
        suction_collision = p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.012,   # 1.2cm radius - thinner
            height=0.04     # 4cm height - much shorter to prevent impaling
        )
        
        # Calculate the position for the suction cup
        # We need to position it at the end of the last joint (in the red circle)
        # For UR5, this is at the end of the wrist_3_link
        
        # Get the joint info to find the joint axis
        joint_info = p.getJointInfo(self.arm_id, last_joint_index)
        joint_axis = joint_info[13]  # Joint axis in local frame
        
        # Convert joint axis to world frame using the joint orientation
        rot_matrix = p.getMatrixFromQuaternion(last_joint_orn)
        axis_world = [
            rot_matrix[0] * joint_axis[0] + rot_matrix[1] * joint_axis[1] + rot_matrix[2] * joint_axis[2],
            rot_matrix[3] * joint_axis[0] + rot_matrix[4] * joint_axis[1] + rot_matrix[5] * joint_axis[2],
            rot_matrix[6] * joint_axis[0] + rot_matrix[7] * joint_axis[1] + rot_matrix[8] * joint_axis[2]
        ]
        
        # Position the suction cup at the end of the joint along its axis
        # The offset is in the direction of the joint axis
        offset = 0.02  # 2cm offset from the joint center
        suction_pos = [
            last_joint_pos[0] + axis_world[0] * offset,
            last_joint_pos[1] + axis_world[1] * offset,
            last_joint_pos[2] + axis_world[2] * offset
        ]
        
        # Create the suction cup
        self.suction_cup = p.createMultiBody(
            baseMass=0.1,
            baseVisualShapeIndex=suction_visual,
            baseCollisionShapeIndex=suction_collision,
            basePosition=suction_pos,
            baseOrientation=last_joint_orn
        )
        
        # Create a constraint to attach the suction cup to the last joint
        self.suction_constraint = p.createConstraint(
            parentBodyUniqueId=self.arm_id,
            parentLinkIndex=last_joint_index,
            childBodyUniqueId=self.suction_cup,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[axis_world[0] * offset, axis_world[1] * offset, axis_world[2] * offset],
            childFramePosition=[0, 0, 0]
        )
        
        # Create a visual shape for the suction tip - black for contrast
        tip_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.015,  # 1.5cm radius - slightly wider than the cup
            length=0.01,   # 1cm height - extremely short tip to prevent impaling
            rgbaColor=[0.1, 0.1, 0.1, 1.0]  # Dark gray/black color
        )
        
        # Create a collision shape for the tip
        tip_collision = p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.015,  # 1.5cm radius
            height=0.01    # 1cm height - extremely short
        )
        
        # Position the tip at the end of the suction cup
        # The offset is along the local z-axis of the suction cup
        tip_offset = [0, 0, -0.025]  # 2.5cm below the center of the suction cup
        
        # Calculate the world position of the tip
        rot_matrix = p.getMatrixFromQuaternion(last_joint_orn)
        tip_pos = [
            suction_pos[0] + rot_matrix[2] * tip_offset[2],  # Use z-axis of rotation matrix
            suction_pos[1] + rot_matrix[5] * tip_offset[2],
            suction_pos[2] + rot_matrix[8] * tip_offset[2]
        ]
        
        # Create the suction tip
        self.suction_tip = p.createMultiBody(
            baseMass=0.05,
            baseVisualShapeIndex=tip_visual,
            baseCollisionShapeIndex=tip_collision,
            basePosition=tip_pos,
            baseOrientation=last_joint_orn
        )
        
        # Create a constraint to attach the tip to the suction cup
        self.tip_constraint = p.createConstraint(
            parentBodyUniqueId=self.suction_cup,
            parentLinkIndex=-1,
            childBodyUniqueId=self.suction_tip,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=tip_offset,
            childFramePosition=[0, 0, 0]
        )
        
        # Initialize gripper state (closed)
        self.gripper_state = False  # False = closed, True = open
        print("Suction cup and tip positioned at the end of the last joint (in the red circle)")
    
    def update_gripper_position(self):
        """Update the gripper position based on the arm's end effector and move any attached cube"""
        # If we have a cube attached, update its position with realistic tracking
        if hasattr(self, 'cube_attached') and self.cube_attached and hasattr(self, 'grabbed_cube_id'):
            try:
                # Get the suction tip position and orientation
                tip_pos, tip_orn = p.getBasePositionAndOrientation(self.suction_tip)
                
                # Get the current cube position and orientation
                current_cube_pos, current_cube_orn = p.getBasePositionAndOrientation(self.grabbed_cube_id)
                
                # IMPROVED TRACKING: Combine fixed offset with relative offset tracking
                # This approach is inspired by the destacking procedure
                
                # Get the rotation matrix from the tip's orientation
                rot_matrix = p.getMatrixFromQuaternion(tip_orn)
                
                # Extract the local z-axis (down direction) from the rotation matrix
                z_axis = [rot_matrix[2], rot_matrix[5], rot_matrix[8]]
                
                # Calculate the offset distance - fixed distance below the tip
                offset_distance = 0.035  # 3.5cm below the tip - increased to prevent impaling
                
                # If we don't have stored offsets yet, calculate and store them
                # This helps maintain the cube's original relative position to the tip
                if not hasattr(self, 'cube_offset_x') or not hasattr(self, 'cube_offset_y'):
                    # Calculate the initial offset between cube and suction tip
                    suction_pos, _ = p.getBasePositionAndOrientation(self.suction_cup)
                    self.cube_offset_x = current_cube_pos[0] - suction_pos[0]
                    self.cube_offset_y = current_cube_pos[1] - suction_pos[1]
                    
                    # Also store the initial height offset for better z-tracking
                    self.cube_offset_z = current_cube_pos[2] - tip_pos[2]
                
                # Calculate the target position using both fixed offset and relative offsets
                # This creates more stable tracking that respects the original grab position
                target_pos = [
                    tip_pos[0] + self.cube_offset_x * 0.7,  # Maintain some of the original X offset
                    tip_pos[1] + self.cube_offset_y * 0.7,  # Maintain some of the original Y offset
                    tip_pos[2] + z_axis[2] * offset_distance  # Fixed Z offset for stable height
                ]
                
                # Use a variable tracking factor based on speed
                # Faster movements need tighter tracking to prevent lagging
                if hasattr(self, 'last_tip_pos'):
                    # Calculate tip movement speed
                    tip_speed = np.linalg.norm(np.array(tip_pos) - np.array(self.last_tip_pos))
                    # Adjust tracking factor based on speed (faster movement = higher tracking)
                    tracking_factor = min(0.95, 0.8 + tip_speed * 50)
                else:
                    tracking_factor = 0.9  # Default tracking factor
                    self.last_tip_pos = tip_pos
                
                # Update last tip position for next frame
                self.last_tip_pos = tip_pos
                
                # Calculate the new position with tracking factor
                new_pos = [
                    current_cube_pos[0] + (target_pos[0] - current_cube_pos[0]) * tracking_factor,
                    current_cube_pos[1] + (target_pos[1] - current_cube_pos[1]) * tracking_factor,
                    current_cube_pos[2] + (target_pos[2] - current_cube_pos[2]) * tracking_factor
                ]
                
                # Calculate the distance between the cube and target position
                distance = np.sqrt(
                    (new_pos[0] - target_pos[0])**2 + 
                    (new_pos[1] - target_pos[1])**2 + 
                    (new_pos[2] - target_pos[2])**2
                )
                
                # If the cube is too far from the target, snap it back
                # This prevents the cube from drifting too far
                max_allowed_distance = 0.03  # 3cm maximum drift (slightly increased for smoother motion)
                if distance > max_allowed_distance:
                    # Snap back to the target position with a blend for smoother correction
                    new_pos = [
                        current_cube_pos[0] * 0.2 + target_pos[0] * 0.8,
                        current_cube_pos[1] * 0.2 + target_pos[1] * 0.8,
                        current_cube_pos[2] * 0.2 + target_pos[2] * 0.8
                    ]
                
                # Blend the orientation for realistic rotation
                # This creates a slight lag in rotation that looks natural
                rotation_factor = 0.85  # Slightly higher for better tracking
                new_orn = p.getQuaternionSlerp(current_cube_orn, tip_orn, rotation_factor)
                
                # Update the cube position and orientation
                p.resetBasePositionAndOrientation(self.grabbed_cube_id, new_pos, new_orn)
                
                # Set appropriate physics properties for the grabbed cube
                p.changeDynamics(
                    self.grabbed_cube_id, 
                    -1, 
                    mass=0.05,  # Light but not too light
                    linearDamping=0.9,  # High damping for stability
                    angularDamping=0.9,  # High damping for stability
                    activationState=p.ACTIVATION_STATE_WAKE_UP  # Keep it active for physics
                )
                
                # Disable collisions for the grabbed cube
                p.setCollisionFilterGroupMask(self.grabbed_cube_id, -1, 0, 0)
                
                # Specifically disable collisions with key objects
                p.setCollisionFilterPair(self.plane_id, self.grabbed_cube_id, -1, -1, 0)  # Ground
                p.setCollisionFilterPair(self.arm_id, self.grabbed_cube_id, -1, -1, 0)    # Arm
                p.setCollisionFilterPair(self.suction_tip, self.grabbed_cube_id, -1, -1, 0)  # Tip
                
                # Apply a small upward force to simulate suction
                # This creates a subtle hovering effect that looks more realistic
                suction_force = [0, 0, 0.5]  # Moderate suction force
                p.applyExternalForce(
                    objectUniqueId=self.grabbed_cube_id,
                    linkIndex=-1,
                    forceObj=suction_force,
                    posObj=new_pos,
                    flags=p.WORLD_FRAME
                )
                
                # Log which cube is being moved (if we have the color information)
                if hasattr(self, 'grabbed_cube_color'):
                    cube_color = self.grabbed_cube_color
                    # Only log occasionally to avoid console spam
                    if np.random.random() < 0.005:  # Log rarely
                        print(f"Moving {cube_color} cube with smooth tracking")
                
            except Exception as e:
                print(f"Error updating cube position: {e}")
                # If there's an error, we'll try to recover next frame
    
    def toggle_gripper(self):
        """Toggle the suction cup between active and inactive states"""
        self.gripper_state = not self.gripper_state
        
        # Change the appearance of both the suction cup and tip to indicate state
        if self.gripper_state:  # Active (on)
            # Change suction cup color to bright blue
            p.changeVisualShape(self.suction_cup, -1, rgbaColor=[0.0, 0.4, 0.8, 1.0])
            # Change tip color to darker blue to show suction
            p.changeVisualShape(self.suction_tip, -1, rgbaColor=[0.0, 0.2, 0.5, 1.0])
            print("Suction activated")
            
            # Check if the cube is near the suction tip and grab it
            if not hasattr(self, 'cube_attached') or not self.cube_attached:
                self.try_grab_cube()
        else:  # Inactive (off)
            # Change suction cup color back to blue
            p.changeVisualShape(self.suction_cup, -1, rgbaColor=[0.2, 0.3, 0.7, 1.0])
            # Change tip color back to black
            p.changeVisualShape(self.suction_tip, -1, rgbaColor=[0.1, 0.1, 0.1, 1.0])
            print("Suction deactivated")
            
            # Release any attached cube with smooth physics
            if hasattr(self, 'cube_attached') and self.cube_attached and hasattr(self, 'grabbed_cube_id'):
                # Get current cube position and velocity for smooth release
                cube_pos, cube_orn = p.getBasePositionAndOrientation(self.grabbed_cube_id)
                cube_lin_vel, cube_ang_vel = p.getBaseVelocity(self.grabbed_cube_id)
                
                # Store which cube we're releasing (for logging)
                released_cube_id = self.grabbed_cube_id
                released_cube_color = self.grabbed_cube_color if hasattr(self, 'grabbed_cube_color') else "unknown"
                
                # SMOOTH RELEASE: Apply a small downward impulse for a natural drop effect
                # This simulates the cube being released from suction
                release_impulse = [cube_lin_vel[0] * 0.5, cube_lin_vel[1] * 0.5, -0.1]
                p.applyExternalForce(
                    self.grabbed_cube_id,
                    -1,
                    release_impulse,
                    cube_pos,
                    p.WORLD_FRAME
                )
                
                # Restore the original mass and properties of the cube with smooth transition
                if hasattr(self, 'original_cube_mass'):
                    # First set intermediate properties for a smoother transition
                    # Make sure we're using scalar values, not tuples
                    mass_value = float(self.original_cube_mass) * 0.5 if isinstance(self.original_cube_mass, (int, float)) else 0.05
                    friction_value = float(self.original_cube_friction) * 0.5 if isinstance(self.original_cube_friction, (int, float)) else 0.5
                    
                    p.changeDynamics(
                        self.grabbed_cube_id, 
                        -1, 
                        mass=mass_value,  # Half-way to original mass
                        lateralFriction=friction_value  # Half-way to original friction
                    )
                    
                    # Step the simulation a few times to let physics settle
                    for _ in range(2):
                        p.stepSimulation()
                        time.sleep(0.01)
                    
                    # Then restore full original properties, ensuring all values are scalar
                    mass_value = float(self.original_cube_mass) if isinstance(self.original_cube_mass, (int, float)) else 0.1
                    friction_value = float(self.original_cube_friction) if isinstance(self.original_cube_friction, (int, float)) else 0.9
                    restitution_value = float(self.original_cube_restitution) if hasattr(self, 'original_cube_restitution') and isinstance(self.original_cube_restitution, (int, float)) else 0.1
                    
                    p.changeDynamics(
                        self.grabbed_cube_id, 
                        -1, 
                        mass=mass_value,
                        lateralFriction=friction_value,
                        restitution=restitution_value,
                        linearDamping=0.1,  # Reduced damping for natural movement
                        angularDamping=0.1,  # Reduced damping for natural rotation
                        activationState=p.ACTIVATION_STATE_WAKE_UP  # Ensure it's active for physics
                    )
                    print(f"Smoothly restored original cube properties")
                
                # Re-enable all collisions for the cube
                # This is critical for proper physics after release
                p.setCollisionFilterGroupMask(self.grabbed_cube_id, -1, 1, 1)  # Enable all collisions
                
                # IMPORTANT: Only disable collision between the grabbed cube and other objects
                # We need to maintain collisions between all other cubes to prevent them from disappearing
                if hasattr(self, 'cubes'):
                    # First, re-enable collisions between the released cube and all other cubes
                    for color, cube_info in self.cubes.items():
                        other_cube_id = cube_info["id"]
                        if other_cube_id != self.grabbed_cube_id:
                            # Enable collision between the released cube and this cube
                            p.setCollisionFilterPair(self.grabbed_cube_id, other_cube_id, -1, -1, 1)
                    
                    # Then, make sure all other cubes maintain their collisions with each other
                    # This is critical to prevent cubes from disappearing after releasing
                    for color1, cube_info1 in self.cubes.items():
                        for color2, cube_info2 in self.cubes.items():
                            if color1 != color2 and cube_info1["id"] != self.grabbed_cube_id and cube_info2["id"] != self.grabbed_cube_id:
                                # Enable collision between these two non-grabbed cubes
                                p.setCollisionFilterPair(cube_info1["id"], cube_info2["id"], -1, -1, 1)
                
                # Re-enable collisions with the ground plane
                p.setCollisionFilterPair(self.plane_id, self.grabbed_cube_id, -1, -1, 1)
                
                # Re-enable collisions with the arm
                p.setCollisionFilterPair(self.arm_id, self.grabbed_cube_id, -1, -1, 1)
                
                # Apply a small downward force to help the cube detach naturally
                cube_pos, _ = p.getBasePositionAndOrientation(self.grabbed_cube_id)
                detach_force = [0, 0, -0.3]  # Small downward force
                p.applyExternalForce(
                    self.grabbed_cube_id,
                    -1,
                    detach_force,
                    cube_pos,
                    p.WORLD_FRAME
                )
                print("Applied detach force to cube")
                
                # Reset the cube_attached flag and clean up attributes
                self.cube_attached = False
                if hasattr(self, 'grab_offset'):
                    delattr(self, 'grab_offset')
                if hasattr(self, 'last_cube_pos'):
                    delattr(self, 'last_cube_pos')
                
                # Create a subtle visual effect for release
                for _ in range(3):
                    p.stepSimulation()
                    time.sleep(0.01)
                
                print(f"{released_cube_color.capitalize()} cube smoothly released")
    
    def try_grab_cube(self):
        """Try to grab the closest cube if it's near the suction tip"""
        # Get the suction tip position and orientation
        tip_pos, tip_orn = p.getBasePositionAndOrientation(self.suction_tip)
        
        # Find the closest cube to the suction tip
        closest_cube_id = None
        closest_distance = float('inf')
        closest_color = None
        
        if hasattr(self, 'cubes'):
            for color, cube_info in self.cubes.items():
                cube_id = cube_info["id"]
                cube_pos, cube_orn = p.getBasePositionAndOrientation(cube_id)
                
                # Calculate distance from this cube to the tip
                distance = np.linalg.norm(np.array(cube_pos) - np.array(tip_pos))
                
                # If this cube is closer than any we've seen so far, update our tracking
                if distance < closest_distance:
                    closest_distance = distance
                    closest_cube_id = cube_id
                    closest_color = color
        
        # If we didn't find any cubes, return
        if closest_cube_id is None:
            print("No cubes found to grab.")
            return
        
        # Print the distance to the closest cube
        print(f"Try grab cube - Distance from tip to {closest_color} cube: {closest_distance:.4f}")
        
        # Check if the closest cube is close enough to grab
        # Use a much more generous threshold for better usability
        if closest_distance > 0.25:  # 25cm threshold - increased to allow grabbing from further away
            print(f"Closest cube ({closest_color}) too far to grab. Move closer to the cube.")
            return
        
        print(f"{closest_color.capitalize()} cube is close enough to grab!")
        
        # Store the original mass and properties of the cube for later restoration
        # With proper error handling for potential tuple values
        try:
            dynamics_info = p.getDynamicsInfo(closest_cube_id, -1)
            
            # Safely extract mass (index 0)
            try:
                mass_value = dynamics_info[0]
                self.original_cube_mass = float(mass_value) if not isinstance(mass_value, tuple) else 0.1
            except (TypeError, ValueError):
                self.original_cube_mass = 0.1  # Default mass if conversion fails
                
            # Safely extract friction (index 1)
            try:
                friction_value = dynamics_info[1]
                self.original_cube_friction = float(friction_value) if not isinstance(friction_value, tuple) else 0.9
            except (TypeError, ValueError):
                self.original_cube_friction = 0.9  # Default friction if conversion fails
                
            # Safely extract restitution (index 2)
            try:
                restitution_value = dynamics_info[2]
                self.original_cube_restitution = float(restitution_value) if not isinstance(restitution_value, tuple) else 0.1
            except (TypeError, ValueError):
                self.original_cube_restitution = 0.1  # Default restitution if conversion fails
                
            print(f"Original cube properties stored: mass={self.original_cube_mass}, friction={self.original_cube_friction}, restitution={self.original_cube_restitution}")
        except Exception as e:
            # If anything goes wrong, use default values
            self.original_cube_mass = 0.1
            self.original_cube_friction = 0.9
            self.original_cube_restitution = 0.1
            print(f"Using default cube properties due to error: {e}")
        
        # Store the cube ID for tracking
        self.grabbed_cube_id = closest_cube_id
        self.cube_attached = True
        # Also store which cube we're grabbing for reference
        self.grabbed_cube_color = closest_color
        
        # REALISTIC TRACKING: Set up the cube for realistic tracking
        # We want the cube to follow the tip with a natural motion
        
        # Get the rotation matrix from the tip's orientation
        rot_matrix = p.getMatrixFromQuaternion(tip_orn)
        
        # Extract the local z-axis (down direction) from the rotation matrix
        z_axis = [rot_matrix[2], rot_matrix[5], rot_matrix[8]]
        
        # Calculate the offset distance - fixed distance below the tip
        offset_distance = 0.035  # 3.5cm below the tip - increased to prevent impaling
        
        # Calculate the initial target position below the tip
        initial_pos = [
            tip_pos[0] + z_axis[0] * offset_distance,
            tip_pos[1] + z_axis[1] * offset_distance,
            tip_pos[2] + z_axis[2] * offset_distance
        ]
        
        # Get the current cube position and orientation
        current_cube_pos, current_cube_orn = p.getBasePositionAndOrientation(closest_cube_id)
        
        # Calculate a position halfway between the current position and the target
        # This creates a smooth initial grab motion
        halfway_pos = [
            current_cube_pos[0] + (initial_pos[0] - current_cube_pos[0]) * 0.5,
            current_cube_pos[1] + (initial_pos[1] - current_cube_pos[1]) * 0.5,
            current_cube_pos[2] + (initial_pos[2] - current_cube_pos[2]) * 0.5
        ]
        
        # Move the cube to the halfway position first
        p.resetBasePositionAndOrientation(closest_cube_id, halfway_pos, current_cube_orn)
        
        # Set appropriate physics properties for the grabbed cube
        p.changeDynamics(
            closest_cube_id, 
            -1, 
            mass=0.05,  # Light but not too light
            linearDamping=0.9,  # High damping for stability
            angularDamping=0.9,  # High damping for stability
            activationState=p.ACTIVATION_STATE_WAKE_UP  # Keep it active for physics
        )
        
        # Disable collisions for the grabbed cube
        p.setCollisionFilterGroupMask(closest_cube_id, -1, 0, 0)
        
        # Specifically disable collisions with key objects
        p.setCollisionFilterPair(self.plane_id, closest_cube_id, -1, -1, 0)  # Ground
        p.setCollisionFilterPair(self.arm_id, closest_cube_id, -1, -1, 0)    # Arm
        p.setCollisionFilterPair(self.suction_tip, closest_cube_id, -1, -1, 0)  # Tip
        
        # Apply a small upward force to simulate initial suction
        # This creates a subtle lifting effect that looks more realistic
        suction_force = [0, 0, 0.5]  # Moderate initial lift
        p.applyExternalForce(
            closest_cube_id,
            -1,
            suction_force,
            halfway_pos,
            p.WORLD_FRAME
        )
        
        # Step the simulation a few times to let the initial grab motion settle
        for _ in range(3):
            p.stepSimulation()
            time.sleep(0.01)
        
        # SMOOTH INITIAL POSITIONING: Move the cube to the initial position with a small offset
        # This creates a more natural initial grab motion
        
        # Get the current cube position
        current_cube_pos, current_cube_orn = p.getBasePositionAndOrientation(closest_cube_id)
        
        # Create a slightly offset initial position (halfway between current and target)
        # This prevents a sudden jump at the beginning
        halfway_pos = [
            current_cube_pos[0] + (initial_pos[0] - current_cube_pos[0]) * 0.5,
            current_cube_pos[1] + (initial_pos[1] - current_cube_pos[1]) * 0.5,
            current_cube_pos[2] + (initial_pos[2] - current_cube_pos[2]) * 0.5
        ]
        
        # Smoothly move to the initial position
        p.resetBasePositionAndOrientation(closest_cube_id, halfway_pos, current_cube_orn)
        
        # Change the color of the suction tip to indicate active suction
        p.changeVisualShape(self.suction_tip, -1, rgbaColor=[0.0, 0.2, 0.5, 1.0])  # Darker blue
        
        # Create a subtle visual effect for the suction activation
        for _ in range(3):
            p.stepSimulation()
            time.sleep(0.01)
        
        print(f"{closest_color.capitalize()} cube successfully grabbed with smooth tracking enabled!")

    def reset_arm_position(self):
        """Reset the arm to a stable starting position"""
        # Define a stable starting configuration for the UR5 arm
        # These values are in radians for each joint
        stable_positions = [
            0,              # Base rotation
            -0.8,           # Shoulder (lowered to better reach cubes)
            1.0,            # Elbow (adjusted to position arm lower)
            -1.5,           # Wrist 1 (adjusted to position suction cup better)
            -1.57,          # Wrist 2
            0               # Wrist 3
        ]
        
        # Apply the joint positions
        for i in range(min(len(stable_positions), self.num_joints)):
            p.resetJointState(self.arm_id, i, stable_positions[i])
        
        # Update the position if it exists
        if hasattr(self, 'position'):
            self.position = np.array([0.5, 0.0], dtype=np.float32)
        
        # Update rotation if rotation_options exists
        if hasattr(self, 'rotation_options') and hasattr(self, 'rotation_index'):
            self.rotation_index = 1  # Use a higher position to avoid ground clipping
            self.rotation = self.rotation_options[self.rotation_index]
        
        # Let the simulation settle
        for _ in range(20):
            p.stepSimulation()
            time.sleep(0.01)
        
        print("Arm reset to stable position")
        
    def run_gearbox_assembly_task(self):
        """Run the gearbox assembly task using the imported function"""
        print("\nStarting gearbox assembly task...")
        try:
            # Call the imported function for gearbox assembly
            run_gearbox_assembly(self)
            print("Gearbox assembly completed!")
        except Exception as e:
            print(f"Error during gearbox assembly: {e}")
            import traceback
            traceback.print_exc()
    
    def run_gearbox_disassembly_task(self):
        """Run the gearbox disassembly task using the imported function"""
        print("\nStarting gearbox disassembly task...")
        try:
            # Call the imported function for gearbox disassembly
            run_disassembly_task(self)
            print("Gearbox disassembly completed!")
        except Exception as e:
            print(f"Error during gearbox disassembly: {e}")
            import traceback
            traceback.print_exc()
            
    def update_status_display(self):
        """Update the control panel with current status"""
        # Create a copy of the control panel
        display = self.control_panel.copy()
        
        # Add status information
        status_text = [
            f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}]",
            f"Height: {self.rotation_index}/6",
            f"Step Size: {self.step_size:.3f}",
            f"Gripper: {'Open' if self.gripper_state else 'Closed'}",
            f"Camera: Dist={self.camera_distance:.1f}, Yaw={self.camera_yaw:.1f}, Pitch={self.camera_pitch:.1f}"
        ]
        
        # Add cube position information if cube exists
        if hasattr(self, 'cubes') and self.cubes:
            for color, cube_info in self.cubes.items():
                cube_id = cube_info["id"]
                cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
                status_text.append(f"{color.capitalize()} cube: [{cube_pos[0]:.2f}, {cube_pos[1]:.2f}, {cube_pos[2]:.2f}]")
        
        # Add platform position if analyzed
        if hasattr(self, 'scene_objects') and 'blue_platform' in self.scene_objects and self.scene_objects['blue_platform']:
            platform_pos = self.scene_objects['blue_platform'][0]['position']
            status_text.append(f"Platform: [{platform_pos[0]:.2f}, {platform_pos[1]:.2f}, {platform_pos[2]:.2f}]")
        
        for i, text in enumerate(status_text):
            cv2.putText(display, text, (10, 10 + i*20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Add section titles
        cv2.putText(display, "Arm Controls:", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.putText(display, "Cube Controls:", (10, 350), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.putText(display, "Camera Controls:", (10, 500), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.putText(display, "Task Controls:", (10, 600), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.imshow(self.window_name, display)

def main():
    parser = argparse.ArgumentParser(description='UR5 Robotic Arm Simulation with PyBullet')
    parser.add_argument('--headless', action='store_true', help='Run in headless mode (no GUI)')
    args = parser.parse_args()
    
    render_mode = "DIRECT" if args.headless else "GUI"
    sim = UR5ArmSimulation(render_mode=render_mode)
    sim.run_simulation()

if __name__ == "__main__":
    main()
