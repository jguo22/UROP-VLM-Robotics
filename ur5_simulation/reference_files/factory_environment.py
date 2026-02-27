import pybullet as p
import os
import numpy as np
import random

class FactoryEnvironment:
    """Class to add factory environment textures and objects to the simulation"""
    
    def __init__(self, textures_dir):
        """Initialize the factory environment with textures directory"""
        self.textures_dir = textures_dir
        self.texture_ids = {}
        self.object_ids = []
        self.warning_stripe_ids = []
    
    def load_textures(self):
        """Load all textures from the textures directory"""
        # Check if textures directory exists
        if not os.path.exists(self.textures_dir):
            print(f"Textures directory {self.textures_dir} does not exist")
            return False
        
        # Define all textures to load
        texture_files = {
            "floor": "industrial_floor.jpg",  # Updated to more industrial floor
            "wall": "factory_wall.jpg",
            "platform": "metal_platform.jpg",
            "warning_stripes": "warning_stripes.jpg",
            "metal": "metal_texture.jpg",
            "robot": "robot_metal.jpg"
        }
        
        # Load each texture
        for texture_name, file_name in texture_files.items():
            texture_path = os.path.join(self.textures_dir, file_name)
            if os.path.exists(texture_path):
                self.texture_ids[texture_name] = p.loadTexture(texture_path)
                print(f"Loaded {texture_name} texture: {file_name}")
            else:
                print(f"{texture_name.capitalize()} texture not found: {texture_path}")
        
        # Fallback to original floor texture if industrial floor not found
        if "floor" not in self.texture_ids:
            floor_texture_path = os.path.join(self.textures_dir, "factory_floor.jpg")
            if os.path.exists(floor_texture_path):
                self.texture_ids["floor"] = p.loadTexture(floor_texture_path)
                print(f"Loaded fallback floor texture: {floor_texture_path}")
        
        return len(self.texture_ids) > 0
    
    def apply_texture_to_plane(self, plane_id):
        """Apply factory floor texture to the plane"""
        if "floor" in self.texture_ids:
            p.changeVisualShape(plane_id, -1, textureUniqueId=self.texture_ids["floor"])
            print("Applied factory floor texture to plane")
            
            # Add warning stripes around the work area
            self.create_warning_stripes()
            return True
        return False
        
    def create_warning_stripes(self):
        """Create warning stripes on the floor to mark work areas"""
        if "warning_stripes" not in self.texture_ids:
            print("Warning stripes texture not loaded, skipping creation")
            return False
            
        # Define the work area with warning stripes
        stripe_thickness = 0.02  # 2cm thick stripes
        stripe_height = 0.001  # Very thin to avoid collision issues
        work_area_size = 1.2  # Size of the work area
        
        # Create stripes around the robot work area
        stripe_positions = [
            # Front stripe
            ([0.6, work_area_size/2, stripe_height/2], [0, 0, 0], [work_area_size, stripe_thickness, stripe_height]),
            # Back stripe
            ([0.6, -work_area_size/2, stripe_height/2], [0, 0, 0], [work_area_size, stripe_thickness, stripe_height]),
            # Left stripe
            ([0.6 - work_area_size/2, 0, stripe_height/2], [0, 0, np.pi/2], [work_area_size, stripe_thickness, stripe_height]),
            # Right stripe
            ([0.6 + work_area_size/2, 0, stripe_height/2], [0, 0, np.pi/2], [work_area_size, stripe_thickness, stripe_height])
        ]
        
        for position, orientation, size in stripe_positions:
            # Create visual shape
            stripe_visual = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size[0]/2, size[1]/2, size[2]/2],
                rgbaColor=[1.0, 1.0, 0.0, 1.0]  # Yellow base color
            )
            
            # Create collision shape (very thin to avoid physics issues)
            stripe_collision = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size[0]/2, size[1]/2, size[2]/2]
            )
            
            # Create the stripe
            stripe_id = p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=stripe_collision,
                baseVisualShapeIndex=stripe_visual,
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler(orientation)
            )
            
            # Apply warning stripes texture
            p.changeVisualShape(stripe_id, -1, textureUniqueId=self.texture_ids["warning_stripes"])
            
            # Add to list of created objects
            self.warning_stripe_ids.append(stripe_id)
            self.object_ids.append(stripe_id)
        
        print("Created warning stripes around work area")
        return True
    
    def apply_texture_to_platform(self, platform_id):
        """Apply metal texture to the platform"""
        if "platform" in self.texture_ids:
            p.changeVisualShape(platform_id, -1, textureUniqueId=self.texture_ids["platform"])
            print("Applied metal texture to platform")
            return True
        return False
    
    def create_factory_walls(self):
        """Create walls around the environment to make it look like a factory"""
        if "wall" not in self.texture_ids:
            print("Wall texture not loaded, skipping wall creation")
            return False
        
        # Wall dimensions
        wall_height = 1.5
        wall_thickness = 0.1
        room_size = 2.0
        
        # Wall positions (back, left, right)
        wall_positions = [
            # Back wall
            ([0, -room_size/2, wall_height/2], [0, 0, 0], [room_size, wall_thickness, wall_height]),
            # Left wall
            ([-room_size/2, 0, wall_height/2], [0, 0, np.pi/2], [room_size, wall_thickness, wall_height]),
            # Right wall
            ([room_size/2, 0, wall_height/2], [0, 0, np.pi/2], [room_size, wall_thickness, wall_height])
        ]
        
        # Create each wall
        for position, orientation, size in wall_positions:
            # Create visual shape
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size[0]/2, size[1]/2, size[2]/2],
                rgbaColor=[0.5, 0.5, 0.5, 1.0]  # Gray color as base
            )
            
            # Create collision shape
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size[0]/2, size[1]/2, size[2]/2]
            )
            
            # Create the wall
            wall_id = p.createMultiBody(
                baseMass=0,  # Static wall
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler(orientation)
            )
            
            # Apply wall texture
            p.changeVisualShape(wall_id, -1, textureUniqueId=self.texture_ids["wall"])
            
            # Add to list of created objects
            self.object_ids.append(wall_id)
        
        # Add industrial details to the walls - pipes, vents, etc.
        self.add_industrial_details()
        
        print("Created factory walls with textures")
        return True
        
    def add_industrial_details(self):
        """Add industrial details like pipes, vents, and control panels to the walls"""
        # Add some pipes along the back wall
        pipe_radius = 0.02
        pipe_length = 1.5
        pipe_positions = [
            # Horizontal pipes along back wall
            ([0, -0.95, 0.3], [0, 0, 0], pipe_radius, pipe_length),
            ([0, -0.95, 0.5], [0, 0, 0], pipe_radius, pipe_length),
            # Vertical pipes
            ([-0.5, -0.95, 0.4], [np.pi/2, 0, 0], pipe_radius, 0.8),
            ([0.5, -0.95, 0.4], [np.pi/2, 0, 0], pipe_radius, 0.8)
        ]
        
        for position, orientation, radius, length in pipe_positions:
            pipe_visual = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=radius,
                length=length,
                rgbaColor=[0.7, 0.7, 0.7, 1.0]  # Silver color
            )
            
            pipe_collision = p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=radius,
                height=length
            )
            
            pipe_id = p.createMultiBody(
                baseMass=0,  # Static
                baseCollisionShapeIndex=pipe_collision,
                baseVisualShapeIndex=pipe_visual,
                basePosition=position,
                baseOrientation=p.getQuaternionFromEuler(orientation)
            )
            
            # Apply metal texture
            if "metal" in self.texture_ids:
                p.changeVisualShape(pipe_id, -1, textureUniqueId=self.texture_ids["metal"])
            
            self.object_ids.append(pipe_id)
        
        # Add a control panel on the back wall
        panel_size = [0.3, 0.05, 0.2]
        panel_pos = [0.6, -0.95, 0.5]
        
        panel_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[panel_size[0]/2, panel_size[1]/2, panel_size[2]/2],
            rgbaColor=[0.2, 0.2, 0.2, 1.0]  # Dark gray
        )
        
        panel_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[panel_size[0]/2, panel_size[1]/2, panel_size[2]/2]
        )
        
        panel_id = p.createMultiBody(
            baseMass=0,  # Static
            baseCollisionShapeIndex=panel_collision,
            baseVisualShapeIndex=panel_visual,
            basePosition=panel_pos,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        self.object_ids.append(panel_id)
        
        print("Added industrial details to factory walls")
        return True
    
    def apply_robot_texture(self, arm_id):
        """Apply robot metal texture to the robot arm"""
        if "robot" not in self.texture_ids:
            print("Robot texture not loaded, skipping application")
            return False
            
        # Get the number of links in the robot arm
        num_links = p.getNumJoints(arm_id)
        
        # Apply texture to each link of the robot
        for i in range(-1, num_links):  # -1 is the base link
            p.changeVisualShape(arm_id, i, textureUniqueId=self.texture_ids["robot"])
        
        print("Applied robot metal texture to robot arm")
        return True
    
    def add_factory_lights(self):
        """Add factory-style lighting to the environment"""
        # Enable shadows for more realistic look
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        
        # Set main overhead light
        p.configureDebugVisualizer(lightPosition=[0, 0, 3])
        
        # Create additional light sources using visual-only objects
        light_positions = [
            [1.0, 1.0, 2.0],  # Front right
            [-1.0, 1.0, 2.0],  # Front left
            [0.0, -1.0, 2.0]   # Back center
        ]
        
        # Create small light fixtures at each position
        for pos in light_positions:
            # Create a small visual-only light fixture
            light_visual = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.05,
                rgbaColor=[1.0, 1.0, 0.8, 1.0]  # Warm light color
            )
            
            light_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=light_visual,
                basePosition=pos
            )
            
            self.object_ids.append(light_id)
        
        print("Added enhanced factory lighting")
        return True
    
    def cleanup(self):
        """Remove all created objects"""
        for obj_id in self.object_ids:
            p.removeBody(obj_id)
        self.object_ids = []
        print("Cleaned up factory environment objects")
