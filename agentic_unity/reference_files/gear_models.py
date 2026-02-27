#!/usr/bin/env python3
"""
Gear Models Generator for PyBullet
This script provides functions to create realistic gear models for use in PyBullet simulations.
"""

import os
import pybullet as p
import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom


def create_gear_urdf(gear_name, gear_diameter, height, teeth_count, tooth_height, 
                     mass, color, output_dir):
    """
    Creates a URDF file for a gear with the specified parameters.
    
    Args:
        gear_name (str): Name of the gear (used for file naming)
        gear_diameter (float): Base diameter of the gear in meters
        height (float): Height/thickness of the gear in meters
        teeth_count (int): Number of teeth on the gear
        tooth_height (float): Height of each tooth in meters
        mass (float): Mass of the gear in kg
        color (list): RGBA color as [r, g, b, a] with values from 0 to 1
        output_dir (str): Directory to save the URDF file
    
    Returns:
        str: Path to the created URDF file
    """
    # Calculate gear properties
    base_radius = gear_diameter / 2
    outer_radius = base_radius + tooth_height
    inertia_val = (1/12) * mass * (3 * base_radius**2 + height**2)  # Approximation
    
    # Create XML structure
    robot = ET.Element('robot', name=gear_name)
    
    # Link element
    link = ET.SubElement(robot, 'link', name='base_link')
    
    # Inertial properties
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'origin', xyz="0 0 0", rpy="0 0 0")
    inertia = ET.SubElement(inertial, 'inertia', 
                           ixx=str(inertia_val), ixy="0", ixz="0",
                           iyy=str(inertia_val), iyz="0", izz=str(inertia_val))
    
    # Base cylinder (visual)
    visual = ET.SubElement(link, 'visual')
    ET.SubElement(visual, 'origin', xyz="0 0 0", rpy="0 0 0")
    base_visual = ET.SubElement(visual, 'geometry')
    ET.SubElement(base_visual, 'cylinder', radius=str(base_radius), length=str(height))
    material = ET.SubElement(visual, 'material', name=f"{gear_name}_material")
    ET.SubElement(material, 'color', rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}")
    
    # Add teeth visuals
    for i in range(teeth_count):
        angle = (2 * np.pi * i) / teeth_count
        tooth_width = 2 * np.pi * base_radius / (teeth_count * 2.5)  # Tooth width proportional to gear size
        
        # Calculate tooth position
        tooth_pos_x = (base_radius + tooth_height/2) * np.cos(angle)
        tooth_pos_y = (base_radius + tooth_height/2) * np.sin(angle)
        
        # Add tooth visual
        tooth_visual = ET.SubElement(link, 'visual')
        ET.SubElement(tooth_visual, 'origin', 
                     xyz=f"{tooth_pos_x} {tooth_pos_y} 0", 
                     rpy=f"0 0 {angle}")
        tooth_geom = ET.SubElement(tooth_visual, 'geometry')
        ET.SubElement(tooth_geom, 'box', 
                     size=f"{tooth_height} {tooth_width} {height}")
        tooth_material = ET.SubElement(tooth_visual, 'material', name=f"{gear_name}_tooth_material")
        
        # Make teeth slightly lighter than base for visual distinction
        tooth_color = [min(c * 1.2, 1.0) for c in color[:3]] + [color[3]]
        ET.SubElement(tooth_material, 'color', 
                     rgba=f"{tooth_color[0]} {tooth_color[1]} {tooth_color[2]} {tooth_color[3]}")
    
    # Add outer rim for better tooth visibility
    rim_visual = ET.SubElement(link, 'visual')
    ET.SubElement(rim_visual, 'origin', xyz="0 0 0", rpy="0 0 0")
    rim_geom = ET.SubElement(rim_visual, 'geometry')
    rim_height = height * 0.4  # Smaller height to let teeth be more visible
    ET.SubElement(rim_geom, 'cylinder', radius=str(outer_radius * 0.92), length=str(rim_height))
    rim_material = ET.SubElement(rim_visual, 'material', name=f"{gear_name}_rim_material")
    
    # Darker shade for rim
    rim_color = [c * 0.8 for c in color[:3]] + [color[3]]
    ET.SubElement(rim_material, 'color', 
                 rgba=f"{rim_color[0]} {rim_color[1]} {rim_color[2]} {rim_color[3]}")
    
    # Center hole (visual)
    center_hole_radius = base_radius / 5
    visual_hole = ET.SubElement(link, 'visual')
    ET.SubElement(visual_hole, 'origin', xyz="0 0 0", rpy="0 0 0")
    hole_visual = ET.SubElement(visual_hole, 'geometry')
    ET.SubElement(hole_visual, 'cylinder', radius=str(center_hole_radius), length=str(height * 1.01))
    material_hole = ET.SubElement(visual_hole, 'material', name="hole_material")
    ET.SubElement(material_hole, 'color', rgba="0.1 0.1 0.1 1")
    
    # Add inner pattern for industrial look
    pattern_count = min(teeth_count // 2, 6)  # Fewer patterns than teeth
    for i in range(pattern_count):
        angle = (2 * np.pi * i) / pattern_count
        pattern_dist = base_radius * 0.6  # Position between center and edge
        pattern_radius = base_radius * 0.15  # Size of pattern feature
        
        pattern_pos_x = pattern_dist * np.cos(angle)
        pattern_pos_y = pattern_dist * np.sin(angle)
        
        pattern_visual = ET.SubElement(link, 'visual')
        ET.SubElement(pattern_visual, 'origin', 
                     xyz=f"{pattern_pos_x} {pattern_pos_y} 0", 
                     rpy="0 0 0")
        pattern_geom = ET.SubElement(pattern_visual, 'geometry')
        ET.SubElement(pattern_geom, 'cylinder', 
                     radius=str(pattern_radius), 
                     length=str(height * 1.02))  # Slightly longer to be visible
        pattern_material = ET.SubElement(pattern_visual, 'material', name="pattern_material")
        ET.SubElement(pattern_material, 'color', rgba="0.2 0.2 0.2 1")
    
    # Base collision shape (simplified for better physics)
    collision = ET.SubElement(link, 'collision')
    ET.SubElement(collision, 'origin', xyz="0 0 0", rpy="0 0 0")
    base_collision = ET.SubElement(collision, 'geometry')
    # Use outer_radius for collision to include teeth
    ET.SubElement(base_collision, 'cylinder', radius=str(outer_radius), length=str(height))
    
    # Convert to string with pretty formatting
    rough_string = ET.tostring(robot, 'utf-8')
    parsed = minidom.parseString(rough_string)
    pretty_xml = parsed.toprettyxml(indent="  ")
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Write to file
    urdf_path = os.path.join(output_dir, f"{gear_name}.urdf")
    with open(urdf_path, "w") as f:
        f.write(pretty_xml)
    
    return urdf_path


def create_basic_gear_set(output_dir):
    """
    Creates a set of basic gears with different sizes and colors.
    
    Args:
        output_dir (str): Directory to save the URDF files
        
    Returns:
        dict: Dictionary mapping gear colors to their URDF paths
    """
    gear_configs = [
        {"name": "gear_red", "diameter": 0.08, "height": 0.02, "teeth": 16, 
         "tooth_height": 0.01, "mass": 0.05, "color": [1, 0, 0, 1]},
        
        {"name": "gear_green", "diameter": 0.06, "height": 0.02, "teeth": 12, 
         "tooth_height": 0.008, "mass": 0.04, "color": [0, 1, 0, 1]},
        
        {"name": "gear_blue", "diameter": 0.1, "height": 0.02, "teeth": 24, 
         "tooth_height": 0.012, "mass": 0.06, "color": [0, 0, 1, 1]},
        
        {"name": "gear_yellow", "diameter": 0.07, "height": 0.02, "teeth": 14, 
         "tooth_height": 0.009, "mass": 0.045, "color": [1, 1, 0, 1]}
    ]
    
    gear_paths = {}
    
    for config in gear_configs:
        color_name = config["name"].split('_')[1]
        urdf_path = create_gear_urdf(
            config["name"], 
            config["diameter"], 
            config["height"], 
            config["teeth"], 
            config["tooth_height"],
            config["mass"],
            config["color"],
            output_dir
        )
        gear_paths[color_name] = urdf_path
        print(f"Created {color_name} gear URDF: {urdf_path}")
    
    return gear_paths


if __name__ == "__main__":
    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                            "../assets/gears")
    create_basic_gear_set(output_dir)
    print(f"Gear URDF files created in {output_dir}")
