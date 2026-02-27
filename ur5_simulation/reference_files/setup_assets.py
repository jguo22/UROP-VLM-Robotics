#!/usr/bin/env python3
"""
Setup Assets Script for UROP-RobotDT

This script automatically copies the necessary URDF and mesh files from the VimaBench
installation to the project's assets directory. This ensures that all required files
are available for the simulation to run properly.

Usage:
    python setup_assets.py
"""

import os
import sys
import shutil
import site
import subprocess

def find_vimabench_assets():
    """Find the VimaBench assets directory."""
    try:
        # Try to get the site-packages directory
        site_packages = site.getsitepackages()[0]
        vima_assets = os.path.join(site_packages, "vima_bench", "tasks", "assets")
        
        if os.path.exists(vima_assets):
            return vima_assets
        
        # If not found, try to find it using pip show
        result = subprocess.run(
            ["pip", "show", "-f", "vima-bench"], 
            capture_output=True, 
            text=True
        )
        
        for line in result.stdout.splitlines():
            if "vima_bench/tasks/assets" in line:
                path = line.strip()
                # Extract the path up to assets
                idx = path.find("vima_bench/tasks/assets")
                if idx != -1:
                    base_path = path[:idx]
                    full_path = os.path.join(base_path, "vima_bench", "tasks", "assets")
                    if os.path.exists(full_path):
                        return full_path
        
        raise FileNotFoundError("Could not locate VimaBench assets directory")
    
    except Exception as e:
        print(f"Error finding VimaBench assets: {e}")
        sys.exit(1)

def setup_assets():
    """Set up the assets directory with necessary URDF and mesh files."""
    # Get the project root directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # Create assets directories if they don't exist
    assets_dir = os.path.join(project_root, "assets")
    ur5_dir = os.path.join(assets_dir, "ur5")
    ur5_visual_dir = os.path.join(ur5_dir, "visual")
    ur5_collision_dir = os.path.join(ur5_dir, "collision")
    block_dir = os.path.join(assets_dir, "block")
    
    os.makedirs(ur5_visual_dir, exist_ok=True)
    os.makedirs(ur5_collision_dir, exist_ok=True)
    os.makedirs(block_dir, exist_ok=True)
    
    # Find VimaBench assets
    vima_assets = find_vimabench_assets()
    print(f"Found VimaBench assets at: {vima_assets}")
    
    # Copy URDF files
    print("Copying URDF files...")
    shutil.copy(
        os.path.join(vima_assets, "ur5", "ur5.urdf"),
        os.path.join(ur5_dir, "ur5.urdf")
    )
    shutil.copy(
        os.path.join(vima_assets, "ur5", "workspace.urdf"),
        os.path.join(ur5_dir, "workspace.urdf")
    )
    shutil.copy(
        os.path.join(vima_assets, "block", "block.urdf"),
        os.path.join(block_dir, "block.urdf")
    )
    shutil.copy(
        os.path.join(vima_assets, "plane", "plane.urdf"),
        os.path.join(assets_dir, "plane.urdf")
    )
    
    # Copy mesh files
    print("Copying mesh files...")
    # Copy visual meshes
    for mesh_file in os.listdir(os.path.join(vima_assets, "ur5", "visual")):
        if mesh_file.endswith(".stl"):
            shutil.copy(
                os.path.join(vima_assets, "ur5", "visual", mesh_file),
                os.path.join(ur5_visual_dir, mesh_file)
            )
    
    # Copy collision meshes
    for mesh_file in os.listdir(os.path.join(vima_assets, "ur5", "collision")):
        if mesh_file.endswith(".stl"):
            shutil.copy(
                os.path.join(vima_assets, "ur5", "collision", mesh_file),
                os.path.join(ur5_collision_dir, mesh_file)
            )
    
    print("Assets setup complete!")
    print(f"Assets directory: {assets_dir}")

if __name__ == "__main__":
    setup_assets()
