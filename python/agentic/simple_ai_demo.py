"""
Simple AI Agent Demo for Dual UR5 Robot System
This is a basic example showing how to connect to Unity and control robots
"""

import socket
import json
import time
import math

class SimpleAIDemo:
    def __init__(self, host="127.0.0.1", port=65432):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
    def connect_to_unity(self):
        """Connect to Unity simulation"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Retry connection
            max_retries = 10
            for attempt in range(max_retries):
                try:
                    self.socket.connect((self.host, self.port))
                    self.connected = True
                    print(f"✅ Connected to Unity at {self.host}:{self.port}")
                    return True
                except ConnectionRefusedError:
                    print(f"⏳ Attempt {attempt + 1}: Waiting for Unity...")
                    time.sleep(2)
            
            print("❌ Failed to connect to Unity")
            return False
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False
    
    def send_command(self, command):
        """Send command to Unity"""
        if not self.connected:
            return None
            
        try:
            message = json.dumps(command)
            self.socket.sendall(message.encode('utf-8'))
            
            # Receive response
            response = self.socket.recv(4096)
            if response:
                return json.loads(response.decode('utf-8'))
                
        except Exception as e:
            print(f"❌ Communication error: {e}")
            self.connected = False
        
        return None
    
    def get_scene_state(self):
        """Get current scene state"""
        command = {
            "type": "get_scene_state",
            "timestamp": time.time()
        }
        return self.send_command(command)
    
    def move_robot(self, robot_name, target_position, speed=0.3):
        """Move robot to target position"""
        command = {
            "type": "execute_action",
            "action_type": "move_robot",
            "robot_name": robot_name,
            "parameters": {
                "target_position": target_position,
                "speed": speed
            },
            "timestamp": time.time()
        }
        return self.send_command(command)
    
    def activate_suction(self, robot_name):
        """Activate robot suction"""
        command = {
            "type": "execute_action",
            "action_type": "activate_suction",
            "robot_name": robot_name,
            "parameters": {},
            "timestamp": time.time()
        }
        return self.send_command(command)
    
    def deactivate_suction(self, robot_name):
        """Deactivate robot suction"""
        command = {
            "type": "execute_action",
            "action_type": "deactivate_suction",
            "robot_name": robot_name,
            "parameters": {},
            "timestamp": time.time()
        }
        return self.send_command(command)
    
    def run_demo(self):
        """Run a simple demo sequence"""
        print("🤖 Starting Simple AI Demo...")
        
        if not self.connect_to_unity():
            return
        
        try:
            # Demo sequence
            demo_steps = [
                ("Get initial state", self.demo_get_state),
                ("Move robots to home positions", self.demo_move_home),
                ("Demonstrate coordination", self.demo_coordination),
                ("Test suction systems", self.demo_suction),
                ("Return to rest positions", self.demo_return_home)
            ]
            
            for step_name, step_func in demo_steps:
                print(f"\n📋 {step_name}...")
                if not step_func():
                    print(f"❌ Failed: {step_name}")
                    break
                time.sleep(2)  # Pause between steps
                
        except KeyboardInterrupt:
            print("\n🛑 Demo stopped by user")
        except Exception as e:
            print(f"❌ Demo error: {e}")
        finally:
            self.cleanup()
    
    def demo_get_state(self):
        """Demo: Get and display scene state"""
        scene_state = self.get_scene_state()
        if scene_state:
            print("📊 Scene State:")
            if 'robots' in scene_state:
                for robot in scene_state['robots']:
                    pos = robot['end_effector_position']
                    print(f"  🤖 {robot['name']}: Position [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
                    print(f"     Suction: {'ON' if robot['suction_active'] else 'OFF'}")
            
            if 'objects' in scene_state:
                print(f"  📦 Objects found: {len(scene_state['objects'])}")
                for obj in scene_state['objects']:
                    pos = obj['position']
                    print(f"    - {obj['name']}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
            return True
        return False
    
    def demo_move_home(self):
        """Demo: Move robots to home positions"""
        home_positions = {
            "ur5_left": [-0.3, 0.0, 0.3],
            "ur5_right": [0.3, 0.0, 0.3]
        }
        
        success = True
        for robot_name, position in home_positions.items():
            print(f"  Moving {robot_name} to home position...")
            result = self.move_robot(robot_name, position)
            if not result or not result.get('success', False):
                print(f"    ❌ Failed to move {robot_name}")
                success = False
            else:
                print(f"    ✅ {robot_name} moving to home")
        
        return success
    
    def demo_coordination(self):
        """Demo: Show coordinated movement"""
        print("  Demonstrating coordinated circular motion...")
        
        # Create circular paths for both robots
        center_left = [-0.3, 0.0, 0.3]
        center_right = [0.3, 0.0, 0.3]
        radius = 0.1
        
        for angle in range(0, 360, 45):  # 8 positions around circle
            rad = math.radians(angle)
            
            # Left robot moves clockwise
            left_pos = [
                center_left[0] + radius * math.cos(rad),
                center_left[1] + radius * math.sin(rad),
                center_left[2]
            ]
            
            # Right robot moves counter-clockwise
            right_pos = [
                center_right[0] + radius * math.cos(-rad),
                center_right[1] + radius * math.sin(-rad),
                center_right[2]
            ]
            
            # Move both robots simultaneously
            left_result = self.move_robot("ur5_left", left_pos, speed=0.2)
            right_result = self.move_robot("ur5_right", right_pos, speed=0.2)
            
            if not (left_result and right_result):
                return False
            
            time.sleep(1)  # Wait between movements
        
        print("  ✅ Coordination demo completed")
        return True
    
    def demo_suction(self):
        """Demo: Test suction systems"""
        robots = ["ur5_left", "ur5_right"]
        
        for robot in robots:
            print(f"  Testing suction for {robot}...")
            
            # Activate suction
            result = self.activate_suction(robot)
            if result and result.get('success', False):
                print(f"    ✅ {robot} suction activated")
                time.sleep(1)
                
                # Deactivate suction
                result = self.deactivate_suction(robot)
                if result and result.get('success', False):
                    print(f"    ✅ {robot} suction deactivated")
                else:
                    print(f"    ❌ Failed to deactivate {robot} suction")
                    return False
            else:
                print(f"    ❌ Failed to activate {robot} suction")
                return False
        
        return True
    
    def demo_return_home(self):
        """Demo: Return robots to rest positions"""
        rest_positions = {
            "ur5_left": [-0.5, 0.0, 0.2],
            "ur5_right": [0.5, 0.0, 0.2]
        }
        
        success = True
        for robot_name, position in rest_positions.items():
            print(f"  Returning {robot_name} to rest position...")
            result = self.move_robot(robot_name, position, speed=0.1)
            if not result or not result.get('success', False):
                success = False
        
        return success
    
    def cleanup(self):
        """Cleanup resources"""
        if self.socket:
            self.socket.close()
        print("🧹 Demo cleanup completed")

if __name__ == "__main__":
    demo = SimpleAIDemo()
    demo.run_demo()

