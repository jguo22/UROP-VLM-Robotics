import socket
import json
import time

from unityenv import get_socket_env
from ik_solver import UR5IKSolver

ik_solver = UR5IKSolver()

def get_ik_decision(scene_state):
    robot_state = scene_state["robot_state"]
    
    if len(robot_state["robot_name"]) == 0: # no robot available, skip IK
        return None, False

    current_joint_angles = robot_state["current_joint_angles"]
    target_end_effector_position = robot_state["end_effector_position"]

    ik_success, ik_joint_angles, ik_iterations = ik_solver.calculate_inverse_kinematics(
        ik_solver.transform_coordinates(target_end_effector_position),
        # calc_unit_quat,
        q0 = current_joint_angles  # Use current joint angles as initial guess
    )

    print(f"IK SUCCESS: {ik_success}")
    print(ik_joint_angles)
    print(ik_iterations)

    """
    public class IKCommand
    {
        public string type;
        public string robot_name;
        public ActionParameters parameters;
        public float timestamp;
    }
    """

    ik_dict = {
        "type": "move_to_target" if ik_success else "waiting",
        "robot_name": robot_state.get("robot_name"),
        "joint_angles": ik_joint_angles.tolist() if ik_success else [],
        "timestamp": time.time()
    }

    return json.dumps(ik_dict), ik_success


def main():
    HOST, PORT = get_socket_env()
    print("HOST:", HOST)
    print("PORT:", PORT)

    # Sending data (client)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        while True:
            try:
                s.connect((HOST, PORT))
                break
            except ConnectionRefusedError:
                print("Waiting for unity sim to start!")
                time.sleep(2)

        print("✅ Connected to Unity")
        
        # iteration = 0
        ik_waiting = False
        while True:
            # iteration += 1
            # print(f"\n{'='*60}")
            # print(f"Iteration {iteration}")
            # print(f"{'='*60}")
            
            # First, request scene state from Unity
            get_state_command = {
                "type": "waiting",
                "timestamp": time.time()
            }
            s.sendall(json.dumps(get_state_command).encode('utf-8'))
            if not ik_waiting: print(f"📤 Sent to Unity: {get_state_command['type']}")
            
            # Wait for Unity's response with scene state
            try:
                data = s.recv(4096)
                if not data:
                    print("❌ Connection closed by Unity")
                    break
            except ConnectionAbortedError:
                print("❌ Connection aborted while waiting for Unity response")
                break
            
            try:
                scene_state = json.loads(data.decode('utf-8'))
            except json.JSONDecodeError as e:
                print(f"⚠️ Could not parse scene state as JSON: {e}")
                time.sleep(0.5)
                continue
            except ConnectionAbortedError:
                print("❌ Connection aborted while waiting for Unity response")
                break
            if not ik_waiting: print("📥 Received scene state from Unity")

            if not scene_state["success"]:
                print("❌ Unity reported failure: ", scene_state["message"])
                time.sleep(0.5)
                continue

            ik_command, ik_success = get_ik_decision(scene_state) # returns json string command
            if ik_command and ik_success:
                ik_waiting = False
                try:
                    print("\n🧠 IK calculated")
                    s.sendall(ik_command.encode('utf-8'))
                    print("🔧 Executing IK movement")

                    # Wait for Unity's response
                    response_data = s.recv(4096)
                    if response_data:
                        response = json.loads(response_data.decode('utf-8'))
                        success = response.get('success', False)
                        if not success:
                            print("❌ Unity reported IK execution failure: ", response.get("message"))
                        print("✅ Unity response", response.get('message'))
                except json.JSONDecodeError as e:
                    print(f"⚠️ Could not parse IK response as JSON: {e}")
                except ConnectionAbortedError:
                    print("❌ Connection aborted while waiting for Unity response")
                    break
            elif ik_command and not ik_success:
                print("\n⏳ IK failed to calculate this iteration")
            else:
                ik_waiting = True
            
            # Wait before next iteration
            time.sleep(0.5)  # 2 Hz control loop

if __name__ == "__main__":
    main()
