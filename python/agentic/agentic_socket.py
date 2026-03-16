import sys
import socket
import json
import time
from pathlib import Path
from sys import argv

sys.path.insert(0, str(Path(__file__).parent.parent))
from profiler import Profiler

from ik_solver import UR5IKSolver
from agent import OpenAIAgent
from agentic_setup import TEST_PROMPT, SIMULTAENOUS_PROMPT
from unityenv import PROJECT_DIR, SOCKET_FILE_NAME

SOCKET_CONN_MAX_BYTES = 4096
SOCKET_CONN_MAX_ATTEMPTS = 5
LOOP_PERIOD = 1.5  # Control loop period (s)
SIMULATION_MEASUREMENT_PRECISION = 5  # Decimal places for rounding positions
DEBUG_LOG_PATH = PROJECT_DIR / ".cursor" / "debug-9a7441.log"
DEBUG_SESSION_ID = "9a7441"

ik_solver = UR5IKSolver()


def debug_log(run_id, hypothesis_id, location, message, data):
    payload = {
        "sessionId": DEBUG_SESSION_ID,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=True) + "\n")


def solve_ik(ik_response):
    robot_state = ik_response["robot_state"]
    current_joint_angles = robot_state["current_joint_angles"]
    target_end_effector_position = robot_state["end_effector_position"]

    ik_success, ik_joint_angles, _ = ik_solver.calculate_inverse_kinematics(
        ik_solver.transform_coordinates(target_end_effector_position),
        # calc_unit_quat,
        q0=current_joint_angles  # Use current joint angles as initial guess
    )

    if not ik_success or ik_joint_angles is None:
        return None

    """
    {
        "type": "execute_action",
        "action_type": "solve_ik",
        "robot_name": "ur5_left" | "ur5_right",
        "parameters": {
            "joint_angles": [angle1, angle2, angle3, angle4, angle5, angle6]
        }
    }
    """

    ik_dict = {
        "type": "execute_action",
        "action_type": "solve_ik",
        "robot_name": robot_state.get("robot_name"),
        "parameters": {
            "joint_angles": ik_joint_angles.tolist()
        },
        "timestamp": time.time()
    }

    return json.dumps(ik_dict)


def solve_simul_ik(ik_response):
    robot_states = ik_response["robot_states"]

    ik_dict = {
        "type": "execute_action",
        "ur5_left": {
            "action_type": "stationary"
        },
        "ur5_right": {
            "action_type": "stationary"
        }
    }

    for robot_state in robot_states:
        robot_name = robot_state["robot_name"]
        current_joint_angles = robot_state["current_joint_angles"]
        target_end_effector_position = robot_state["end_effector_position"]

        ik_success, ik_joint_angles, _ = ik_solver.calculate_inverse_kinematics(
            ik_solver.transform_coordinates(target_end_effector_position),
            # calc_unit_quat,
            q0=current_joint_angles  # Use current joint angles as initial guess
        )

        if not ik_success or ik_joint_angles is None:
            return None

        ik_dict[robot_name] = {
            "action_type": "solve_ik",
            "parameters": {
                "joint_angles": ik_joint_angles.tolist()
            }
        }

    ik_dict["timestamp"] = time.time()

    return json.dumps(ik_dict)


def round_floats(obj, precision=SIMULATION_MEASUREMENT_PRECISION):
    if isinstance(obj, float):
        return round(obj, precision)
    if isinstance(obj, dict):
        return {k: round_floats(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [round_floats(i) for i in obj]
    return obj


def main():
    profiler = Profiler()
    run_id = f"run-{int(time.time() * 1000)}"
    prompt = ''
    if len(argv) < 2:
        print("Moving arms one at a time")
        prompt = TEST_PROMPT
    elif argv[1] == '/one':
        print("Moving arms one at a time")
        prompt = TEST_PROMPT
    elif argv[1] == '/simul':
        print("Moving arms simultaneously")
        prompt = SIMULTAENOUS_PROMPT
    else:
        print("Too many inputs or wrong inputs")
        return

    agent = OpenAIAgent(prompt)
    agent.info("Initialized OpenAI Agent")
    print(f"Host: {repr(agent.host)}")
    print(f"Port: {repr(agent.port)}")
    profiler.start_frame()
    # #region agent log
    debug_log(
        run_id,
        "H4",
        "agentic_socket.py:main:init",
        "cwd and socket env path",
        {
            "cwd": str(PROJECT_DIR),
            "socket_env_abs_path": str(SOCKET_FILE_NAME),
            "socket_env_exists": SOCKET_FILE_NAME.exists(),
        },
    )
    # #endregion

    # Sending data (client)
    attempts = 0
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # #region agent log
        debug_log(
            run_id,
            "H3",
            "agentic_socket.py:main:socket_create",
            "socket metadata after create",
            {
                "family": int(s.family),
                "type": int(s.type),
                "proto": int(s.proto),
                "fileno": s.fileno(),
            },
        )
        # #endregion
        while True:
            try:
                # #region agent log
                debug_log(
                    run_id, "H1_H2", "agentic_socket.py:main:pre_connect", "connect target value/type checks", {
                        "host_repr": repr(
                            agent.host), "host_type": str(
                            type(
                                agent.host)), "host_len": len(
                            agent.host) if isinstance(
                            agent.host, str) else None, "host_codepoints": [
                                ord(c) for c in agent.host] if isinstance(
                                    agent.host, str) else None, "port_repr": repr(
                                        agent.port), "port_type": str(
                                            type(
                                                agent.port)), "port_in_range": isinstance(
                                                    agent.port, int) and 0 <= agent.port <= 65535, }, )
                # #endregion
                try:
                    gai = socket.getaddrinfo(
                        agent.host, agent.port, socket.AF_INET, socket.SOCK_STREAM)
                    # #region agent log
                    debug_log(
                        run_id,
                        "H5",
                        "agentic_socket.py:main:getaddrinfo",
                        "getaddrinfo(AF_INET, SOCK_STREAM) result",
                        {
                            "result_count": len(gai),
                            "first_result": str(gai[0]) if gai else None,
                        },
                    )
                    # #endregion
                except Exception as gai_error:
                    # #region agent log
                    debug_log(
                        run_id,
                        "H5",
                        "agentic_socket.py:main:getaddrinfo_error",
                        "getaddrinfo failed",
                        {
                            "error_type": type(gai_error).__name__,
                            "error_str": str(gai_error),
                        },
                    )
                    # #endregion
                s.connect((agent.host, agent.port))
                break
            except ConnectionRefusedError:
                agent.info(
                    "Waiting for unity sim to start! Retrying in 2 seconds...")
                attempts += 1
                if attempts >= SOCKET_CONN_MAX_ATTEMPTS:
                    agent.error("Could not connect to Unity. Exiting...")
                    return
                time.sleep(2)
            except Exception as conn_error:
                # #region agent log
                debug_log(
                    run_id,
                    "H1_H2_H3_H5",
                    "agentic_socket.py:main:connect_error",
                    "connect raised non-ConnectionRefusedError",
                    {
                        "error_type": type(conn_error).__name__,
                        "error_str": str(conn_error),
                        "errno": getattr(conn_error, "errno", None),
                        "host_repr": repr(agent.host),
                        "port_repr": repr(agent.port),
                    },
                )
                # #endregion
                raise

        agent.info("Connected to Unity")
        profiler.record("connect")
        time.sleep(2)  # Wait for Unity to be ready
        agent.info("Requesting initial scene state from Unity...")

        # First, request scene state from Unity
        get_state_command = {
            "type": "get_scene_state",
            "timestamp": time.time()
        }
        s.sendall(json.dumps(get_state_command).encode('utf-8'))
        agent.info(f"Sent to Unity: {get_state_command['type']}")

        # Wait for Unity's response with scene state
        data = s.recv(SOCKET_CONN_MAX_BYTES)
        if not data:
            agent.error("Connection closed by Unity")
            return

        scene_state = json.loads(data.decode('utf-8'))
        scene_state = round_floats(scene_state)
        robot_data = scene_state.get('robots', [])
        robot_name_list = []
        agent.info("Received scene state from Unity:")
        agent.info(scene_state)
        agent.info("---")
        agent.info(f"Robots in scene: {len(robot_data)}")
        for robot in robot_data:
            agent.info(f"Robot Name: {robot.get('name')}")
            robot_name_list.append(robot.get('name'))
        profiler.record("get_scene_state")

        # Get AI decision based on scene state
        agent.info("AI is thinking...")
        ai_response = agent.get_ai_decision(scene_state)
        profiler.record("openai_api")

        if not ai_response:
            agent.error("No AI response received, exiting...")
            return

        agent.info(f"Prompt: {agent.prompt}")
        agent.info(f"AI Response: {ai_response}")

        # find list of JSON commands in response
        json_commands = ai_response.split("```")[1].lstrip("json\n")
        # agent.info(f"Extracted JSON commands: {json_commands}")

        json_command_lines = json_commands.splitlines()
        clean_json_commands = ''
        for line in json_command_lines:
            # Remove comments
            clean_json_commands += line.split("//")[0] + "\n"
        agent.info(
            f"Extracted and cleaned JSON commands: {clean_json_commands}")

        data_commands = json.loads(clean_json_commands)
        profiler.record("parse_response")

        # for robot_name in robot_name_list:
        #     with open(f"{robot_name}.py", "w") as f: # write json commands to robot specific files
        #         f.write(f"data_commands = {json.dumps(data_commands, indent=4)}\n")

        """
        In main loop, send each command to Unity
        Wait for Unity response, if Unity requests IK, run IK solver and send joint angles back to Unity
        Repeat until all commands are executed
        """

        for command in data_commands:
            print("command")
            print(command)
            s.sendall(json.dumps(command).encode('utf-8'))
            agent.info(f"Processing command and sending to Unity: {command}")

            data = s.recv(SOCKET_CONN_MAX_BYTES)
            if not data:
                agent.error("Connection closed by Unity")
                return

            response = json.loads(data.decode('utf-8'))
            success = response.get('success', False)
            if not success:
                agent.error(
                    f"Unity reported execution failure: {response.get('message')}")
                return

            if response.get('message') == 'run_ik':  # RUN IK SOLVER
                agent.info("Running IK solver...")
                ik_command = solve_ik(response)
                if ik_command is not None:
                    s.sendall(ik_command.encode('utf-8'))
                    agent.info("Executing IK movement...")
                    ik_response_data = s.recv(SOCKET_CONN_MAX_BYTES)
                    if ik_response_data:
                        ik_response = json.loads(
                            ik_response_data.decode('utf-8'))
                        ik_success = ik_response.get('success', False)
                        if not ik_success:
                            agent.error(
                                f"Unity reported IK execution failure: {ik_response.get('message')}")
                        agent.info(
                            f"Unity response: {ik_response.get('message')}")
                else:
                    agent.debug(
                        "IK solver failed to find a solution, target gear likely out of range of arm.")

            elif response.get('message') == 'run_simul_ik':
                agent.info("Running IK solver...")
                ik_command = solve_simul_ik(response)
                if ik_command is not None:
                    s.sendall(ik_command.encode('utf-8'))
                    agent.info("Executing IK movement...")
                    ik_response_data = s.recv(SOCKET_CONN_MAX_BYTES)
                    if ik_response_data:
                        ik_response = json.loads(
                            ik_response_data.decode('utf-8'))
                        ik_success = ik_response.get('success', False)
                        if not ik_success:
                            agent.error(
                                f"Unity reported IK execution failure: {ik_response.get('message')}")
                        agent.info(
                            f"Unity response: {ik_response.get('message')}")
                else:
                    agent.debug(
                        "IK solver failed to find a solution, target gear likely out of range of arm.")

            time.sleep(LOOP_PERIOD)  # Control loop period

        profiler.record("command_loop")
        profiler.end_frame()
        profiler.save_profile()
        agent.info("All commands processed. Closing connection.")


if __name__ == "__main__":
    main()
