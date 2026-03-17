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

IDLE_POLL_INTERVAL = 0.5   # seconds between motion status polls
IDLE_TIMEOUT = 30.0        # max seconds to wait for robots to stop

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


def send_recv(s, payload: dict) -> dict:
    """Send a JSON payload and return the parsed JSON response."""
    s.sendall(json.dumps(payload).encode('utf-8'))
    data = s.recv(SOCKET_CONN_MAX_BYTES)
    if not data:
        raise ConnectionError("Connection closed by Unity")
    return json.loads(data.decode('utf-8'))


def send_control(s, control_type: str) -> bool:
    """Send a scene control command (reset_scene / start_recording / stop_recording)."""
    resp = send_recv(s, {"type": control_type, "timestamp": time.time()})
    return resp.get("success", False)


def fetch_scene_state(s) -> dict:
    """Request and return the current scene state from Unity."""
    resp = send_recv(s, {"type": "get_scene_state", "timestamp": time.time()})
    return round_floats(resp)


def wait_for_idle(s, timeout: float = IDLE_TIMEOUT) -> bool:
    """
    Poll Unity until all robot joints are idle (velocity near zero) or timeout.
    Returns True if idle was reached, False if timed out.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = send_recv(s, {"type": "get_motion_status", "timestamp": time.time()})
        if resp.get("is_idle", False):
            return True
        time.sleep(IDLE_POLL_INTERVAL)
    return False


def execute_command(s, agent, command: dict) -> bool:
    """
    Send one AI command to Unity and handle the IK round-trip if needed.
    Returns False on fatal failure.
    """
    resp = send_recv(s, command)
    if not resp.get("success", False):
        agent.error(f"Unity execution failure: {resp.get('message')}")
        return False

    msg = resp.get("message")
    if msg == "run_ik":
        agent.info("Running IK solver...")
        ik_cmd = solve_ik(resp)
        if ik_cmd is None:
            agent.debug("IK solver failed — target likely out of range.")
            return True  # non-fatal
        ik_resp = send_recv(s, json.loads(ik_cmd))
        if not ik_resp.get("success", False):
            agent.error(f"IK execution failure: {ik_resp.get('message')}")
        agent.info(f"Unity IK response: {ik_resp.get('message')}")
    elif msg == "run_simul_ik":
        agent.info("Running simul IK solver...")
        ik_cmd = solve_simul_ik(resp)
        if ik_cmd is None:
            agent.debug("Simul IK solver failed — target likely out of range.")
            return True
        ik_resp = send_recv(s, json.loads(ik_cmd))
        if not ik_resp.get("success", False):
            agent.error(f"Simul IK execution failure: {ik_resp.get('message')}")
        agent.info(f"Unity simul IK response: {ik_resp.get('message')}")
    return True


def run_episode(s, agent, profiler) -> bool:
    """
    Execute one full episode: get scene state → AI planning → command loop.
    Returns True on success, False on fatal error.
    """
    profiler.start_frame()

    agent.info("Requesting scene state from Unity...")
    scene_state = fetch_scene_state(s)
    agent.info(f"Scene state: {scene_state}")
    profiler.record("get_scene_state")

    agent.info("AI is thinking...")
    ai_response = agent.get_ai_decision(scene_state)
    profiler.record("openai_api")

    if not ai_response:
        agent.error("No AI response received.")
        return False

    agent.info(f"AI Response: {ai_response}")

    json_block = ai_response.split("```")[1].lstrip("json\n")
    clean = "\n".join(line.split("//")[0] for line in json_block.splitlines())
    agent.info(f"Extracted and cleaned JSON commands: {clean}")
    commands = json.loads(clean)
    if isinstance(commands, dict):
        commands = [commands]
    profiler.record("parse_response")

    for i, command in enumerate(commands):
        agent.info(f"Executing: {command}")
        send_control(s, "start_recording" if i == 0 else "resume_recording")
        if not execute_command(s, agent, command):
            send_control(s, "pause_recording")
            return False
        wait_for_idle(s)
        send_control(s, "pause_recording")

    profiler.record("command_loop")
    profiler.end_frame()
    return True


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
        time.sleep(2)  # Wait for Unity to be ready

        episode = 0
        while True:
            agent.info(f"--- Episode {episode} ---")

            # Wait for any lingering motion from previous episode (idempotent on first)
            wait_for_idle(s)
            send_control(s, "stop_recording")
            send_control(s, "reset_scene")
            wait_for_idle(s)        # wait for arms to finish returning to home
            time.sleep(2.0)         # let gear rigidbodies settle onto the table
            success = run_episode(s, agent, profiler)

            send_control(s, "stop_recording")
            profiler.save_profile()

            if not success:
                agent.error(f"Episode {episode} failed, stopping.")
                break

            episode += 1


if __name__ == "__main__":
    main()
