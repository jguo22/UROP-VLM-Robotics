from profiler import Profiler
from agent import OpenAIAgent
from agentic_setup import TEST_PROMPT, SIMULTAENOUS_PROMPT
from unityenv import PROJECT_DIR, SOCKET_FILE_NAME
import socket
import json
import time
from sys import argv


SOCKET_CONN_MAX_BYTES = 4096
SOCKET_CONN_MAX_ATTEMPTS = 5
LOOP_PERIOD = 1.5  # Control loop period (s)
SIMULATION_MEASUREMENT_PRECISION = 5  # Decimal places for rounding positions
DEBUG_LOG_PATH = PROJECT_DIR / ".cursor" / "debug-9a7441.log"
DEBUG_SESSION_ID = "9a7441"

IDLE_POLL_INTERVAL = 1   # seconds between motion status polls
IDLE_TIMEOUT = 6.0        # max seconds to wait for robots to stop


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
    time.sleep(0.5)  # let motion start before polling
    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = send_recv(s,
                         {"type": "get_motion_status",
                          "timestamp": time.time()})
        if resp.get("is_idle", False):
            return True
        time.sleep(IDLE_POLL_INTERVAL)
    return False


def execute_command(s, agent, command: dict) -> bool:
    """Send one AI command to Unity. Returns False on fatal failure."""
    resp = send_recv(s, command)
    if not resp.get("success", False):
        agent.log_error(f"Unity execution failure: {resp.get('message')}")
        return False
    return True


def run_episode(sock, agent, profiler) -> bool:
    """
    Execute one full episode: get scene state → AI planning → command loop.
    Returns True on success, False on fatal error.
    """
    profiler.start_frame()

    agent.log_info("Requesting scene state from Unity...")
    scene_state = fetch_scene_state(sock)
    agent.log_info(f"Scene state: {scene_state}")
    profiler.record("get_scene_state")

    agent.log_info("AI is thinking...")
    ai_response = agent.get_ai_decision(scene_state)
    profiler.record("openai_api")

    if not ai_response:
        agent.log_error("No AI response received.")
        return False

    agent.log_info(f"AI Response: {ai_response}")

    json_block = ai_response.split("```")[1].lstrip("json\n")
    clean = "\n".join(line.split("//")[0] for line in json_block.splitlines())
    agent.log_info(f"Extracted and cleaned JSON commands: {clean}")
    commands = json.loads(clean)
    if isinstance(commands, dict):
        commands = [commands]
    profiler.record("parse_response")

    send_control(sock, "start_recording")
    for command in commands:
        agent.log_info(f"Executing: {command}")
        if not execute_command(sock, agent, command):
            return False
        wait_for_idle(sock)
        time.sleep(1)  # make it slower so that replay is more reliable

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
    agent.log_info("Initialized OpenAI Agent")
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
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        # #region agent log
        debug_log(
            run_id,
            "H3",
            "agentic_socket.py:main:socket_create",
            "socket metadata after create",
            {
                "family": int(sock.family),
                "type": int(sock.type),
                "proto": int(sock.proto),
                "fileno": sock.fileno(),
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
                sock.connect((agent.host, agent.port))
                break
            except ConnectionRefusedError:
                agent.log_info(
                    "Waiting for unity sim to start! Retrying in 2 seconds...")
                attempts += 1
                if attempts >= SOCKET_CONN_MAX_ATTEMPTS:
                    agent.log_error("Could not connect to Unity. Exiting...")
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

        agent.log_info("Connected to Unity")
        time.sleep(2)  # Wait for Unity to be ready

        episode = 0
        while True:
            agent.log_info(f"--- Episode {episode} ---")

            # Wait for any lingering motion from previous episode (idempotent
            # on first)
            wait_for_idle(sock)
            send_control(sock, "stop_recording")
            send_control(sock, "reset_scene")
            # wait for arms to finish returning to home
            wait_for_idle(sock)
            # let gear rigidbodies settle onto the table
            time.sleep(2.0)
            success = run_episode(sock, agent, profiler)

            send_control(sock, "stop_recording")
            profiler.save_profile()

            if not success:
                agent.log_error(f"Episode {episode} failed, stopping.")
                break

            episode += 1


if __name__ == "__main__":
    main()
