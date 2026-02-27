from datetime import datetime
from pathlib import Path

PYTHON_DIR = Path(__file__).parent
PROJECT_DIR = Path(__file__).parent.parent.parent
SOCKET_FILE_NAME = PROJECT_DIR / "socket.env"
AGENTIC_CONFIG_FILE_NAME = PROJECT_DIR / "config.env"
LOG_FOLDER = PROJECT_DIR / "_logs"
AGENTIC_LOG_FILE_NAME = LOG_FOLDER / "agent.log"


def get_socket_env():
    HOST = ''  # Standard loopback interface address (localhost)
    PORT = 0  # Port to listen on (non-privileged ports are > 1023)

    with open(SOCKET_FILE_NAME, mode="r") as f:
        lines = f.readlines()
        for line in lines:
            # remove newline and comments
            clean_line = line.strip().split("#")[0]
            if ("HOST" in clean_line):
                HOST = clean_line.replace('"', "").replace("HOST=", "").strip()
            elif ("PORT" in clean_line):
                PORT = int(clean_line.replace("PORT=", "").strip())

    return HOST, PORT


def get_agentic_env():
    with open(AGENTIC_CONFIG_FILE_NAME, mode="r") as f:
        lines = f.readlines()
        for line in lines:
            clean_line = line.replace("\n", "")
            if ("OPENAI_API_KEY" in clean_line):
                return clean_line.replace("OPENAI_API_KEY=", "")


def get_agentic_log_file():
    LOG_FOLDER.mkdir(parents=True, exist_ok=True)
    return AGENTIC_LOG_FILE_NAME.with_stem(
        f"{AGENTIC_LOG_FILE_NAME.stem}_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
