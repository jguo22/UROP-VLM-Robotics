import json
import logging
from openai import OpenAI

from unityenv import get_socket_env, get_agentic_env, get_agentic_log_file

class OpenAIAgent:
    """
    A class to manage OpenAI API interactions for the agent.
    """

    def __init__(self, prompt, model = "gpt-4o", temperature = 0.4):
        self.host, self.port = get_socket_env()

        self.model = model
        self.client = OpenAI(api_key=get_agentic_env())
        self.prompt = prompt
        self.temperature = temperature

        logging.basicConfig(
            filename=get_agentic_log_file(),
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger("AgentLogger")
        self.logger.setLevel(logging.DEBUG)
        # self.logger.addHandler(logging.FileHandler(AGENTIC_LOG_FILE_NAME))
        
    def get_ai_decision(self, scene_data): # TEST_PROMPT
        """Get AI decision based on current scene state"""
        prompt = self.prompt.replace("<scene_data>", json.dumps(scene_data))

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature = self.temperature
        )

        return response.choices[0].message.content

    def test_api(self):
        """Test OpenAI API connection"""
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "user", "content": "Say a funny joke!"}
            ]
        )
        self.log_info(completion.choices[0].message.content)

    def log_debug(self, message):
        self.logger.debug(message)

    def log_info(self, message):
        self.logger.info(message)

    def log_error(self, message):
        self.logger.error(message)


if __name__ == "__main__":
    test_agent = OpenAIAgent("")
    test_agent.test_api()
