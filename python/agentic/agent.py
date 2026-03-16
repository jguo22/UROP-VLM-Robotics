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

        #client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = model
        self.client = OpenAI(api_key=get_agentic_env())
        self.prompt = prompt
        self.temperature = temperature
        # self.conversation_history = [{"role": "system", "content": TEST_PROMPT}]

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

    def old_ai_decision(self, scene_data):
        """Get AI decision based on current scene state"""
        user_message = f"""Current robot environment state:
    {json.dumps(scene_data, indent=2)}

    Analyze this situation and decide the next action. Respond with a JSON command to control the robots."""
        
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })
        
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=self.conversation_history,
                temperature=0.7,
                max_tokens=1000
            )
            
            ai_response = response.choices[0].message.content
            self.conversation_history.append({
                "role": "assistant",
                "content": ai_response
            })
            
            # Manage conversation history length
            if len(self.conversation_history) > 20:
                self.conversation_history[1:3] = []  # Remove oldest user/assistant pair
            
            return ai_response
            
        except Exception as e:
            self.error(f"❌ OpenAI API error: {e}")
            return None

    def test_api(self):
        """Test OpenAI API connection"""
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "user", "content": "Say a funny joke!"}
            ]
        )
        self.info(completion.choices[0].message.content)

    def info(self, message):
        self.logger.info(message)

    def debug(self, message):
        self.logger.debug(message)

    def error(self, message):
        self.logger.error(message)


if __name__ == "__main__":
    test_agent = OpenAIAgent()
    test_agent.test_api()


"""
        scene_state = json.loads(data.decode('utf-8'))
        scene_state = round_floats(scene_state)
        # print(f"📥 Received scene state from Unity")
        agent.info("Received scene state from Unity:")
        agent.info(scene_state)
        agent.info("---")
        agent.info(f"Robots in scene: {len(scene_state.get('robots', []))}")

        # Get AI decision based on scene state
        # print("\n🧠 AI is thinking...")
        agent.info("AI is thinking...")
        ai_response = agent.get_ai_decision(scene_state)
"""