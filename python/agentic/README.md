# OpenAI-Powered Robot Agent

This directory contains an intelligent AI agent that uses OpenAI's GPT-4 to control dual UR5 robots in Unity simulation.

## Files

- **agentic_socket.py** - Main script with OpenAI agent integrated into Unity socket communication
- **agent.py** - Alternative standalone agent implementation with function calling
- **simple_ai_demo.py** - Simple demo showing basic robot control (for testing)

## Setup

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set your OpenAI API key:**
   ```bash
   # Windows PowerShell
   $env:OPENAI_API_KEY="your-api-key-here"
   
   # Linux/Mac
   export OPENAI_API_KEY="your-api-key-here"
   ```

3. **Configure socket connection (optional):**
   Edit `../socket.env` to set the Unity connection:
   ```
   HOST="127.0.0.1"
   PORT=65432
   ```

## Running the Agent

1. **Start Unity simulation** first (it should be listening on the configured port)

2. **Run the OpenAI agent:**
   ```bash
   python agentic_socket.py
   ```
   
   This will:
   - Connect to Unity using the socket pattern Unity expects
   - Request scene state each iteration
   - Use OpenAI to make intelligent decisions
   - Send commands back to Unity
   - Display AI reasoning and results

## How It Works

The `agentic_socket.py` integrates OpenAI intelligence into the Unity socket communication:

1. **Connect** - Establishes socket connection to Unity (with retry logic)
2. **Request State** - Sends `get_scene_state` command to Unity
3. **Receive State** - Gets current robot and object positions from Unity
4. **AI Thinks** - GPT-4 analyzes the scene and generates a JSON command
5. **Execute** - Sends AI's command to Unity
6. **Get Result** - Receives Unity's response
7. **Repeat** - Loops at 2 Hz

### AI-Generated Commands

The AI generates JSON commands in Unity's expected format:

```json
{
  "type": "execute_action",
  "action_type": "move_robot",
  "robot_name": "ur5_left",
  "parameters": {
    "target_position": [-0.3, 0.0, 0.3],
    "speed": 0.3
  }
}
```

Available action types:
- `move_robot` - Move robot to target position
- `activate_suction` - Activate gripper
- `deactivate_suction` - Release gripper
- `get_scene_state` - Request current state

### Features

- ✅ Autonomous decision-making using GPT-4
- ✅ Real-time environment observation
- ✅ Dual robot coordination
- ✅ Collision avoidance reasoning
- ✅ Task planning and execution
- ✅ Conversation history for context

## File Structure

### agentic_socket.py (Main)
The primary agent implementation:
- Uses original socket connection pattern (Unity-compatible)
- Reads HOST/PORT from socket.env
- Requests scene state from Unity each iteration
- Sends scene data to OpenAI GPT-4 for decisions
- Parses AI response for JSON commands
- Sends commands back to Unity
- Maintains conversation history for context

### agent.py (Alternative)
Standalone agent class with:
- OpenAI function calling approach
- Built-in socket management
- More modular structure
- Can be used as a library

### simple_ai_demo.py (Testing)
Basic demonstration script for:
- Testing socket connectivity
- Sequential pre-programmed actions
- Verifying Unity communication
- No AI required (good for debugging)

## Configuration

Edit `agentic_socket.py` to customize:

```python
# Adjust control loop frequency (line 175)
time.sleep(0.5)  # 2 Hz (0.5 seconds between iterations)

# Modify system prompt (line 28-58) to change AI behavior

# Adjust conversation history limit (line 93)
if len(conversation_history) > 20:  # Keep last 20 messages
```

## Notes

- The agent runs at 2 Hz by default (observes/acts every 0.5 seconds)
- Conversation history is trimmed to prevent token limit issues
- Function calls are executed immediately and results fed back to the AI
- System prompt defines robot workspace and safety rules

## Troubleshooting

**"OPENAI_API_KEY not set"**
- Make sure you've exported the environment variable

**"Failed to connect to Unity"**  
- Ensure Unity simulation is running first
- Check socket.env for correct HOST/PORT

**"No response from Unity"**
- Verify socket connection is working
- Check Unity console for errors

**Token limit errors**
- Reduce `max_history` parameter in `run_agent_loop()`
- Shorten system prompt if needed
