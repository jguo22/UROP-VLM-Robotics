# 🤖 AI Agent Development Guide

## 🎯 **Getting Started with Your AI Agent**

This guide will help you build an AI agent that controls dual UR5 robots in your Unity simulation.

---

## 📋 **Prerequisites**

### Unity Setup:
1. **Unity 6000.2.2f1** installed
2. **ur5_simulation** project opened
3. **Scene loaded**: `RobotArmScene.unity` or `SampleScene.unity`
4. **SceneSetup configured**:
   - ✅ Enable "Agentic AI" checkbox
   - ✅ Enable "Python Socket" checkbox

### Python Setup:
```bash
# Install required packages
pip install numpy  # For mathematical operations
# Add other AI/ML libraries as needed (torch, tensorflow, etc.)
```

---

## 🚀 **Quick Start (5 Minutes)**

### Step 1: Test Basic Connection
```bash
# Terminal 1: Start Unity
# - Open ur5_simulation project
# - Load RobotArmScene
# - Enable "Agentic AI" and "Python Socket" in SceneSetup
# - Press Play

# Terminal 2: Run simple demo
cd python_socket_template
python simple_ai_demo.py
```

### Step 2: Verify Communication
You should see:
```
✅ Connected to Unity at 127.0.0.1:65432
📋 Get initial state...
📊 Scene State:
  🤖 ur5_left: Position [-0.30, 0.00, 0.30]
     Suction: OFF
  🤖 ur5_right: Position [0.30, 0.00, 0.30]
     Suction: OFF
```

---

## 🏗️ **Development Architecture**

### **Communication Flow:**
```
Python AI Agent  ←→  Unity Socket  ←→  Robot Controllers
     ↓                    ↓                   ↓
  Decision Logic    JSON Commands      Physical Actions
  Scene Analysis    State Updates      Joint Movements
  Task Planning     Error Handling     Suction Control
```

### **Key Files:**
- **`ai_agent_starter.py`**: Complete AI framework template
- **`simple_ai_demo.py`**: Basic connection and movement demo  
- **`PythonSocketSetup.cs`**: Unity communication handler
- **`AgentRobotController.cs`**: Unity AI integration point

---

## 🎯 **Development Phases**

### **Phase 1: Basic AI Agent (Week 1)**

#### **Goal**: Get a simple rule-based AI working

#### **Tasks**:
1. **Enhance the starter template**:
   ```python
   # Modify ai_agent_starter.py
   def make_decision(self, scene_state, analysis):
       # Add your decision logic here
       if self.should_pick_object(scene_state):
           return self.plan_pick_sequence(scene_state)
       elif self.should_coordinate_robots(scene_state):
           return self.plan_coordination(scene_state)
       # ... more logic
   ```

2. **Implement basic behaviors**:
   - Object detection and tracking
   - Simple pick-and-place sequences
   - Collision avoidance between robots
   - Error recovery

3. **Test with simple tasks**:
   - Move robots to specific positions
   - Pick up a single block
   - Coordinate dual-robot movements

#### **Success Criteria**:
- ✅ AI can observe scene state
- ✅ AI can move robots to target positions
- ✅ AI can activate/deactivate suction
- ✅ Robots don't collide with each other

---

### **Phase 2: Task Planning (Week 2)**

#### **Goal**: Implement intelligent task planning

#### **Enhanced Decision Making**:
```python
class TaskPlanner:
    def __init__(self):
        self.task_queue = []
        self.robot_assignments = {}
        
    def plan_assembly_task(self, scene_state):
        # Analyze available objects
        available_blocks = self.find_available_blocks(scene_state)
        
        # Plan optimal sequence
        task_sequence = self.optimize_task_sequence(available_blocks)
        
        # Assign robots to tasks
        self.assign_robots_to_tasks(task_sequence)
        
        return self.generate_action_sequence()
```

#### **Key Features to Implement**:
- **Multi-step task planning**
- **Optimal robot assignment**
- **Dependency management** (Task B requires Task A completion)
- **Dynamic replanning** when conditions change

#### **Success Criteria**:
- ✅ AI can plan multi-step assembly tasks
- ✅ Optimal robot assignment based on position/capability
- ✅ Handles task dependencies correctly
- ✅ Adapts when objects move or tasks fail

---

### **Phase 3: Learning & Optimization (Week 3+)**

#### **Goal**: Add learning capabilities for improved performance

#### **Learning Approaches**:

**A. Reinforcement Learning**:
```python
class RLAgent:
    def __init__(self):
        self.q_network = self.build_network()
        self.experience_buffer = []
        
    def choose_action(self, state):
        if random.random() < self.epsilon:
            return self.random_action()
        else:
            return self.q_network.predict(state)
            
    def learn_from_experience(self):
        # Train on collected experience
        batch = self.sample_experience()
        self.q_network.train(batch)
```

**B. Imitation Learning**:
```python
class ImitationLearner:
    def learn_from_demonstrations(self, demo_trajectories):
        # Learn from human demonstrations or optimal solutions
        for trajectory in demo_trajectories:
            self.update_policy(trajectory.states, trajectory.actions)
```

#### **Success Criteria**:
- ✅ AI improves performance over time
- ✅ Learns from successful/failed attempts
- ✅ Adapts to new scenarios without reprogramming

---

## 🔧 **API Reference**

### **Scene State Structure**:
```python
{
    "robots": [
        {
            "name": "ur5_left",
            "joint_angles": [0.0, -45.0, 90.0, -45.0, -90.0, 0.0],
            "end_effector_position": [-0.3, 0.0, 0.3],
            "suction_active": False,
            "is_moving": False
        }
    ],
    "objects": [
        {
            "name": "Block_Red",
            "position": [0.5, 0.0, 0.05],
            "rotation": [0.0, 0.0, 0.0, 1.0],
            "is_attached": False
        }
    ],
    "timestamp": 1234567890.123
}
```

### **Available Actions**:

#### **Move Robot**:
```python
command = {
    "type": "execute_action",
    "action_type": "move_robot",
    "robot_name": "ur5_left",
    "parameters": {
        "target_position": [x, y, z],
        "speed": 0.3  # meters/second
    }
}
```

#### **Suction Control**:
```python
# Activate suction
command = {
    "type": "execute_action",
    "action_type": "activate_suction",
    "robot_name": "ur5_left",
    "parameters": {}
}

# Deactivate suction
command = {
    "type": "execute_action",
    "action_type": "deactivate_suction",
    "robot_name": "ur5_left",
    "parameters": {}
}
```

---

## 🎮 **Testing Your AI**

### **Unit Tests**:
```python
def test_collision_avoidance():
    # Test that robots don't collide
    scene_state = create_collision_scenario()
    actions = ai_agent.make_decision(scene_state)
    assert no_collision_risk(actions)

def test_task_completion():
    # Test that tasks are completed successfully
    initial_state = create_test_scenario()
    ai_agent.run_task("pick_and_place")
    final_state = get_scene_state()
    assert task_completed(initial_state, final_state)
```

### **Integration Tests**:
1. **Run simple demo**: `python simple_ai_demo.py`
2. **Test with complex scenarios**: Multiple objects, obstacles
3. **Stress test**: Long-running sessions, error conditions
4. **Performance test**: Measure task completion time and success rate

---

## 🐛 **Common Issues & Solutions**

### **Connection Issues**:
```python
# Problem: "Connection refused"
# Solution: Ensure Unity is running and socket is enabled

# Check Unity Console for:
# "Unity server started, waiting for Python connection..."
```

### **Robot Not Moving**:
```python
# Problem: Commands sent but robot doesn't move
# Solution: Check robot is in correct mode

# In Unity, ensure:
# - SceneSetup has "Agentic AI" enabled
# - Individual robot controllers are disabled
# - No manual input is interfering
```

### **JSON Parsing Errors**:
```python
# Problem: "JSON decode error"
# Solution: Validate command structure

def validate_command(command):
    required_fields = ["type", "timestamp"]
    for field in required_fields:
        if field not in command:
            raise ValueError(f"Missing field: {field}")
```

---

## 🚀 **Advanced Features**

### **Multi-Agent Coordination**:
```python
class MultiAgentCoordinator:
    def coordinate_robots(self, task):
        # Assign subtasks to each robot
        subtasks = self.decompose_task(task)
        
        # Optimize assignments
        assignments = self.optimize_assignments(subtasks, self.robots)
        
        # Synchronize execution
        return self.synchronize_execution(assignments)
```

### **Dynamic Replanning**:
```python
class DynamicPlanner:
    def replan_if_needed(self, current_state, plan):
        if self.plan_is_invalid(current_state, plan):
            new_plan = self.replan(current_state)
            return new_plan
        return plan
```

### **Performance Monitoring**:
```python
class PerformanceMonitor:
    def track_metrics(self):
        return {
            "task_success_rate": self.calculate_success_rate(),
            "average_completion_time": self.get_avg_time(),
            "collision_count": self.get_collision_count(),
            "efficiency_score": self.calculate_efficiency()
        }
```

---

## 📚 **Next Steps**

### **Immediate (This Week)**:
1. ✅ Run `simple_ai_demo.py` to verify setup
2. ✅ Modify `ai_agent_starter.py` with your first decision logic
3. ✅ Test basic robot movements and suction control

### **Short Term (Next 2 Weeks)**:
1. 📋 Implement rule-based task planning
2. 📋 Add multi-robot coordination
3. 📋 Create error handling and recovery
4. 📋 Build performance monitoring

### **Long Term (Next Month)**:
1. 🎯 Add machine learning capabilities
2. 🎯 Implement advanced coordination algorithms
3. 🎯 Create complex assembly tasks
4. 🎯 Optimize for real-world deployment

---

## 🤝 **Getting Help**

### **Debug Information**:
- **Unity Console**: Shows socket connection status and errors
- **Python Output**: Shows AI decision-making process
- **Performance Logs**: Track success rates and timing

### **Useful Debug Commands**:
```python
# Get detailed scene information
scene_state = ai_agent.get_scene_state()
print(json.dumps(scene_state, indent=2))

# Test individual robot control
ai_agent.move_robot("ur5_left", [0, 0, 0.5])
ai_agent.activate_suction("ur5_left")
```

---

**Ready to build your AI agent? Start with `simple_ai_demo.py` and then customize `ai_agent_starter.py` for your specific needs!** 🚀

