/*
contains JSON serializable data structures for IKSocketSetup and AgenticSocketSetup AI communication
*/

[System.Serializable]
public class IKRobotStateData
{
    public string robot_name;
    public float[] current_joint_angles;
    public float[] end_effector_position;
}

[System.Serializable]
public class IKCommand
{
    public string type;
    public string robot_name;
    public float[] joint_angles;
    public float timestamp;
}

[System.Serializable]
public class SimulIKCommand
{
    public string type = "execute_action";
    public string action_type = "solve_ik";
    public float[] ur5_left;
    public float[] ur5_right;
    public float timestamp;
}

[System.Serializable]
public class IKResponse
{
    public bool success;
    public string message;
    public IKRobotStateData robot_state;
    public float timestamp;
}

[System.Serializable]
public class SimulIKResponse
{
    public bool success;
    public string message = "run_simul_ik";
    public IKRobotStateData[] robot_states;
    public float timestamp;
}


[System.Serializable]
public class AICommand
{
    public string type;
    public string action_type;
    public string robot_name;
    public ActionParameters parameters;
    public float timestamp;
}

[System.Serializable]
public class SimulAICommand
{
    public string type;
    public SimulArmCommand ur5_left;
    public SimulArmCommand ur5_right;
    public float timestamp;
}

[System.Serializable]
public class SimulArmCommand
{
    public string action_type;
    public ActionParameters parameters;
}

[System.Serializable]
public class ActionParameters
{
    public float[] target_position;
    public float[] joint_angles;
    //public float speed;
}

[System.Serializable]
public class AIResponse
{
    public bool success;
    public string message;
    public float timestamp;
}

[System.Serializable]
public class SceneStateData
{
    public RobotStateData[] robots;
    // public ObjectStateData[] objects;
    public float timestamp;
}

[System.Serializable]
public class RobotStateData
{
    public string name;
    // public float[] joint_angles;
    public float[] end_effector_position;
    public ObjectStateData[] objects;
    public bool suction_active;
    //public bool is_moving;
}

[System.Serializable]
public class ObjectStateData
{
    public string name;
    public float[] position;
    // public float[] rotation;
    public bool is_attached;
}
