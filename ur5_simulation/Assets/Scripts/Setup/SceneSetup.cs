using System;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control; //for ManualController
using UnityEngine;

[System.Serializable]
public class RobotArms
{
    public GameObject robot;
    public GameObject[] targets;
    // public int ikPort;
}

public class SceneSetup : MonoBehaviour
{
    [Header("Scene References")]
    [SerializeField]
    public RobotArms[] robotArms;

    [HideInInspector]
    public GameObject[] robots;

    [HideInInspector]
    public GameObject[] targets;
    private List<GameObject[]> robotTargetsList = new List<GameObject[]>();

    [Header("IK Socket")]
    public bool ikSocket = false;

    [Header("Agentic AI")]
    public bool allRobotsActive = false;
    public bool agenticSocket = false;
    public bool simulAgenticSocket = false;

    [HideInInspector]
    public AgentRobotController agentRobotController;

    [HideInInspector]
    public AgentTrajectoryController agentTrajectoryController;

    [HideInInspector]
    public AgentTrajectory agentTrajectory;

    [HideInInspector]
    public AgenticSocketSetup agenticSocketSetup;

    [HideInInspector]
    public SimulAgenticSocketSetup simulAgenticSocketSetup;

    [HideInInspector]
    public IKSocketSetup ikSocketSetup;

    [HideInInspector]
    public int selectedRobotIndex;

    [HideInInspector]
    public UR5Controller ur5Controller;

    //[HideInInspector]
    //public IKController ikController;
    [HideInInspector]
    public SuctionController suctionController;

    private SceneResetController sceneResetController;

    // [HideInInspector]
    // public UR5IKController ikController;
    // [HideInInspector]
    // public UR5IKServer ikServer;


    void Start()
    {
        if (robotArms == null || robotArms.Length == 0)
        {
            Debug.LogError("Robots not assigned in the inspector.");
            return;
        }

        // foreach (GameObject target in targets)
        // {
        //     Collider blockCollider = target.GetComponent<Collider>();
        //     Debug.Log(target.name + " Original Position: " + blockCollider.bounds.center);
        // }

        int robotCount = robotArms.Length;
        robots = new GameObject[robotCount];
        int robotIdx = 0;
        int targetCount = 0;

        foreach (RobotArms robotArm in robotArms)
        {
            if (robotArm == null || robotArm.robot == null || robotArm.targets == null)
            {
                Debug.LogError(
                    "Invalid robotArms configuration: each entry needs robot and targets assigned."
                );
                return;
            }

            robots[robotIdx] = robotArm.robot;
            robotTargetsList.Add(robotArm.targets);
            targetCount += robotArm.targets.Length;
            robotIdx += 1;
        }

        targets = new GameObject[targetCount];
        int targetIdx = 0;
        foreach (GameObject[] targetArray in robotTargetsList)
        {
            foreach (GameObject target in targetArray)
            {
                targets[targetIdx] = target;
                targetIdx += 1;
            }
        }

        sceneResetController = this.gameObject.GetComponent<SceneResetController>();
        if (sceneResetController == null)
            sceneResetController = this.gameObject.AddComponent<SceneResetController>();
        sceneResetController.Initialize(this);

        if (allRobotsActive)
            agentRobotController = this.gameObject.AddComponent<AgentRobotController>();
        if (allRobotsActive && !agenticSocket && !simulAgenticSocket)
        {
            agentRobotController.defaultJointAngleY = 60;
        }

        // int idx = 0;
        // foreach (GameObject robot in robots)
        // {
        //     RobotSetup(robot, idx);
        //     idx += 1;
        // }

        selectedRobotIndex = 0; // Default to first robot

        if (agenticSocket)
        {
            if (allRobotsActive)
                agenticSocketSetup = this.gameObject.AddComponent<AgenticSocketSetup>();
            else
            {
                agenticSocket = false;
            }
            simulAgenticSocket = false;
        }
        else if (ikSocket && !allRobotsActive)
        {
            agenticSocket = false;
            simulAgenticSocket = false;
            ikSocketSetup = this.gameObject.AddComponent<IKSocketSetup>();
        }
        else if (simulAgenticSocket)
        {
            if (allRobotsActive)
                simulAgenticSocketSetup = this.gameObject.AddComponent<SimulAgenticSocketSetup>();
            else
            {
                simulAgenticSocket = false;
            }
        }
        else
        {
            agenticSocket = false;
            simulAgenticSocket = false;
            ikSocket = false;
        }

        int idx = 0;
        foreach (GameObject robot in robots)
        {
            RobotSetup(robot, idx);
            idx += 1;
        }
    }

    void RobotSetup(GameObject robot, int idx)
    {
        Debug.Log($"RobotSetup: {robot.name}");

        GameObject[] targets = robotTargetsList[idx];
        foreach (GameObject target in targets)
        {
            Collider blockCollider = target.GetComponent<Collider>();
            Debug.Log(target.name + " Original Position: " + blockCollider.bounds.center);
        }

        SuctionController suctionController = robot.GetComponent<SuctionController>();
        if (suctionController != null)
            suctionController.targetBlocks = targets;

        UR5Controller ur5Controller = robot.GetComponent<UR5Controller>();

        UR5IKController ikController = robot.GetComponent<UR5IKController>();
    }

    public void ResetSceneState()
    {
        if (sceneResetController == null)
        {
            Debug.LogWarning("Scene reset skipped: SceneResetController is missing.");
            return;
        }

        sceneResetController.ResetSceneState();
    }

    void Update()
    {
        bool resetScene = Input.GetKeyDown(KeyCode.R);
        if (resetScene)
        {
            ResetSceneState();
        }

        if (allRobotsActive)
            return;

        bool selectRobot = Input.GetKeyDown(KeyCode.P);
        bool closeSocket = Input.GetKeyDown(KeyCode.X);

        if (agenticSocket && closeSocket)
        {
            //agenticSocketSetup.closeSocket();
            agenticSocketSetup.enabled = false;
            agenticSocket = false;
        }

        if (ikSocket && closeSocket)
        {
            ikSocketSetup.enabled = false;
            ikSocket = false;
        }
    }

    void OnGUI()
    {
        if (allRobotsActive)
            return;

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 16;
        style.normal.textColor = Color.white;

        string robotName = robots[selectedRobotIndex].name;
        GUI.Label(new Rect(Screen.width - 250, 10, 300, 30), "Selected Robot: " + robotName, style);
        GUI.Label(new Rect(Screen.width - 250, 35, 300, 20), "Press 'P' to switch", style);
    }
}
