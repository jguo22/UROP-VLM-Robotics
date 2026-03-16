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
    public UnifiedRobotController unifiedRobotController;

    //[HideInInspector]
    //public IKController ikController;
    [HideInInspector]
    public SuctionController suctionController;

    [HideInInspector]
    public CSVTrajectoryController csvTrajectoryController;

    [HideInInspector]
    public CSVTrajectoryExample csvTrajectoryExample;
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
            agentTrajectoryController = this.gameObject.AddComponent<AgentTrajectoryController>();
            agentTrajectory = this.gameObject.AddComponent<AgentTrajectory>();
            agentTrajectoryController.robots = robots;
            agentTrajectory.robots = robots;
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
                agentTrajectoryController =
                    this.gameObject.AddComponent<AgentTrajectoryController>();
                agentTrajectory = this.gameObject.AddComponent<AgentTrajectory>();
                agentTrajectoryController.robots = robots;
                agentTrajectory.robots = robots;
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
                agentTrajectoryController =
                    this.gameObject.AddComponent<AgentTrajectoryController>();
                agentTrajectory = this.gameObject.AddComponent<AgentTrajectory>();
                agentTrajectoryController.robots = robots;
                agentTrajectory.robots = robots;
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

        if (!allRobotsActive)
        {
            ToggleRobotOff(robots[(selectedRobotIndex + 1) % robots.Length]);
            ToggleRobotOn(robots[selectedRobotIndex]);
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

        ManualController manualController = robot.GetComponent<ManualController>();
        UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
        CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
        CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();

        UR5IKController ikController = robot.GetComponent<UR5IKController>();
        // UR5IKServer ikServer = robot.GetComponent<UR5IKServer>();

        // if (simulAgenticSocket)
        // {
        //     SimulAgentController simulAgentController = robot.GetComponent<SimulAgentController>();
        //     if (simulAgentController != null)
        //     {
        //         simulAgentController.simulAgenticSocketSetup = this.gameObject.GetComponent<SimulAgenticSocketSetup>();
        //     }

        // }

        if (allRobotsActive)
        {
            if (manualController != null)
                manualController.enabled = false;
            if (unifiedController != null)
                unifiedController.enabled = false;
            if (csvController != null)
                csvController.enabled = false;
            if (csvExample != null)
                csvExample.enabled = false;
            if (ikController != null)
                ikController.enabled = false;
            // if (ikServer != null) ikServer.enabled = false;
        }
        else
        {
            if (csvController != null)
                csvController.enabled = true;
            if (csvExample != null)
                csvExample.enabled = true;

            // if (ikController != null) ikController.enabled = true;
            // if (ikServer != null)
            // {
            //     // ikServer.port = robotArms[idx].ikPort;
            //     // ikServer.enabled = true;
            // }
        }
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

        if (selectRobot)
        {
            selectedRobotIndex = (selectedRobotIndex + 1) % robots.Length;
            ToggleRobotOff(robots[(selectedRobotIndex + 1) % robots.Length]);
            ToggleRobotOn(robots[selectedRobotIndex]);
        }

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

    void ToggleRobotOff(GameObject robot)
    {
        if (robot != null)
        {
            //disable all controllers in robot
            ManualController manualController = robot.GetComponent<ManualController>();
            if (manualController != null)
                manualController.enabled = false;

            UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
            if (unifiedController != null)
            {
                unifiedController.currentMode = UnifiedRobotController.ControlMode.Start;
                unifiedController.enabled = false;
            }

            SuctionController suctionController = robot.GetComponent<SuctionController>();
            if (suctionController != null)
                suctionController.enabled = false;

            CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
            if (csvController != null)
                csvController.enabled = false;

            CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();
            if (csvExample != null)
                csvExample.enabled = false;

            UR5IKController ikController = robot.GetComponent<UR5IKController>();
            if (ikController != null)
                ikController.enabled = false;

            // UR5IKServer ikServer = robot.GetComponent<UR5IKServer>();
            // if (ikServer != null) ikServer.enabled = false;
        }
    }

    void ToggleRobotOn(GameObject robot)
    {
        if (robot != null)
        {
            //enable all controllers in robot
            UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
            if (unifiedController != null)
                unifiedController.enabled = true;

            SuctionController suctionController = robot.GetComponent<SuctionController>();
            if (suctionController != null)
                suctionController.enabled = true;

            CSVTrajectoryController csvController = robot.GetComponent<CSVTrajectoryController>();
            if (csvController != null)
                csvController.enabled = true;

            CSVTrajectoryExample csvExample = robot.GetComponent<CSVTrajectoryExample>();
            if (csvExample != null)
                csvExample.enabled = true;

            // UR5IKController ikController = robot.GetComponent<UR5IKController>();
            // if (ikController != null) ikController.enabled = true;

            // UR5IKServer ikServer = robot.GetComponent<UR5IKServer>();
            // if (ikServer != null) ikServer.enabled = true;
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
