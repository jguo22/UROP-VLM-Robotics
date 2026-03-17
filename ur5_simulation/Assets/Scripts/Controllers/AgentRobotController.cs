using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Agent Robot Controller - Provides easy access to all robot control methods through agentic AI
/// </summary>
public class AgentRobotController : MonoBehaviour
{
    [HideInInspector]
    public AgentTrajectoryController agentTrajectoryController;

    [Header("Control Settings")]
    public ControlMode currentMode = ControlMode.Start; //ControlMode.Start;
    public float jointSpeed = JointSpeed; // degrees per second

    public enum ControlMode
    {
        Start, // initial state at start up
        CSVTrajectory, // CSV file trajectory playback
        AgenticAI // Agentic AI control mode
        ,
    }

    //private ArticulationBody[] articulationChain;
    private List<ArticulationBody[]> robotJointsList = new List<ArticulationBody[]>();
    private Transform endEffector;

    [HideInInspector]
    public GameObject[] robots;

    [HideInInspector]
    public int defaultJointAngleY = 35;

    void Start()
    {
        InitializeComponents();
    }

    void InitializeComponents()
    {
        SceneSetup sceneSetup = this.gameObject.GetComponent<SceneSetup>();
        if (sceneSetup == null)
        {
            Debug.LogError("AgentRobotController: SceneSetup component not found!");
            return;
        }

        robots = sceneSetup.robots;
        foreach (GameObject robot in robots)
        {
            RobotArmSetup robotArmSetup = robot.GetComponent<RobotArmSetup>();
            if (robotArmSetup != null)
                robotJointsList.Add(robotArmSetup.robotJoints);
        }

        agentTrajectoryController = GetComponent<AgentTrajectoryController>();
        Debug.Log(
            $"{robotJointsList.Count}, {robotJointsList[0].Length}, {robotJointsList[1].Length}"
        );

        // Get joints from robotArmSetup
        // if (robotArmSetup != null && robotArmSetup.articulationChain != null)
        // {
        //     articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        //     for (int i = BaseIndex; i < BaseIndex + JointCount; i++)
        //     {
        //         ArticulationBody joint = articulationChain[i];
        //         joint.gameObject.AddComponent<JointControl>();
        //         joint.jointFriction = JointFriction;
        //         joint.angularDamping = AngularDamping;
        //         ArticulationDrive currentDrive = joint.xDrive;
        //         currentDrive.forceLimit = ForceLimit;
        //         joint.xDrive = currentDrive;
        //     }

        //     robotJoints = new ArticulationBody[JointCount + 1];
        //     Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);
        //     robotJoints[JointCount] = articulationChain[EndEffectorIndex];

        //     //joints = robotArmSetup.articulationChain;
        //     // Find end effector (usually the last joint)
        //     if (robotJoints.Length > 0)
        //     {
        //         endEffector = robotJoints[robotJoints.Length - 1].transform;
        //     }
        //     else
        //     {
        //         Debug.LogError("Unified Robot Controller - no joints found!");
        //     }
        // }
    }

    void Update()
    {
        HandleModeSwitching();
        HandleCurrentMode();
    }

    void HandleModeSwitching()
    {
        // Switch modes with number keys
        if (Input.GetKeyDown(KeyCode.Alpha1))
            SetControlMode(ControlMode.CSVTrajectory);
        if (Input.GetKeyDown(KeyCode.Alpha2))
            SetControlMode(ControlMode.AgenticAI);
    }

    void HandleCurrentMode()
    {
        switch (currentMode)
        {
            case ControlMode.CSVTrajectory:
                HandleCSVTrajectoryControl();
                break;
            case ControlMode.AgenticAI:
                HandleAgenticAIControl();
                break;
        }
    }

    #region CSV Trajectory Control

    void HandleCSVTrajectoryControl()
    {
        // CSV trajectory control is handled by the CSVTrajectoryController component
        // Additional keyboard controls can be added here if needed

        // Additional controls specific to CSV trajectory mode
        if (Input.GetKeyDown(KeyCode.L))
        {
            if (agentTrajectoryController != null)
            {
                agentTrajectoryController.loopTrajectory =
                    !agentTrajectoryController.loopTrajectory;
                Debug.Log($"CSV Trajectory loop: {agentTrajectoryController.loopTrajectory}");
            }
        }
    }

    #endregion

    #region Agentic AI Control

    void HandleAgenticAIControl()
    {
        // Placeholder for Agentic AI control logic
        // This could involve receiving commands from an AI system and executing them
    }

    #endregion

    #region Public Control Methods
    public float[] GetJointAngles(GameObject robot)
    {
        RobotArmSetup robotArmSetup = robot.GetComponent<RobotArmSetup>();
        if (robotArmSetup == null)
            return null;

        ArticulationBody[] robotJoints = robotArmSetup.robotJoints;
        float[] angles = new float[JointCount]; // Assuming 6-DOF UR5 robot

        // Get the current joint angles from the joint controllers
        for (int i = 0; i < JointCount; i++)
        {
            if (i < robotJoints.Length)
            {
                // Get the angle from the joint controller
                angles[i] = robotJoints[i].jointPosition[0]; // Convert to radians
            }
        }

        return angles;
    }

    public void SetControlMode(ControlMode mode)
    {
        currentMode = mode;
        Debug.Log($"Switched to {mode} control mode");

        // CSV Trajectory controller is always enabled but mode controls its behavior
        // The CSVTrajectoryController handles its own playback state
    }

    public bool ResetToHomePosition(GameObject robot)
    {
        RobotArmSetup robotArmSetup = robot.GetComponent<RobotArmSetup>();
        if (robotArmSetup != null)
        {
            robotArmSetup.ResetArmPosition();
            Debug.Log("Robot reset to home position");
            return true;
        }
        return false;
    }

    // public void MoveToHomePosition()
    // {
    //     // Smooth move to home position
    //     StartCoroutine(MoveToPosition(StableStartingRotations));
    // }

    public void MoveEndEffectorTo(Vector3 targetPosition) { }

    #endregion

    #region CSV Trajectory Methods

    /// <summary>
    /// Load a CSV trajectory file
    /// </summary>
    public void LoadCSVTrajectory(string[] filePaths)
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.LoadTrajectoryFromPath(filePaths);
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Load a CSV trajectory from TextAsset
    /// </summary>
    public void LoadCSVTrajectory(TextAsset textAsset)
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.LoadTrajectoryFromTextAsset(textAsset);
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Start CSV trajectory playback
    /// </summary>
    public void StartCSVPlayback()
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.StartPlayback();
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Stop CSV trajectory playback
    /// </summary>
    public void StopCSVPlayback()
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.StopPlayback();
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Pause/Resume CSV trajectory playback
    /// </summary>
    public void PauseCSVPlayback()
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.PausePlayback();
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Set CSV playback speed
    /// </summary>
    public void SetCSVPlaybackSpeed(float speed)
    {
        if (agentTrajectoryController != null)
        {
            agentTrajectoryController.SetPlaybackSpeed(speed);
        }
        else
        {
            Debug.LogError("CSVTrajectoryController not found!");
        }
    }

    /// <summary>
    /// Check if CSV trajectory is currently playing
    /// </summary>
    public bool IsCSVPlaying()
    {
        return agentTrajectoryController != null && agentTrajectoryController.IsPlaying();
    }

    /// <summary>
    /// Get CSV trajectory progress (0-1)
    /// </summary>
    public float GetCSVProgress()
    {
        return agentTrajectoryController != null ? agentTrajectoryController.GetProgress() : 0f;
    }

    #endregion

    #region Public Control Methods

    public bool SetJointAngles(GameObject robot, float[] angles)
    {
        RobotArmSetup robotArmSetup = robot.GetComponent<RobotArmSetup>();
        if (robotArmSetup == null || angles == null)
            return false;

        ArticulationBody[] robotJoints = robotArmSetup.robotJoints;
        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (robotJoints[i] != null)
            {
                ArticulationDrive drive = robotJoints[i].xDrive;
                drive.target = angles[i] * Mathf.Rad2Deg;

                drive.stiffness = JointStiffness;
                drive.damping = JointDamping;
                drive.forceLimit = ForceLimit;

                robotJoints[i].xDrive = drive;
                // Debug.Log($"Joint {i}: Setting target to {drive.target} degrees ({angles[i]} radians)");
            }
        }
        return true;
    }

    public Vector3 GetEndEffectorPosition()
    {
        return endEffector != null ? endEffector.position : Vector3.zero;
    }

    public Quaternion GetEndEffectorRotation()
    {
        return endEffector != null ? endEffector.rotation : Quaternion.identity;
    }

    #endregion

    #region Coroutines

    // private IEnumerator MoveToPosition(float[] targetAngles)
    // {
    //     float[] startAngles = GetCurrentJointAngles(robotJoints);
    //     float duration = 2f; // seconds
    //     float elapsed = 0f;

    //     while (elapsed < duration)
    //     {
    //         elapsed += Time.deltaTime;
    //         float t = elapsed / duration;
    //         t = Mathf.SmoothStep(0f, 1f, t); // Smooth interpolation

    //         float[] currentAngles = new float[targetAngles.Length];
    //         for (int i = 0; i < targetAngles.Length; i++)
    //         {
    //             currentAngles[i] = Mathf.Lerp(startAngles[i], targetAngles[i], t);
    //         }

    //         SetJointAngles(currentAngles);
    //         yield return null;
    //     }

    //     // Ensure final position
    //     SetJointAngles(targetAngles);
    // }

    #endregion

    #region Suction Control Methods

    /// <summary>
    /// Set suction state
    /// </summary>
    public void SetSuctionState(GameObject robot, bool active)
    {
        SuctionController suctionController = robot.GetComponent<SuctionController>();
        if (suctionController != null)
        {
            suctionController.SetSuctionState(active);
        }
    }

    public void SetBlockAttractedState(GameObject robot, bool attracted)
    {
        SuctionController suctionController = robot.GetComponent<SuctionController>();
        if (suctionController != null)
        {
            if (attracted && !suctionController.isBlockAttached)
            {
                suctionController.suctionDistance = 0.05f; // Set to a larger distance to simulate attachment
            }
        }
    }

    /// <summary>
    /// Check if block is currently attached to suction
    /// </summary>
    // public bool IsBlockAttached()
    // {
    //     return suctionController != null && suctionController.IsBlockAttached();
    // }

    /// <summary>
    /// Get distance to target block
    /// </summary>
    // public float GetDistanceToBlock()
    // {
    //     return suctionController != null ? suctionController.GetDistanceToBlock() : float.MaxValue;
    // }

    /// <summary>
    /// Check if block is within suction range
    /// </summary>
    // public bool IsBlockInSuctionRange()
    // {
    //     return suctionController != null && suctionController.IsWithinSuctionRange();
    // }

    #endregion

    #region GUI

    void OnGUI()
    {
        // Display current control mode
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 16;
        style.normal.textColor = Color.white;

        //string modeText = currentMode == ControlMode.Start ? "Control Mode: <Select>" : $"Control Mode: {currentMode}";
        string modeText = "Control Mode: Agentic AI";
        // string instructionText = "Load trajectory [F1] | Play trajectory [Space]";

        // switch (currentMode)
        // {
        //     case ControlMode.Start:
        //         instructionText = "Select a control mode!";
        //         break;
        //     case ControlMode.CSVTrajectory:
        //         instructionText = "Space: Play/Pause | S: Stop | +/-: Speed | L: Toggle loop";
        //         break;
        //     case ControlMode.AgenticAI:
        //         instructionText = "Space: Play/Pause | S: Stop | +/-: Speed | L: Toggle loop";
        //         break;
        // }

        GUI.Label(new Rect(10, 10, 400, 30), modeText, style);
        // GUI.Label(new Rect(10, 35, 400, 30), instructionText, style);
        //GUI.Label(new Rect(10, 60, 400, 20), "Press 1-2 to switch modes", style);
        //GUI.Label(new Rect(10, 85, 400, 20), "Space: Toggle suction", style);

        int jointAngle_y = defaultJointAngleY;

        for (int robotIdx = 0; robotIdx < robots.Length; robotIdx++)
        {
            GameObject robot = robots[robotIdx];
            GUI.Label(new Rect(10, jointAngle_y, 100, 30), robot.name + ": ", style);
            // float[] angles = GetCurrentJointAngles(robotJointsList[robotIdx]);
            // if (angles != null)
            // {
            //     string angleText = robot.name + " Joint Angles: ";
            //     for (int i = 0; i < angles.Length; i++)
            //     {
            //         angleText += $"{angles[i]:F1}° ";
            //     }
            //     GUI.Label(new Rect(10, jointAngle_y, 500, 30), angleText, style);
            // }

            SuctionController suctionController = robot.GetComponent<SuctionController>();
            // Display suction information
            if (suctionController != null)
            {
                string suctionText = $"Suction: {(suctionController.enableSuction ? "ON" : "OFF")}";
                Color suctionColor = suctionController.enableSuction ? Color.green : Color.red;
                GUIStyle suctionStyle = new GUIStyle(style);
                suctionStyle.normal.textColor = suctionColor;

                GUI.Label(new Rect(100, jointAngle_y, 100, 20), suctionText, suctionStyle); //y=135

                if (suctionController.enableSuction)
                {
                    float distance = suctionController.GetDistanceToBlock();
                    bool inRange = suctionController.IsWithinSuctionRange();
                    bool attached = suctionController.IsBlockAttached();

                    //GUI.Label(new Rect(10, 160, 400, 20), $"Distance from nearest block: {distance:F3}m", style);

                    string statusText = attached
                        ? "ATTACHED"
                        : (inRange ? "IN RANGE" : "OUT OF RANGE");
                    Color statusColor = attached
                        ? Color.green
                        : (inRange ? Color.yellow : Color.red);
                    GUIStyle statusStyle = new GUIStyle(style);
                    statusStyle.normal.textColor = statusColor;

                    GUI.Label(
                        new Rect(210, jointAngle_y, 400, 20),
                        $"Status: {statusText}",
                        statusStyle
                    ); //y=185
                }
            }

            jointAngle_y += 25;
        }
    }

    #endregion
}
