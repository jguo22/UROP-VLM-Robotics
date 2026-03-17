using System;
using System.Collections;
//using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Unified Robot Controller - Provides easy access to all robot control methods
/// </summary>
public class UnifiedRobotController : MonoBehaviour
{
    [HideInInspector]
    public RobotArmSetup robotArmSetup;

    [HideInInspector]
    public UR5IKController ur5IkController;

    [HideInInspector]
    public PickAndPlaceController pickAndPlaceController;

    [HideInInspector]
    public ManualController manualController;

    [HideInInspector]
    public CSVTrajectoryController csvTrajectoryController;

    [HideInInspector]
    public SuctionController suctionController;

    [Header("Control Settings")]
    public ControlMode currentMode = ControlMode.Start;
    public float jointSpeed = JointSpeed; // degrees per second
    public float moveSpeed = 0.5f; // units per second for IK movement

    public enum ControlMode
    {
        Start, // initial state at start up
        Manual, // Keyboard joint control
        IK, // Inverse Kinematics end-effector control
        PickAndPlace, // Automated pick and place
        Programmatic, // Script-based control
        CSVTrajectory // CSV file trajectory playback
        ,
    }

    private ArticulationBody[] articulationChain;
    private ArticulationBody[] robotJoints;
    private Transform endEffector;

    /// <summary>
    /// Public access to robot joints for debugging
    /// </summary>
    public ArticulationBody[] Joints => robotJoints;

    void Start()
    {
        InitializeComponents();
    }

    void InitializeComponents()
    {
        // Find or assign robot components
        if (robotArmSetup == null)
            //robotArmSetup = FindObjectOfType<RobotArmSetup>();
            robotArmSetup = GetComponent<RobotArmSetup>();

        if (ur5IkController == null)
            //ur5IkController = FindObjectOfType<UR5IKController>();
            ur5IkController = GetComponent<UR5IKController>();

        if (pickAndPlaceController == null)
            //pickAndPlaceController = FindObjectOfType<PickAndPlaceController>();
            pickAndPlaceController = GetComponent<PickAndPlaceController>();

        if (manualController == null)
            //manualController = FindObjectOfType<ManualController>();
            manualController = GetComponent<ManualController>();

        if (csvTrajectoryController == null)
            //csvTrajectoryController = FindObjectOfType<CSVTrajectoryController>();
            csvTrajectoryController = GetComponent<CSVTrajectoryController>();

        if (suctionController == null)
            suctionController = GetComponent<SuctionController>();

        // Get joints from robotArmSetup
        if (robotArmSetup != null && robotArmSetup.articulationChain != null)
        {
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            for (int i = BaseIndex; i < BaseIndex + JointCount; i++)
            {
                ArticulationBody joint = articulationChain[i];
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = JointFriction;
                joint.angularDamping = AngularDamping;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = ForceLimit;
                joint.xDrive = currentDrive;
            }

            robotJoints = new ArticulationBody[JointCount + 1];
            Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);
            robotJoints[JointCount] = articulationChain[EndEffectorIndex];

            //joints = robotArmSetup.articulationChain;
            // Find end effector (usually the last joint)
            if (robotJoints.Length > 0)
            {
                endEffector = robotJoints[robotJoints.Length - 1].transform;
            }
            else
            {
                Debug.LogError("Unified Robot Controller - no joints found!");
            }
        }
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
            SetControlMode(ControlMode.Manual);
        if (Input.GetKeyDown(KeyCode.Alpha2))
            SetControlMode(ControlMode.IK);
        if (Input.GetKeyDown(KeyCode.Alpha3))
            SetControlMode(ControlMode.PickAndPlace);
        if (Input.GetKeyDown(KeyCode.Alpha4))
            SetControlMode(ControlMode.Programmatic);
        if (Input.GetKeyDown(KeyCode.Alpha5))
            SetControlMode(ControlMode.CSVTrajectory);

        // Suction controls
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (suctionController != null)
            {
                suctionController.ToggleSuction();
            }
        }
    }

    void HandleCurrentMode()
    {
        switch (currentMode)
        {
            case ControlMode.Manual:
                HandleManualControl();
                break;
            case ControlMode.IK:
                HandleIKControl();
                break;
            case ControlMode.PickAndPlace:
                // Pick and place handles its own input
                break;
            case ControlMode.Programmatic:
                HandleProgrammaticControl();
                break;
            case ControlMode.CSVTrajectory:
                HandleCSVTrajectoryControl();
                break;
        }
    }

    #region Manual Joint Control

    void HandleManualControl()
    {
        // Joint selection with arrow keys (handled by Controller script)
        // Movement with WASD (handled by Controller script)

        // Additional manual controls
        if (Input.GetKeyDown(KeyCode.R))
            ResetToHomePosition();

        if (Input.GetKeyDown(KeyCode.H))
            MoveToHomePosition();
    }

    #endregion

    #region Inverse Kinematics Control

    void HandleIKControl()
    {
        // Vector3 movement = Vector3.zero;

        // Move end effector with WASDQE
        // if (Input.GetKey(KeyCode.W)) movement += Vector3.forward * moveSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.S)) movement += Vector3.back * moveSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.A)) movement += Vector3.left * moveSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.D)) movement += Vector3.right * moveSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.Q)) movement += Vector3.up * moveSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.E)) movement += Vector3.down * moveSpeed * Time.deltaTime;

        // if (movement != Vector3.zero && endEffector != null)
        // {
        //     Vector3 targetPosition = endEffector.position + movement;
        //     MoveEndEffectorTo(targetPosition);
        // }

        // Reset with R
        if (Input.GetKeyDown(KeyCode.R))
            ResetToHomePosition();
    }

    #endregion

    #region Programmatic Control

    void HandleProgrammaticControl()
    {
        // Add programmatic control logic here
        // This could be waypoint following, trajectory planning, etc.
    }

    #endregion

    #region CSV Trajectory Control

    void HandleCSVTrajectoryControl()
    {
        // CSV trajectory control is handled by the CSVTrajectoryController component
        // Additional keyboard controls can be added here if needed

        // Additional controls specific to CSV trajectory mode
        if (Input.GetKeyDown(KeyCode.L))
        {
            if (csvTrajectoryController != null)
            {
                csvTrajectoryController.loopTrajectory = !csvTrajectoryController.loopTrajectory;
                Debug.Log($"CSV Trajectory loop: {csvTrajectoryController.loopTrajectory}");
            }
        }
    }

    #endregion

    #region Public Control Methods
    public float[] GetCurrentJointAngles(ArticulationBody[] robotJoints)
    {
        if (robotJoints == null)
            return null;

        float[] angles = new float[JointCount]; // UR5 has 6 joints
        for (int i = 0; i < angles.Length && i < robotJoints.Length; i++)
        {
            angles[i] = robotJoints[i].jointPosition[0] * Mathf.Rad2Deg;
        }
        return angles;
    }

    public void SetControlMode(ControlMode mode)
    {
        currentMode = mode;
        Debug.Log($"Switched to {mode} control mode");

        // Enable/disable appropriate components
        if (manualController != null)
            manualController.enabled = (mode == ControlMode.Manual);

        if (ur5IkController != null)
            ur5IkController.enabled = (mode == ControlMode.IK);

        if (pickAndPlaceController != null)
            pickAndPlaceController.enabled = (mode == ControlMode.PickAndPlace);

        // CSV Trajectory controller is always enabled but mode controls its behavior
        // The CSVTrajectoryController handles its own playback state
    }

    public void ResetToHomePosition()
    {
        if (robotArmSetup != null)
        {
            robotArmSetup.ResetArmPosition();
            Debug.Log("Robot reset to home position");
        }
    }

    public void MoveToHomePosition()
    {
        // Smooth move to home position
        StartCoroutine(MoveToPosition(StableStartingRotations));
    }

    public void MoveEndEffectorTo(Vector3 targetPosition)
    {
        if (!ur5IkController.ikActive)
        {
            ur5IkController.MoveToEndEffectorPose(targetPosition);
        }
    }

    #endregion

    #region CSV Trajectory Methods

    /// <summary>
    /// Load a CSV trajectory file
    /// </summary>
    public void LoadCSVTrajectory(string filePath)
    {
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.LoadTrajectoryFromPath(filePath);
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
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.LoadTrajectoryFromTextAsset(textAsset);
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
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.StartPlayback();
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
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.StopPlayback();
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
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.PausePlayback();
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
        if (csvTrajectoryController != null)
        {
            csvTrajectoryController.SetPlaybackSpeed(speed);
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
        return csvTrajectoryController != null && csvTrajectoryController.IsPlaying();
    }

    /// <summary>
    /// Get CSV trajectory progress (0-1)
    /// </summary>
    public float GetCSVProgress()
    {
        return csvTrajectoryController != null ? csvTrajectoryController.GetProgress() : 0f;
    }

    #endregion

    #region Public Control Methods

    public void SetJointAngles(float[] angles)
    {
        Debug.Log(
            $"UnifiedRobotController.SetJointAngles called with {angles?.Length ?? 0} angles"
        );

        if (robotArmSetup != null && angles != null)
        {
            Debug.Log($"RobotArmSetup found, joints array length: {robotJoints?.Length ?? 0}");

            for (int i = 0; i < Mathf.Min(angles.Length, StableStartingRotations.Length); i++)
            {
                if (i < robotJoints.Length)
                {
                    //Debug.Log($"Setting joint {i} to angle {angles[i] * Mathf.Rad2Deg:F2} degrees");
                    Debug.Log($"Setting joint {i} to angle {angles[i]:F2} degrees");

                    ArticulationDrive drive = robotJoints[i].xDrive;
                    drive.target = angles[i];
                    //drive.target = angles[i] * Mathf.Rad2Deg;

                    // Ensure proper drive settings for movement
                    drive.stiffness = 10000f;
                    drive.damping = 100f;
                    drive.forceLimit = 1000f;

                    robotJoints[i].xDrive = drive;

                    Debug.Log(
                        $"Joint {i} drive updated: target={drive.target:F2}, stiffness={drive.stiffness}"
                    );
                }
                else
                {
                    Debug.LogWarning(
                        $"Joint index {i} is out of bounds (joints array length: {robotJoints.Length})"
                    );
                }
            }
        }
        else
        {
            if (robotArmSetup == null)
                Debug.LogError("UnifiedRobotController: robotArmSetup is null!");
            if (angles == null)
                Debug.LogError("UnifiedRobotController: angles array is null!");
        }
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

    private IEnumerator MoveToPosition(float[] targetAngles)
    {
        float[] startAngles = GetCurrentJointAngles(robotJoints);
        float duration = 2f; // seconds
        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = elapsed / duration;
            t = Mathf.SmoothStep(0f, 1f, t); // Smooth interpolation

            float[] currentAngles = new float[targetAngles.Length];
            for (int i = 0; i < targetAngles.Length; i++)
            {
                currentAngles[i] = Mathf.Lerp(startAngles[i], targetAngles[i], t);
            }

            SetJointAngles(currentAngles);
            yield return null;
        }

        // Ensure final position
        SetJointAngles(targetAngles);
    }

    #endregion

    #region Suction Control Methods

    /// <summary>
    /// Toggle suction on/off
    /// </summary>
    public void ToggleSuction()
    {
        if (suctionController != null)
        {
            suctionController.ToggleSuction();
        }
    }

    /// <summary>
    /// Set suction state
    /// </summary>
    public void SetSuctionState(bool active)
    {
        if (suctionController != null)
        {
            suctionController.SetSuctionState(active);
        }
    }

    public void SetBlockAttractedState(bool attracted)
    {
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
    public bool IsBlockAttached()
    {
        return suctionController != null && suctionController.IsBlockAttached();
    }

    /// <summary>
    /// Get distance to target block
    /// </summary>
    public float GetDistanceToBlock()
    {
        return suctionController != null ? suctionController.GetDistanceToBlock() : float.MaxValue;
    }

    /// <summary>
    /// Check if block is within suction range
    /// </summary>
    public bool IsBlockInSuctionRange()
    {
        return suctionController != null && suctionController.IsWithinSuctionRange();
    }

    #endregion

    #region GUI

    void OnGUI()
    {
        // Display current control mode
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 16;
        style.normal.textColor = Color.white;

        string modeText =
            currentMode == ControlMode.Start
                ? "Control Mode: <Select>"
                : $"Control Mode: {currentMode}";
        string instructionText = "";

        switch (currentMode)
        {
            case ControlMode.Start:
                instructionText = "Select a control mode!";
                break;
            case ControlMode.Manual:
                instructionText = "Left/Right: Select joint | Up/Down: Move joint | R: Reset";
                break;
            case ControlMode.IK:
                instructionText = "Enter target xyz position | R: Reset";
                break;
            case ControlMode.PickAndPlace:
                instructionText = "S: Start pick and place operation";
                break;
            case ControlMode.Programmatic:
                instructionText = "Programmatic control active";
                break;
            case ControlMode.CSVTrajectory:
                instructionText = "Space: Play/Pause | S: Stop | +/-: Speed | L: Toggle loop";
                break;
        }

        GUI.Label(new Rect(10, 10, 400, 20), modeText, style);
        GUI.Label(new Rect(10, 35, 400, 20), instructionText, style);
        GUI.Label(new Rect(10, 60, 400, 20), "Press 1-5 to switch modes", style);
        GUI.Label(new Rect(10, 85, 400, 20), "Space: Toggle suction", style);

        // Display joint angles
        float[] angles = GetCurrentJointAngles(robotJoints);
        if (angles != null)
        {
            string angleText = "Joint Angles: ";
            for (int i = 0; i < angles.Length; i++)
            {
                angleText += $"{angles[i]:F1}° ";
            }
            GUI.Label(new Rect(10, 110, 400, 20), angleText, style);
        }

        // Display end effector coordinates
        if (endEffector != null)
        {
            Vector3 pos = endEffector.position;
            Quaternion rot = endEffector.rotation;
            string eePosText = $"End Effector Position: X={pos.x:F3} Y={pos.y:F3} Z={pos.z:F3}";
            string eeRotText = $"Rotation: X={rot.x:F3} Y={rot.y:F3} Z={rot.z:F3} W={rot.w:F3}";
            GUI.Label(new Rect(10, 135, 400, 20), eePosText, style);
            GUI.Label(new Rect(10, 160, 400, 20), eeRotText, style);
        }

        // Display suction information
        if (suctionController != null)
        {
            string suctionText = $"Suction: {(suctionController.enableSuction ? "ON" : "OFF")}";
            Color suctionColor = suctionController.enableSuction ? Color.green : Color.red;
            GUIStyle suctionStyle = new GUIStyle(style);
            suctionStyle.normal.textColor = suctionColor;

            GUI.Label(new Rect(10, 185, 400, 20), suctionText, suctionStyle);

            if (suctionController.enableSuction)
            {
                float distance = suctionController.GetDistanceToBlock();
                bool inRange = suctionController.IsWithinSuctionRange();
                bool attached = suctionController.IsBlockAttached();

                GUI.Label(
                    new Rect(10, 210, 400, 20),
                    $"Distance from nearest block: {distance:F3}m",
                    style
                );

                string statusText = attached ? "ATTACHED" : (inRange ? "IN RANGE" : "OUT OF RANGE");
                Color statusColor = attached ? Color.green : (inRange ? Color.yellow : Color.red);
                GUIStyle statusStyle = new GUIStyle(style);
                statusStyle.normal.textColor = statusColor;

                GUI.Label(new Rect(10, 235, 400, 20), $"Status: {statusText}", statusStyle);
            }
        }
    }

    #endregion
}
