using UnityEngine;
using System.Collections;
using System.IO;
using System.Text.RegularExpressions;

using static ConstantsUR5;

// helper script, doesn't have any update code
[DefaultExecutionOrder(100)]
[RequireComponent(typeof(RobotArmSetup))]
public class UR5IKController : MonoBehaviour
{
    [Header("IK Settings")]
    public float moveDuration = 0.3f; // Faster for continuous movement
    [HideInInspector]
    public Vector3 endEffectorTargetPosition;
    [HideInInspector]
    public bool ikActive = false;
    [HideInInspector]
    public bool hasIKSolution = false;
    [HideInInspector]
    public double[] ikSolution;

    //IK inputs
    private string targetPosInputString = "";
    private bool isWaitingForPosInput = false;
    private bool isChangingPos = false;

    // Component references
    private RobotArmSetup robotArmSetup;
    private SuctionController suctionController;
    // private UR5IKSolver ikSolver;

    // Robot references
    private ArticulationBody[] articulationChain;
    private ArticulationBody[] robotJoints; // 6 joints + end effector (7 total)
    private Transform endEffector;
    private Transform originTransform; // used to calculate the relative position of the end effector to the base of the robot

    private Coroutine moveCoroutine;
    private bool isMoving = false;

    void Start()
    {
        // Get RobotArmSetup component
        robotArmSetup = GetComponent<RobotArmSetup>();
        if (robotArmSetup == null)
        {
            Debug.LogError("UR5Controller: RobotArmSetup component not found!");
            enabled = false;
            return;
        }

        // Get articulation chain from RobotArmSetup
        articulationChain = robotArmSetup.articulationChain;
        robotJoints = robotArmSetup.robotJoints; // This is already set up by RobotArmSetup (6 joints + end effector)
        foreach (ArticulationBody joint in robotJoints)
            print(joint);

        Debug.Log($"UR5Controller: Found {articulationChain.Length} articulation bodies");
        Debug.Log($"UR5Controller: Robot joints array has {robotJoints.Length} elements");

        // End effector is the last element in robotJoints array
        endEffector = robotJoints[JointCount].transform; // robotJoints[6] is the end effector
        originTransform = this.transform;

        // Get suction controller (optional)
        suctionController = GetComponent<SuctionController>();

        // Get IK solver
        // ikSolver = GetComponent<UR5IKSolver>();
        // if (ikSolver == null)
        // {
        //     Debug.LogWarning("UR5Controller: UR5IKSolver component not found. Adding dummy solver.");
        //     ikSolver = gameObject.AddComponent<UR5IKSolver>();
        // }

        // Configure joint drives
        ConfigureJointDrives();

        Debug.Log("UR5Controller initialized successfully");
    }

    void ConfigureJointDrives()
    {
        // Configure drives for the 6 robot arm joints (indices 0-5 in robotJoints)
        for (int i = 0; i < JointCount; i++)
        {
            if (robotJoints[i] == null) continue;

            ArticulationDrive drive = robotJoints[i].xDrive;
            drive.stiffness = JointStiffness;
            drive.damping = JointDamping;
            drive.forceLimit = ForceLimit;
            robotJoints[i].xDrive = drive;
        }
    }

    /// <summary>
    /// Set joint angles directly (in radians)
    /// </summary>
    public void SetJointAngles(float[] angles)
    {
        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (robotJoints[i] != null)
            {
                ArticulationDrive drive = robotJoints[i].xDrive;
                drive.target = angles[i] * Mathf.Rad2Deg;
                robotJoints[i].xDrive = drive;
                // Debug.Log($"Joint {i}: Setting target to {drive.target} degrees ({angles[i]} radians)");
            }
        }
    }

    void Update()
    {
        if (!isChangingPos)
        {
            bool targetPosInput = Input.GetKeyDown(KeyCode.T);
            if (targetPosInput)
            {
                isWaitingForPosInput = true;
                targetPosInputString = "";
            }
            if (isWaitingForPosInput)
            {
                HandlePosInput();
            }
        }
    }

    private (Vector3 position, Quaternion rotation) ConvertToRobotCoordinates(Vector3 inputPosition, Quaternion inputRotation)
    {
        // Debug.Log(inputPosition);
        // Debug.Log(inputRotation);
        Vector3 position = originTransform.InverseTransformPoint(inputPosition);
        Quaternion rotation = Quaternion.Inverse(originTransform.rotation) * inputRotation;
        return (position, rotation);
    }

    /// <summary>
    /// Control gripper/suction
    /// </summary>
    public void SetGripper(bool close)
    {
        if (suctionController != null)
        {
            suctionController.enableSuction = close;
        }
        else
        {
            Debug.LogWarning("UR5Controller: No SuctionController found");
        }
    }

    /// <summary>
    /// Get current end effector pose
    /// </summary>
    public (Vector3 position, Quaternion rotation) GetEndEffectorPose()
    {
        if (endEffector != null)
        {
            return ConvertToRobotCoordinates(endEffector.position, endEffector.rotation);
        }
        else
        {
            return (Vector3.zero, Quaternion.identity);
        }
    }

    /// <summary>
    /// Move end effector to target position
    /// </summary>
    public void MoveToEndEffectorPose(Vector3 targetInputPosition) // Quaternion targetRotation
    {
        // Convert target to robot coordinates
        //(Vector3 localPosition, Quaternion localRotation) = ConvertToRobotCoordinates(targetPosition, targetRotation);
        Debug.Log("Moving to target position: " + targetInputPosition.ToString("F4"));
        targetInputPosition.y += SuctionIKOffsetY; // slight offset to account for suction cup height
        endEffectorTargetPosition = originTransform.InverseTransformPoint(targetInputPosition);
        ikActive = true;
        // StartCoroutine(RequestAndMoveToIKSolution());

        // double[] jointAngles = {0, 0, 0, 0, 0, 0};
        // when script closes get result joint angles and move them
        // SetJointAngles(jointAngles);

    }

    /// <summary>
    /// Request IK solution and move arm once solution is received
    /// </summary>
    private IEnumerator RequestAndMoveToIKSolution()
    {
        // Signal server to send IK request
        ikActive = true;
        hasIKSolution = false;

        Debug.Log($"Requesting IK solution for target: {endEffectorTargetPosition}");

        // Wait for IK solution (with timeout)
        float timeout = 2f;
        float elapsed = 0f;

        while (!hasIKSolution && elapsed < timeout)
        {
            elapsed += Time.deltaTime;
            yield return null;
        }

        if (hasIKSolution && ikSolution != null && ikSolution.Length == JointCount)
        {
            // Convert solution to float array and move
            float[] solutionAngles = new float[JointCount];
            for (int i = 0; i < JointCount; i++)
                solutionAngles[i] = (float)ikSolution[i];

            Debug.Log($"IK solution received: {string.Join(", ", solutionAngles)}");
            SetJointAngles(solutionAngles);
        }
        else
        {
            Debug.LogError("IK solution not received within timeout or invalid format");
        }

        hasIKSolution = false;
        ikActive = false;
    }

    /// <summary>
    /// Get current joint angles in radians
    /// </summary>
    public float[] GetJointAngles()
    {
        float[] angles = new float[6]; // Assuming 6-DOF UR5 robot

        // Get the current joint angles from the joint controllers
        for (int i = 0; i < 6; i++)
        {
            if (i < robotJoints.Length)
            {
                // Get the angle from the joint controller
                angles[i] = robotJoints[i].jointPosition[0]; // Convert to radians
            }
        }

        return angles;
    }

    public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;
        // GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press left/right arrow keys to select a robot joint.", centeredStyle);
        // GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press up/down keys to move " + selectedJoint + ".", centeredStyle);

        // Display instructions for angle input
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press 'T' to set target pos for selected joint.", centeredStyle);

        if (isWaitingForPosInput)
        {
            GUI.Label(new Rect(Screen.width / 2 - 200, 40, 400, 20), "Enter target xyz position (m) as 'x, y, z' :", centeredStyle);
        
            // Create input field
            GUI.SetNextControlName("PosInputField");
            //GUI.TextField(new Rect(Screen.width / 2 - 100, 90, 200, 20), targetAngleInputString);
            targetPosInputString = GUI.TextField(new Rect(Screen.width / 2 - 100, 70, 200, 20), targetPosInputString);
            targetPosInputString = Regex.Replace(targetPosInputString, "[^0-9 .,-]", "");
            
            // Instructions
            GUI.Label(new Rect(Screen.width / 2 - 200, 100, 400, 20), "Press Enter to confirm, 'E' to exit", centeredStyle);
            
            // Show current input
            if (!string.IsNullOrEmpty(targetPosInputString))
            {
                GUI.Label(new Rect(Screen.width / 2 - 200, 130, 400, 20), $"Input: {targetPosInputString}", centeredStyle);
            }
            
            // Focus the input field
            GUI.FocusControl("PosInputField");
        }

    }

    private void HandlePosInput()
    {
        if (!isWaitingForPosInput) return;

        //Handle character input
        if (Input.inputString.Length > 0)
        {
            //Debug.Log("Input.inputString: " + Input.inputString);
            foreach (char c in Input.inputString)
            {
                if (c == '\b') // Backspace
                {
                    if (targetPosInputString.Length > 0)
                    {
                        targetPosInputString = targetPosInputString.Substring(0, targetPosInputString.Length);
                    }
                }
                else if (c == '\n' || c == '\r') // Enter
                {
                    ProcessPosInput();
                    return;
                }
                else if (c == 'e' || c == 'E')
                {
                    isWaitingForPosInput = false;
                    //return;
                }
            }
        }
    }

    private void ProcessPosInput()
    {
        // Parse input string into Vector3
        string[] parts = targetPosInputString.Split(new char[] { ',', ' ' }, System.StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length == 3)
        {
            if (float.TryParse(parts[0], out float x) &&
                float.TryParse(parts[1], out float y) &&
                float.TryParse(parts[2], out float z))
            {
                Vector3 targetPosition = new Vector3(x, y, z);
                isWaitingForPosInput = false;
                isChangingPos = true;
                MoveToEndEffectorPose(targetPosition);
                isChangingPos = false;
            }
        } else // gear colour input
        {
            Debug.LogError("Invalid position input. Please enter in format: x,y,z");
        }
    }
    
}
