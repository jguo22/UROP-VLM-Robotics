using UnityEngine;
using System.Collections;

// helper script, doesn't have any update code
[DefaultExecutionOrder(100)]
[RequireComponent(typeof(RobotArmSetup))]
public class UR5Controller : MonoBehaviour
{
    [Header("IK Settings")]
    public float moveDuration = 0.3f;
    public float stiffness = 10000f;
    public float damping = 100f;

    // Component references
    private RobotArmSetup robotArmSetup;
    private SuctionController suctionController;
    private UR5IKSolver ikSolver;

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

        Debug.Log($"UR5Controller: Found {articulationChain.Length} articulation bodies");
        Debug.Log($"UR5Controller: Robot joints array has {robotJoints.Length} elements");

        // End effector is the last element in robotJoints array
        endEffector = robotJoints[6].transform; // robotJoints[6] is the end effector
        originTransform = this.transform;

        // Get suction controller (optional)
        suctionController = GetComponent<SuctionController>();

        // Get IK solver
        ikSolver = GetComponent<UR5IKSolver>();
        if (ikSolver == null)
        {
            Debug.LogWarning("UR5Controller: UR5IKSolver component not found. Adding dummy solver.");
            ikSolver = gameObject.AddComponent<UR5IKSolver>();
        }

        // Configure joint drives
        ConfigureJointDrives();

        Debug.Log("UR5Controller initialized successfully");
    }

    void ConfigureJointDrives()
    {
        // Configure drives for the 6 robot arm joints (indices 0-5 in robotJoints)
        for (int i = 0; i < 6; i++)
        {
            if (robotJoints[i] == null) continue;

            ArticulationDrive drive = robotJoints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = float.MaxValue;
            robotJoints[i].xDrive = drive;
        }
    }

    /// <summary>
    /// Move to target position and rotation using IK
    /// </summary>
    void MoveToTarget(Vector3 targetPosition, Quaternion targetRotation)
    {
        // Don't start a new movement if one is already in progress
        if (isMoving)
            return;

        (Vector3 relativePosition, Quaternion relativeRotation) = ConvertToRobotCoordinates(targetPosition, targetRotation);
        float[] currentAngles = GetJointAngles();
        float[] targetAngles = ikSolver.SolveIK(relativePosition, relativeRotation, currentAngles);

        Debug.Log($"UR5Controller: Current angles: [{string.Join(", ", currentAngles)}]");
        Debug.Log($"UR5Controller: Target angles: [{string.Join(", ", targetAngles ?? new float[0])}]");

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            isMoving = true;
            moveCoroutine = StartCoroutine(MoveToAnglesCoroutine(targetAngles));
        }
        else
        {
            Debug.LogWarning("UR5Controller: No IK solution found for target position");
        }
    }


    void MoveToAngles(float[] targetAngles)
    {
        if (moveCoroutine != null)
        {
            StopCoroutine(moveCoroutine);
        }
        moveCoroutine = StartCoroutine(MoveToAnglesCoroutine(targetAngles));
    }

    /// <summary>
    /// Set joint angles directly (in radians)
    /// </summary>
    void SetJointAngles(float[] angles)
    {
        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (robotJoints[i] != null)
            {
                ArticulationDrive drive = robotJoints[i].xDrive;
                drive.target = angles[i] * Mathf.Rad2Deg;
                robotJoints[i].xDrive = drive;
            }
        }
    }

    IEnumerator MoveToAnglesCoroutine(float[] targetAngles)
    {
        float[] startAngles = GetJointAngles();
        float elapsed = 0f;

        while (elapsed < moveDuration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(elapsed / moveDuration);
            t = t * t * (3f - 2f * t); // Smoothstep

            float[] interpolated = new float[6];
            for (int i = 0; i < 6; i++)
            {
                interpolated[i] = Mathf.LerpAngle(startAngles[i] * Mathf.Rad2Deg,
                                                   targetAngles[i] * Mathf.Rad2Deg, t) * Mathf.Deg2Rad;
            }

            SetJointAngles(interpolated);
            yield return null;
        }

        SetJointAngles(targetAngles);
        isMoving = false;
    }

    (Vector3 position, Quaternion rotation) ConvertToRobotCoordinates(Vector3 inputPosition, Quaternion inputRotation)
    {
        Vector3 position = originTransform.InverseTransformPoint(inputPosition);
        Quaternion rotation = Quaternion.Inverse(originTransform.rotation) * inputRotation;
        return (position, rotation);
    }
    /// <summary>
    /// Move by a delta position and rotation
    /// </summary>
    public void MoveDelta(Vector3 deltaPosition, Quaternion deltaRotation)
    {
        if (isMoving)
            return;

        Vector3 targetPos = endEffector.position + deltaPosition;
        Quaternion targetRot = endEffector.rotation * deltaRotation;

        Debug.Log($"MoveDelta - Current Pos: ({endEffector.position.x:F6}, {endEffector.position.y:F6}, {endEffector.position.z:F6}), Current Rot: ({endEffector.rotation.x:F6}, {endEffector.rotation.y:F6}, {endEffector.rotation.z:F6}, {endEffector.rotation.w:F6}) | Target Pos: ({targetPos.x:F6}, {targetPos.y:F6}, {targetPos.z:F6}), Target Rot: ({targetRot.x:F6}, {targetRot.y:F6}, {targetRot.z:F6}, {targetRot.w:F6})");

        print(endEffector.rotation);
        print(targetRot);
        MoveToTarget(targetPos, targetRot);
        print(endEffector.rotation);
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
                angles[i] = robotJoints[i].jointPosition[0];
            }
        }

        return angles;
    }

    void OnGUI()
    {
        if (endEffector == null) return;

        string displayText = $"End Effector Position: ({endEffector.position.x:F5}, {endEffector.position.y:F5}, {endEffector.position.z:F5})\n";
        displayText += $"End Effector Rotation: ({endEffector.rotation.x:F5}, {endEffector.rotation.y:F5}, {endEffector.rotation.z:F5}, {endEffector.rotation.w:F5})\n";
        displayText += $"Euler Angles: ({endEffector.rotation.eulerAngles.x:F2}, {endEffector.rotation.eulerAngles.y:F2}, {endEffector.rotation.eulerAngles.z:F2})";

        float width = 650;
        float height = 80;
        Rect rect = new Rect(Screen.width / 2 - width / 2, 10, width, height);

        // Draw black background
        Texture2D blackTexture = new Texture2D(1, 1);
        blackTexture.SetPixel(0, 0, Color.black);
        blackTexture.Apply();
        GUI.DrawTexture(rect, blackTexture);

        // Draw white text
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.normal.textColor = Color.white;
        style.alignment = TextAnchor.UpperCenter;
        GUI.Label(rect, displayText, style);
    }
}
