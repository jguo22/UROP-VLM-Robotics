using UnityEngine;
using System.Collections;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(RobotArmSetup))]
[RequireComponent(typeof(UR5IKSolver))]
public class UR5Controller : MonoBehaviour
{
    [Header("IK Settings")]
    private float moveDuration = 1f;
    public float stiffness = 10000f;
    public float damping = 100f;

    [Header("Gripper")]
    public float gripperOpenAngle = 0f;
    public float gripperCloseAngle = 0.04f;

    // Component references
    private RobotArmSetup robotArmSetup;
    private UR5IKSolver ikSolver;
    private SuctionController suctionController;

    // Robot references
    private ArticulationBody[] articulationChain;
    private ArticulationBody[] robotJoints; // 6 joints + end effector (7 total)
    private Transform endEffector;

    private float[] currentJointAngles = new float[6];
    private Coroutine moveCoroutine;
    private bool initialized = false;

    void Start()
    {
        Initialize();
    }

    private void Initialize()
    {
        if (initialized) return;

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

        // Get IK solver
        ikSolver = GetComponent<UR5IKSolver>();
        if (ikSolver == null)
        {
            Debug.LogError("UR5Controller: UR5IKSolver component not found!");
            enabled = false;
            return;
        }

        // Get suction controller (optional)
        suctionController = GetComponent<SuctionController>();

        // Configure joint drives
        ConfigureJointDrives();

        // Initialize current angles
        UpdateCurrentAngles();

        initialized = true;
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

    void UpdateCurrentAngles()
    {
        // Get current angles from the 6 robot joints
        for (int i = 0; i < 6; i++)
        {
            if (robotJoints[i] != null)
            {
                currentJointAngles[i] = robotJoints[i].jointPosition[0];
            }
        }
    }

    /// <summary>
    /// Move to target position and rotation using IK
    /// </summary>
    public void MoveToTarget(Vector3 targetPosition, Quaternion targetRotation)
    {
        if (!initialized) Initialize();

        float[] targetAngles = ikSolver.SolveIK(targetPosition, targetRotation, currentJointAngles);

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAngles(targetAngles));
        }
        else
        {
            Debug.LogWarning("UR5Controller: No IK solution found for target position");
        }
    }

    /// <summary>
    /// Apply delta action from VLA model
    /// </summary>
    public void ApplyDeltaAction(float[] deltaAction)
    {
        if (!initialized) Initialize();

        // deltaAction = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
        if (deltaAction.Length < 7)
        {
            Debug.LogError("Delta action must have 7 elements");
            return;
        }

        // Get current end effector pose
        Vector3 currentPos = endEffector.position;
        Quaternion currentRot = endEffector.rotation;

        // Solve IK with delta
        float[] poseDelta = new float[6];
        System.Array.Copy(deltaAction, poseDelta, 6);

        float[] targetAngles = ikSolver.SolveIKDelta(currentPos, currentRot, poseDelta, currentJointAngles);

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAngles(targetAngles));
        }
        else
        {
            Debug.LogWarning("UR5Controller: No IK solution found for delta action");
        }

        // Control gripper/suction
        float gripperCmd = deltaAction[6];
        SetGripper(gripperCmd > 0.5f);
    }

    /// <summary>
    /// Set joint angles directly (in radians)
    /// </summary>
    public void SetJointAngles(float[] angles)
    {
        if (!initialized) Initialize();

        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (robotJoints[i] != null)
            {
                ArticulationDrive drive = robotJoints[i].xDrive;
                drive.target = angles[i] * Mathf.Rad2Deg;
                robotJoints[i].xDrive = drive;
            }
        }
        currentJointAngles = (float[])angles.Clone();
    }

    /// <summary>
    /// Set joint angles instantly (teleport)
    /// </summary>
    public void SetJointAnglesInstant(float[] angles)
    {
        if (!initialized) Initialize();

        for (int i = 0; i < Mathf.Min(6, angles.Length); i++)
        {
            if (robotJoints[i] != null)
            {
                var positions = new ArticulationReducedSpace(angles[i]);
                robotJoints[i].jointPosition = positions;
            }
        }
        currentJointAngles = (float[])angles.Clone();
    }

    private IEnumerator MoveToAngles(float[] targetAngles)
    {
        float[] startAngles = (float[])currentJointAngles.Clone();
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
        if (!initialized) Initialize();

        if (endEffector != null)
        {
            return (endEffector.position, endEffector.rotation);
        }
        return (Vector3.zero, Quaternion.identity);
    }

    /// <summary>
    /// Get current joint angles in radians
    /// </summary>
    public float[] GetJointAngles()
    {
        if (!initialized) Initialize();

        UpdateCurrentAngles();
        return (float[])currentJointAngles.Clone();
    }

    /// <summary>
    /// Get reference to RobotArmSetup
    /// </summary>
    public RobotArmSetup GetRobotArmSetup()
    {
        if (!initialized) Initialize();
        return robotArmSetup;
    }
}
