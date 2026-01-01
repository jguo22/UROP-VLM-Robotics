using UnityEngine;
using System.Collections;

// helper script, doesn't have any update code
[DefaultExecutionOrder(100)]
[RequireComponent(typeof(RobotArmSetup))]
public class UR5Controller : MonoBehaviour
{
    [Header("IK Settings")]
    private float moveDuration = 1f;
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
    public void MoveToTarget(Vector3 targetPosition, Quaternion targetRotation)
    {
        float[] targetAngles = ikSolver.SolveIK(targetPosition, targetRotation, GetJointAngles());

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAnglesCoroutine(targetAngles));
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
        // deltaAction = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
        if (deltaAction.Length < 7)
        {
            Debug.LogError("Delta action must have 7 elements");
            return;
        }

        // Get current end effector pose
        Vector3 currentPos = endEffector.position;
        Quaternion currentRot = endEffector.rotation;

        float[] targetAngles = ikSolver.SolveIKDelta(currentPos, currentRot, deltaAction, GetJointAngles());

        if (targetAngles != null)
        {
            if (moveCoroutine != null)
            {
                StopCoroutine(moveCoroutine);
            }
            moveCoroutine = StartCoroutine(MoveToAnglesCoroutine(targetAngles));
        }
        else
        {
            Debug.LogWarning("UR5Controller: No IK solution found for delta action");
        }

        // Control gripper/suction
        float gripperCmd = deltaAction[6];
        SetGripper(gripperCmd > 0.5f);
    }

    public void MoveToAngles(float[] targetAngles)
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
    public void SetJointAngles(float[] angles)
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

    private IEnumerator MoveToAnglesCoroutine(float[] targetAngles)
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
            Vector3 position = originTransform.InverseTransformPoint(endEffector.position);
            Quaternion rotation = Quaternion.Inverse(originTransform.rotation) * endEffector.rotation;
            return (position, rotation);
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
                angles[i] = robotJoints[i].jointPosition[0]; // Convert to radians
            }
        }

        return angles;
    }
}
