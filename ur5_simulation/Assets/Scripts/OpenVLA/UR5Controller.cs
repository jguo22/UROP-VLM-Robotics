using UnityEngine;

// helper script, doesn't have any update code
[DefaultExecutionOrder(100)]
[RequireComponent(typeof(RobotArmSetup))]
public class UR5Controller : MonoBehaviour
{
    [Header("IK Settings")]
    public float stiffness = 10000f;
    public float damping = 100f;

    // Component references
    private RobotArmSetup robotArmSetup;
    private SuctionController suctionController;
    private UR5IKSolver ikSolver;

    // Robot references
    private ArticulationBody[] robotJoints; // 6 joints + end effector (7 total)
    private Transform endEffector;
    private Transform originTransform; // used to calculate the relative position of the end effector to the base of the robot

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

        // This is already set up by RobotArmSetup (6 joints + end effector)
        robotJoints = robotArmSetup.robotJoints;

        // End effector is the last element in robotJoints array
        endEffector = robotJoints[6].transform; // robotJoints[6] is the end effector
        originTransform = this.transform;

        // Get suction controller (optional)
        suctionController = GetComponent<SuctionController>();

        // Get IK solver
        ikSolver = GetComponent<UR5IKSolver>();
        if (ikSolver == null)
        {
            Debug.LogError("UR5Controller: UR5IKSolver component not found!");
            enabled = false;
            return;
        }

        // Configure joint drives
        ConfigureJointDrives();
    }

    void ConfigureJointDrives()
    {
        // Configure drives for the 6 robot arm joints (indices 0-5 in robotJoints)
        for (int i = 0; i < 6; i++)
        {
            if (robotJoints[i] == null)
                continue;

            ArticulationDrive drive = robotJoints[i].xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = 1000;
            robotJoints[i].xDrive = drive;
        }
    }

    // Move by a delta position and rotation (world coordinates)
    public void MoveDelta(Vector3 deltaPosition, Quaternion deltaRotation)
    {
        Vector3 targetPos = endEffector.position + deltaPosition;
        // Apply rotation in world coordinates: deltaRotation * currentRotation
        Quaternion targetRot = deltaRotation * endEffector.rotation;

        MoveToTarget(targetPos, targetRot);
    }

    // Move to target position and rotation using IK
    public void MoveToTarget(Vector3 targetPosition, Quaternion targetRotation)
    {
        (Vector3 relativePosition, Quaternion relativeRotation) = ConvertToRobotCoordinates(
            targetPosition,
            targetRotation
        );
        float[] currentAngles = GetJointAngles();
        float[] ikResult = ikSolver.SolveIK(relativePosition, relativeRotation, currentAngles);

        if (ikResult != null)
            SetJointAngles(ikResult);
        else
            Debug.LogWarning("UR5Controller: No IK solution found for target position");
    }

    // Set joint angles directly (in radians)
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

    // Get current joint angles in radians
    public float[] GetJointAngles()
    {
        float[] angles = new float[6]; // Assuming 6-DOF UR5 robot

        // Get the current joint angles from the joint controllers
        for (int i = 0; i < 6; i++)
        {
            angles[i] = robotJoints[i].jointPosition[0];
        }

        return angles;
    }

    // Control gripper/suction
    public void SetGripper(bool close)
    {
        suctionController.enableSuction = close;
    }

    (Vector3 position, Quaternion rotation) ConvertToRobotCoordinates(
        Vector3 inputPosition,
        Quaternion inputRotation
    )
    {
        Vector3 position = originTransform.InverseTransformPoint(inputPosition);
        Quaternion rotation = Quaternion.Inverse(originTransform.rotation) * inputRotation;
        return (position, rotation);
    }

    // Get current end effector pose
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

    void OnGUI()
    {
        if (endEffector == null)
            return;

        string displayText =
            $"End Effector Position: ({endEffector.position.x:F5}, {endEffector.position.y:F5}, {endEffector.position.z:F5})\n";
        displayText +=
            $"End Effector Rotation: ({endEffector.rotation.x:F5}, {endEffector.rotation.y:F5}, {endEffector.rotation.z:F5}, {endEffector.rotation.w:F5})\n";
        displayText +=
            $"Euler Angles: ({endEffector.rotation.eulerAngles.x:F2}, {endEffector.rotation.eulerAngles.y:F2}, {endEffector.rotation.eulerAngles.z:F2})";

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
