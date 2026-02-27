using UnityEngine;
using System;

public class UR5IKSolver : MonoBehaviour
{
    [Header("Robot Links")]
    public ArticulationBody baseJoint;        // shoulder_pan_joint
    public ArticulationBody shoulderJoint;    // shoulder_lift_joint
    public ArticulationBody elbowJoint;       // elbow_joint
    public ArticulationBody wrist1Joint;      // wrist_1_joint
    public ArticulationBody wrist2Joint;      // wrist_2_joint
    public ArticulationBody wrist3Joint;      // wrist_3_joint

    [Header("Link Lengths")]
    public float shoulderLength = 0.425f;  // Distance between shoulder and elbow
    public float elbowLength = 0.392f;     // Distance between elbow and wrist
    public float wrist1Length = 0.0997f;   // Distance between wrist1 and wrist2
    public float wrist2Length = 0.0996f;   // Distance between wrist2 and wrist3
    public float toolLength = 0.0823f;     // Length of the end effector tool

    [Header("Joint Limits (in degrees)")]
    public Vector2 baseLimits = new Vector2(-360f, 360f);
    public Vector2 shoulderLimits = new Vector2(-360f, 360f);
    public Vector2 elbowLimits = new Vector2(-180f, 180f);
    public Vector2 wrist1Limits = new Vector2(-360f, 360f);
    public Vector2 wrist2Limits = new Vector2(-360f, 360f);
    public Vector2 wrist3Limits = new Vector2(-360f, 360f);

    private void Start()
    {
        ValidateSetup();
    }

    private void ValidateSetup()
    {
        if (baseJoint == null || shoulderJoint == null || elbowJoint == null ||
            wrist1Joint == null || wrist2Joint == null || wrist3Joint == null)
        {
            Debug.LogError("UR5IKSolver: All joints must be assigned!");
            enabled = false;
            return;
        }
    }

    public bool SolveIK(Vector3 targetPosition, Quaternion targetRotation)
    {
        try
        {
            // Transform target position to robot base space
            Vector3 localTarget = transform.InverseTransformPoint(targetPosition);
            
            // Calculate base rotation (theta1)
            float theta1 = Mathf.Atan2(localTarget.y, localTarget.x);
            
            // Transform target position into the plane of the arm after base rotation
            float projDist = Mathf.Sqrt(localTarget.x * localTarget.x + localTarget.y * localTarget.y);
            float targetX = projDist;
            float targetY = localTarget.z;

            // Adjust for tool offset
            targetY -= toolLength;

            // Calculate arm plane IK using cosine law
            float shoulderToWrist = Mathf.Sqrt(targetX * targetX + targetY * targetY);
            float a = shoulderLength;
            float b = elbowLength;
            float c = shoulderToWrist;

            // Check if target is reachable
            if (c > (a + b) * 0.99999f)
            {
                Debug.LogWarning("Target position out of reach!");
                return false;
            }

            // Calculate shoulder and elbow angles using cosine law
            float cosElbow = (a * a + b * b - c * c) / (2 * a * b);
            cosElbow = Mathf.Clamp(cosElbow, -1f, 1f);
            float theta3 = Mathf.PI - Mathf.Acos(cosElbow); // elbow angle

            float K1 = a + b * Mathf.Cos(theta3);
            float K2 = b * Mathf.Sin(theta3);
            float theta2 = Mathf.Atan2(targetY, targetX) - Mathf.Atan2(K2, K1);

            // Calculate wrist angles based on target rotation
            Quaternion localRotation = Quaternion.Inverse(transform.rotation) * targetRotation;
            Vector3 targetEuler = localRotation.eulerAngles;
            
            float theta4 = -theta2 - theta3; // Keep wrist 1 level
            float theta5 = Mathf.Deg2Rad * targetEuler.y;
            float theta6 = Mathf.Deg2Rad * targetEuler.z;

            // Apply joint angles
            ApplyJointAngles(new float[] {
                theta1,
                theta2,
                theta3,
                theta4,
                theta5,
                theta6
            });

            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"IK Solver error: {e.Message}");
            return false;
        }
    }

    private void ApplyJointAngles(float[] angles)
    {
        // Convert angles to degrees and apply joint limits
        float[] degreeAngles = new float[6];
        degreeAngles[0] = ClampAngle(angles[0] * Mathf.Rad2Deg, baseLimits.x, baseLimits.y);
        degreeAngles[1] = ClampAngle(angles[1] * Mathf.Rad2Deg, shoulderLimits.x, shoulderLimits.y);
        degreeAngles[2] = ClampAngle(angles[2] * Mathf.Rad2Deg, elbowLimits.x, elbowLimits.y);
        degreeAngles[3] = ClampAngle(angles[3] * Mathf.Rad2Deg, wrist1Limits.x, wrist1Limits.y);
        degreeAngles[4] = ClampAngle(angles[4] * Mathf.Rad2Deg, wrist2Limits.x, wrist2Limits.y);
        degreeAngles[5] = ClampAngle(angles[5] * Mathf.Rad2Deg, wrist3Limits.x, wrist3Limits.y);

        // Apply to articulation bodies
        ApplyRotationToJoint(baseJoint, degreeAngles[0]);
        ApplyRotationToJoint(shoulderJoint, degreeAngles[1]);
        ApplyRotationToJoint(elbowJoint, degreeAngles[2]);
        ApplyRotationToJoint(wrist1Joint, degreeAngles[3]);
        ApplyRotationToJoint(wrist2Joint, degreeAngles[4]);
        ApplyRotationToJoint(wrist3Joint, degreeAngles[5]);
    }

    private void ApplyRotationToJoint(ArticulationBody joint, float targetAngle)
    {
        var drive = joint.xDrive;
        drive.target = targetAngle;
        joint.xDrive = drive;
    }

    private float ClampAngle(float angle, float min, float max)
    {
        // Normalize angle to -180 to 180 range
        while (angle > 180f) angle -= 360f;
        while (angle < -180f) angle += 360f;
        
        return Mathf.Clamp(angle, min, max);
    }

    public void ResetToHome()
    {
        ApplyJointAngles(new float[] { 0f, 0f, 0f, 0f, 0f, 0f });
    }
}
