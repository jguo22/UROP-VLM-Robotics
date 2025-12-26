using UnityEngine;
using System;
using System.Collections.Generic;

/// <summary>
/// Analytical Inverse Kinematics solver for UR5 robot.
/// Based on "Analytic IK for Universal Robots UR-5/UR-10" by Kelsey P. Hawkins
/// </summary>
public class UR5IKSolver : MonoBehaviour
{
    // UR5 DH Parameters (in meters)
    private const float d1 = 0.089159f;
    private const float a2 = -0.42500f;
    private const float a3 = -0.39225f;
    private const float d4 = 0.10915f;
    private const float d5 = 0.09465f;
    private const float d6 = 0.0823f;

    /// <summary>
    /// Solve IK for target position and rotation
    /// </summary>
    /// <param name="targetPosition">Target end-effector position in world space</param>
    /// <param name="targetRotation">Target end-effector rotation in world space</param>
    /// <param name="currentAngles">Current joint angles (for selecting closest solution)</param>
    /// <returns>Array of 6 joint angles in radians, or null if no solution</returns>
    public float[] SolveIK(Vector3 targetPosition, Quaternion targetRotation, float[] currentAngles = null)
    {
        // Convert Unity coordinates to robot base frame
        // Unity: Y-up, Z-forward
        // Robot: Z-up, X-forward (typically)
        Vector3 pos = TransformToRobotFrame(targetPosition);
        Matrix4x4 rotMatrix = Matrix4x4.Rotate(targetRotation);

        // Get all possible solutions
        List<float[]> solutions = GetAllSolutions(pos, rotMatrix);

        if (solutions.Count == 0)
        {
            print(targetPosition);
            Debug.LogWarning("No IK solution found - target may be unreachable");
            return null;
        }

        // Select best solution (closest to current angles)
        return SelectBestSolution(solutions, currentAngles);
    }

    /// <summary>
    /// Solve IK using delta actions (like OpenVLA output)
    /// </summary>
    public float[] SolveIKDelta(Vector3 currentPos, Quaternion currentRot,
                                 float[] deltaAction, float[] currentAngles)
    {
        // deltaAction = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw] in END-EFFECTOR frame
        Vector3 deltaPosEE = new Vector3(deltaAction[0], deltaAction[1], deltaAction[2]);
        Vector3 deltaEuler = new Vector3(deltaAction[3], deltaAction[4], deltaAction[5]) * Mathf.Rad2Deg;

        // Transform position delta from end-effector frame to world frame
        Vector3 deltaPosWorld = currentRot * deltaPosEE;
        Vector3 targetPos = currentPos + deltaPosWorld;

        // Apply rotation delta
        Quaternion deltaRot = Quaternion.Euler(deltaEuler);
        Quaternion targetRot = currentRot * deltaRot;

        return SolveIK(targetPos, targetRot, currentAngles);
    }

    private Vector3 TransformToRobotFrame(Vector3 unityPos)
    {
        // Adjust based on how your robot is oriented in Unity
        // This assumes robot base is at origin with Z-up in robot frame
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }

    private List<float[]> GetAllSolutions(Vector3 pos, Matrix4x4 rotMatrix)
    {
        List<float[]> solutions = new List<float[]>();

        float x = pos.x;
        float y = pos.y;
        float z = pos.z;

        // Extract rotation matrix columns
        Vector3 r13 = new Vector3(rotMatrix.m02, rotMatrix.m12, rotMatrix.m22);

        // Wrist center position
        Vector3 wristCenter = pos - d6 * r13;
        float xc = wristCenter.x;
        float yc = wristCenter.y;
        float zc = wristCenter.z;

        // Joint 1: Two solutions (shoulder left/right)
        float[] theta1Options = GetTheta1Options(xc, yc);

        foreach (float theta1 in theta1Options)
        {
            // Joint 5: Two solutions (wrist up/down)
            float[] theta5Options = GetTheta5Options(theta1, rotMatrix);

            foreach (float theta5 in theta5Options)
            {
                // Joint 6
                float theta6 = GetTheta6(theta1, theta5, rotMatrix);

                // Joints 2, 3, 4
                float[][] theta234Options = GetTheta234Options(theta1, wristCenter);

                foreach (float[] theta234 in theta234Options)
                {
                    float[] solution = new float[6];
                    solution[0] = theta1;
                    solution[1] = theta234[0];
                    solution[2] = theta234[1];
                    solution[3] = theta234[2];
                    solution[4] = theta5;
                    solution[5] = theta6;

                    // Validate solution
                    if (IsValidSolution(solution))
                    {
                        solutions.Add(solution);
                    }
                }
            }
        }

        return solutions;
    }

    private float[] GetTheta1Options(float xc, float yc)
    {
        List<float> options = new List<float>();
        float r = Mathf.Sqrt(xc * xc + yc * yc);

        if (r < 1e-6f)
        {
            options.Add(0f);
            return options.ToArray();
        }

        float alpha = Mathf.Atan2(yc, xc);

        if (Mathf.Abs(d4 / r) <= 1f)
        {
            float beta = Mathf.Asin(Mathf.Clamp(d4 / r, -1f, 1f));
            options.Add(alpha - beta + Mathf.PI);
            options.Add(alpha + beta);
        }
        else
        {
            options.Add(alpha);
        }

        return options.ToArray();
    }

    private float[] GetTheta5Options(float theta1, Matrix4x4 rotMatrix)
    {
        List<float> options = new List<float>();

        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);

        // P5_6_y calculation
        float p5_6_y = -s1 * rotMatrix.m02 + c1 * rotMatrix.m12;

        if (Mathf.Abs(p5_6_y) <= 1f)
        {
            float theta5 = Mathf.Acos(Mathf.Clamp(p5_6_y, -1f, 1f));
            options.Add(theta5);
            options.Add(-theta5);
        }

        return options.ToArray();
    }

    private float GetTheta6(float theta1, float theta5, Matrix4x4 rotMatrix)
    {
        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);
        float s5 = Mathf.Sin(theta5);

        if (Mathf.Abs(s5) < 1e-6f)
        {
            return 0f; // Singularity
        }

        float x = (s1 * rotMatrix.m00 - c1 * rotMatrix.m10) / s5;
        float y = (s1 * rotMatrix.m01 - c1 * rotMatrix.m11) / s5;

        return Mathf.Atan2(y, x);
    }

    private float[][] GetTheta234Options(float theta1, Vector3 wristCenter)
    {
        List<float[]> options = new List<float[]>();

        float c1 = Mathf.Cos(theta1);
        float s1 = Mathf.Sin(theta1);

        // Transform to joint 1 frame
        float px = c1 * wristCenter.x + s1 * wristCenter.y;
        float py = -s1 * wristCenter.x + c1 * wristCenter.y - d4;
        float pz = wristCenter.z - d1;

        // Distance from joint 2 to wrist center
        float D = Mathf.Sqrt(px * px + pz * pz);

        float L2 = Mathf.Abs(a2);
        float L3 = Mathf.Abs(a3);

        // Check reachability
        if (D > L2 + L3 || D < Mathf.Abs(L2 - L3))
        {
            return options.ToArray();
        }

        // Joint 3: Two solutions (elbow up/down)
        float cosTheta3 = (D * D - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        cosTheta3 = Mathf.Clamp(cosTheta3, -1f, 1f);

        float[] theta3Options = new float[] { Mathf.Acos(cosTheta3), -Mathf.Acos(cosTheta3) };

        foreach (float theta3 in theta3Options)
        {
            // Joint 2
            float beta = Mathf.Atan2(pz, px);
            float psi = Mathf.Atan2(L3 * Mathf.Sin(theta3), L2 + L3 * Mathf.Cos(theta3));
            float theta2 = beta - psi;

            // Joint 4 (simplified - you may need to refine this based on orientation)
            float theta4 = 0f; // Placeholder - compute based on remaining rotation

            options.Add(new float[] { theta2, theta3, theta4 });
        }

        return options.ToArray();
    }

    private bool IsValidSolution(float[] solution)
    {
        // UR5 joint limits (in radians)
        float[] minLimits = { -2 * Mathf.PI, -2 * Mathf.PI, -Mathf.PI, -2 * Mathf.PI, -2 * Mathf.PI, -2 * Mathf.PI };
        float[] maxLimits = { 2 * Mathf.PI, 2 * Mathf.PI, Mathf.PI, 2 * Mathf.PI, 2 * Mathf.PI, 2 * Mathf.PI };

        for (int i = 0; i < 6; i++)
        {
            if (float.IsNaN(solution[i]) || float.IsInfinity(solution[i]))
                return false;

            if (solution[i] < minLimits[i] || solution[i] > maxLimits[i])
                return false;
        }

        return true;
    }

    private float[] SelectBestSolution(List<float[]> solutions, float[] currentAngles)
    {
        if (currentAngles == null || currentAngles.Length != 6)
        {
            return solutions[0];
        }

        float minDistance = float.MaxValue;
        float[] bestSolution = solutions[0];

        foreach (float[] solution in solutions)
        {
            float distance = 0f;
            for (int i = 0; i < 6; i++)
            {
                float diff = NormalizeAngle(solution[i] - currentAngles[i]);
                distance += diff * diff;
            }

            if (distance < minDistance)
            {
                minDistance = distance;
                bestSolution = solution;
            }
        }

        return bestSolution;
    }

    private float NormalizeAngle(float angle)
    {
        while (angle > Mathf.PI) angle -= 2 * Mathf.PI;
        while (angle < -Mathf.PI) angle += 2 * Mathf.PI;
        return angle;
    }
}
