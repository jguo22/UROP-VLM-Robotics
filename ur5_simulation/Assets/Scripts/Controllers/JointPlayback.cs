using System;
using System.Collections.Generic;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Replays recorded episodes by setting joint angles directly each frame.
/// </summary>
public class JointPlayback : EpisodePlayback
{
    private UnifiedRobotController robotController;

    // Parsed joint angle data
    private List<float[]> jointAnglesTrajectory = new List<float[]>();
    private int[] jointAngleCols = new int[JointCount];

    protected override void InitializeController()
    {
        robotController = GetComponent<UnifiedRobotController>();
        if (robotController == null)
            Debug.LogError("JointPlayback: No UnifiedRobotController found!");

        // Disable UR5Controller if present to prevent drive conflicts
        var ur5 = GetComponent<UR5Controller>();
        if (ur5 != null)
            ur5.enabled = false;
    }

    protected override void ResolveColumns(string[] headers)
    {
        string[] jointNames = { "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" };
        for (int j = 0; j < JointCount; j++)
        {
            jointAngleCols[j] = Array.IndexOf(headers, $"joint_{j}");
            if (jointAngleCols[j] < 0 && j < jointNames.Length)
                jointAngleCols[j] = Array.IndexOf(headers, $"{jointNames[j]}_jointAngle");
        }
    }

    protected override void ParseRow(string[] values)
    {
        float[] jointAngles = new float[JointCount];
        for (int j = 0; j < JointCount; j++)
        {
            if (jointAngleCols[j] >= 0 && jointAngleCols[j] < values.Length)
                float.TryParse(values[jointAngleCols[j]], out jointAngles[j]);
        }
        jointAnglesTrajectory.Add(jointAngles);
    }

    protected override void ClearSubclassData()
    {
        jointAnglesTrajectory.Clear();
    }

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
        if (targetFrame < 0 || targetFrame >= jointAnglesTrajectory.Count)
        {
            Debug.LogWarning(
                $"JointPlayback: Invalid frame index {targetFrame}, trajectory has {jointAnglesTrajectory.Count} frames"
            );
            return;
        }

        if (robotController != null)
        {
            robotController.SetJointAngles(jointAnglesTrajectory[targetFrame]);
            robotController.SetSuctionState(suctionStates[targetFrame]);
        }
        else
        {
            Debug.LogError("JointPlayback: robotController is null!");
        }
    }
}
