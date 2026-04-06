using System;
using System.Collections.Generic;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Replays recorded episodes by setting joint angles directly each frame.
/// </summary>
public class JointPlayback : EpisodePlayback
{
    private int[] jointAngleCols = new int[JointCount];
    private List<float[]> jointAnglesTrajectory = new List<float[]>();

    protected override bool ResolveColumns(string[] headers)
    {
        for (int j = 0; j < JointCount; j++)
        {
            jointAngleCols[j] = Array.IndexOf(headers, $"joint_{j}");
            if (jointAngleCols[j] < 0)
            {
                Debug.LogError($"Missing Joint Angle Column {j}");
                return false;
            }
        }
        return true;
    }

    protected override bool ParseRow(string[] values)
    {
        float[] jointAngles = new float[JointCount];
        for (int j = 0; j < JointCount; j++)
        {
            float.TryParse(values[jointAngleCols[j]], out jointAngles[j]);
            jointAngles[j] *= Mathf.Deg2Rad;
        }
        jointAnglesTrajectory.Add(jointAngles);
        return true;
    }

    protected override void ClearSubclassData()
    {
        jointAnglesTrajectory.Clear();
    }

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
        ur5Controller.SetJointAngles(jointAnglesTrajectory[targetFrame]);
        ur5Controller.SetGripper(suctionStates[targetFrame]);
    }
}
