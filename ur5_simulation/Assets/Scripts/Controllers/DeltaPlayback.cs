using System;
using System.Collections.Generic;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Replays recorded episodes using UR5Controller.MoveDelta to apply
/// per-frame deltas from the CSV. Sets initial pose from frame 0 joint angles.
/// </summary>
public class DeltaPlayback : EpisodePlayback
{
    private UR5Controller ur5Controller;
    private RobotArmSetup robotArmSetup;

    // Parsed data
    private List<float[]> jointAnglesTrajectory = new List<float[]>();
    private List<Vector3> deltaPositions = new List<Vector3>();
    private List<Quaternion> deltaRotations = new List<Quaternion>();

    // Column indices
    private int[] jointAngleCols = new int[JointCount];
    private int dpxCol, dpyCol, dpzCol;
    private int drxCol, dryCol, drzCol, drwCol;

    /// <summary>
    /// Disable controllers in Awake (before Start runs) so UR5Controller.Start()
    /// never executes and reconfigures joint drives.
    /// </summary>
    private void Awake()
    {
        var ur5 = GetComponent<UR5Controller>();
        if (ur5 != null)
            ur5.enabled = false;
        var unified = GetComponent<UnifiedRobotController>();
        if (unified != null)
            unified.enabled = false;
    }

    protected override void InitializeController()
    {
        ur5Controller = GetComponent<UR5Controller>();
        robotArmSetup = GetComponent<RobotArmSetup>();

        if (ur5Controller == null)
            Debug.LogError("DeltaPlayback: No UR5Controller found!");
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

        dpxCol = Array.IndexOf(headers, "delta_pos_x");
        dpyCol = Array.IndexOf(headers, "delta_pos_y");
        dpzCol = Array.IndexOf(headers, "delta_pos_z");
        drxCol = Array.IndexOf(headers, "delta_rot_x");
        dryCol = Array.IndexOf(headers, "delta_rot_y");
        drzCol = Array.IndexOf(headers, "delta_rot_z");
        drwCol = Array.IndexOf(headers, "delta_rot_w");
    }

    protected override void ParseRow(string[] values)
    {
        // Joint angles (needed for initial pose)
        float[] jointAngles = new float[JointCount];
        for (int j = 0; j < JointCount; j++)
        {
            if (jointAngleCols[j] >= 0 && jointAngleCols[j] < values.Length)
                float.TryParse(values[jointAngleCols[j]], out jointAngles[j]);
        }
        jointAnglesTrajectory.Add(jointAngles);

        // Delta pose
        float dpx = 0f, dpy = 0f, dpz = 0f;
        float drx = 0f, dry = 0f, drz = 0f, drw = 1f;
        if (dpxCol >= 0) float.TryParse(values[dpxCol], out dpx);
        if (dpyCol >= 0) float.TryParse(values[dpyCol], out dpy);
        if (dpzCol >= 0) float.TryParse(values[dpzCol], out dpz);
        if (drxCol >= 0) float.TryParse(values[drxCol], out drx);
        if (dryCol >= 0) float.TryParse(values[dryCol], out dry);
        if (drzCol >= 0) float.TryParse(values[drzCol], out drz);
        if (drwCol >= 0) float.TryParse(values[drwCol], out drw);
        deltaPositions.Add(new Vector3(dpx, dpy, dpz));
        deltaRotations.Add(new Quaternion(drx, dry, drz, drw));
    }

    protected override void ClearSubclassData()
    {
        jointAnglesTrajectory.Clear();
        deltaPositions.Clear();
        deltaRotations.Clear();
    }

    protected override void OnPlaybackStart()
    {
        // Set initial pose directly on articulation drives
        if (jointAnglesTrajectory.Count > 0)
            SetJointAnglesDirect(jointAnglesTrajectory[0]);

        if (ur5Controller != null)
            ur5Controller.enabled = true;
    }

    protected override void OnPlaybackStop()
    {
        if (ur5Controller != null)
            ur5Controller.enabled = false;
    }

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
        if (ur5Controller == null)
        {
            Debug.LogError("DeltaPlayback: UR5Controller is null!");
            return;
        }

        // Apply each delta exactly once, catching up if frames were skipped
        for (int f = lastAppliedFrame + 1; f <= targetFrame; f++)
        {
            if (f < 0 || f >= deltaPositions.Count)
                continue;

            Transform ee = robotArmSetup != null
                ? robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform
                : null;
            Vector3 posBefore = ee != null ? ee.position : Vector3.zero;

            Vector3 intendedDelta = deltaPositions[f];
            ur5Controller.MoveDelta(intendedDelta, deltaRotations[f]);
            ur5Controller.SetGripper(suctionStates[f]);

            Vector3 posAfter = ee != null ? ee.position : Vector3.zero;
            Vector3 actualDelta = posAfter - posBefore;

            Debug.Log(
                $"DeltaPlayback frame {f}: "
                + $"diff=({(intendedDelta - actualDelta).magnitude:F5}), "
                + $"intended=({intendedDelta.x:F5}, {intendedDelta.y:F5}, {intendedDelta.z:F5}), "
                + $"actual=({actualDelta.x:F5}, {actualDelta.y:F5}, {actualDelta.z:F5})"
            );
        }
    }

    public override void JumpToFrame(int frameIndex)
    {
        Debug.LogWarning("DeltaPlayback: JumpToFrame not supported in delta mode (deltas are cumulative).");
    }

    private void SetJointAnglesDirect(float[] angles)
    {
        if (robotArmSetup == null || robotArmSetup.robotJoints == null)
        {
            Debug.LogError("DeltaPlayback: RobotArmSetup or robotJoints not available!");
            return;
        }

        ArticulationBody[] joints = robotArmSetup.robotJoints;
        for (int i = 0; i < Mathf.Min(JointCount, angles.Length); i++)
        {
            if (joints[i] == null)
                continue;
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = angles[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }
}
