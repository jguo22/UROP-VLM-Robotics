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
    private List<Vector3> deltaPositions = new List<Vector3>();
    private List<Quaternion> deltaRotations = new List<Quaternion>();

    // Tracking for batch-over-batch logging
    private Vector3 previousBatchEEPosition;
    private Vector3 previousBatchExpectedDelta;
    private bool hasPreviousBatch;

    // Column indices
    private int dpxCol,
        dpyCol,
        dpzCol;
    private int drxCol,
        dryCol,
        drzCol,
        drwCol;

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
        float dpx = 0f,
            dpy = 0f,
            dpz = 0f;
        float drx = 0f,
            dry = 0f,
            drz = 0f,
            drw = 1f;
        if (dpxCol >= 0)
            float.TryParse(values[dpxCol], out dpx);
        if (dpyCol >= 0)
            float.TryParse(values[dpyCol], out dpy);
        if (dpzCol >= 0)
            float.TryParse(values[dpzCol], out dpz);
        if (drxCol >= 0)
            float.TryParse(values[drxCol], out drx);
        if (dryCol >= 0)
            float.TryParse(values[dryCol], out dry);
        if (drzCol >= 0)
            float.TryParse(values[drzCol], out drz);
        if (drwCol >= 0)
            float.TryParse(values[drwCol], out drw);
        deltaPositions.Add(new Vector3(dpx, dpy, dpz));
        deltaRotations.Add(new Quaternion(drx, dry, drz, drw));
    }

    protected override void ClearSubclassData()
    {
        deltaPositions.Clear();
        deltaRotations.Clear();
    }

    protected override void OnPlaybackStart()
    {
        if (ur5Controller != null)
            ur5Controller.enabled = true;

        hasPreviousBatch = false;
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

        Transform ee =
            robotArmSetup != null
                ? robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform
                : null;
        Vector3 currentPos = ee != null ? ee.position : Vector3.zero;

        // Log actual vs expected movement since last batch
        if (hasPreviousBatch)
        {
            Vector3 actualMovement = currentPos - previousBatchEEPosition;
            Debug.Log(
                $"DeltaPlayback batch ending at frame {targetFrame}: "
                    + $"sinceLastBatch: actual=({actualMovement.x:F5}, {actualMovement.y:F5}, {actualMovement.z:F5}), "
                    + $"expected=({previousBatchExpectedDelta.x:F5}, {previousBatchExpectedDelta.y:F5}, {previousBatchExpectedDelta.z:F5}), "
                    + $"error={(actualMovement - previousBatchExpectedDelta).magnitude:F5}"
            );
        }

        // Save EE position at start of this batch and accumulate expected deltas
        previousBatchEEPosition = currentPos;
        Vector3 batchExpectedDelta = Vector3.zero;

        // Apply each delta exactly once, catching up if frames were skipped
        for (int f = lastAppliedFrame + 1; f <= targetFrame; f++)
        {
            if (f < 0 || f >= deltaPositions.Count)
                continue;

            Vector3 intendedDelta = deltaPositions[f];
            batchExpectedDelta += intendedDelta;
            ur5Controller.MoveDelta(intendedDelta, deltaRotations[f]);
            ur5Controller.SetGripper(suctionStates[f]);
        }

        previousBatchExpectedDelta = batchExpectedDelta;
        hasPreviousBatch = true;
    }

}
