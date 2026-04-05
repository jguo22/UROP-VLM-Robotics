using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Replays recorded episodes using UR5Controller.MoveDelta to apply
/// per-frame deltas from the CSV. Sets initial pose from frame 0 joint angles.
/// </summary>
public class DeltaPlayback : EpisodePlayback
{
    private Transform endEffector;

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

    protected override void Start()
    {
        base.Start();
        RobotArmSetup robotArmSetup = GetComponent<RobotArmSetup>();
        if (robotArmSetup != null && robotArmSetup.robotJoints != null)
            endEffector = robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform;
    }

    protected override bool ResolveColumns(string[] headers)
    {
        dpxCol = Array.IndexOf(headers, "delta_pos_x");
        dpyCol = Array.IndexOf(headers, "delta_pos_y");
        dpzCol = Array.IndexOf(headers, "delta_pos_z");
        drxCol = Array.IndexOf(headers, "delta_rot_x");
        dryCol = Array.IndexOf(headers, "delta_rot_y");
        drzCol = Array.IndexOf(headers, "delta_rot_z");
        drwCol = Array.IndexOf(headers, "delta_rot_w");
        return dpxCol > 0
            && dpyCol > 0
            && dpzCol > 0
            && drxCol > 0
            && dryCol > 0
            && drzCol > 0
            && drwCol > 0;
    }

    protected override bool ParseRow(string[] values)
    {
        float.TryParse(values[dpxCol], out float dpx);
        float.TryParse(values[dpyCol], out float dpy);
        float.TryParse(values[dpzCol], out float dpz);
        float.TryParse(values[drxCol], out float drx);
        float.TryParse(values[dryCol], out float dry);
        float.TryParse(values[drzCol], out float drz);
        float.TryParse(values[drwCol], out float drw);
        deltaPositions.Add(new Vector3(dpx, dpy, dpz));
        deltaRotations.Add(new Quaternion(drx, dry, drz, drw));
        return true;
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

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
        if (ur5Controller == null)
        {
            Debug.LogError("DeltaPlayback: UR5Controller is null!");
            return;
        }

        Vector3 currentPos = endEffector != null ? endEffector.position : Vector3.zero;

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
