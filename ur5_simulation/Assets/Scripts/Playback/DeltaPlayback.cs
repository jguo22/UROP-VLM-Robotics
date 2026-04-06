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

    // Column indices
    private int dpxCol,
        dpyCol,
        dpzCol,
        drxCol,
        dryCol,
        drzCol,
        drwCol;

    // Parsed data
    private List<Vector3> deltaPositions = new List<Vector3>();
    private List<Quaternion> deltaRotations = new List<Quaternion>();

    // Tracking for batch-over-batch logging
    private Vector3 previousBatchEEPosition;
    private Vector3 previousBatchIntendedDelta;
    private bool hasPreviousBatch;

    protected override void Start()
    {
        base.Start();
        RobotArmSetup robotArmSetup = GetComponent<RobotArmSetup>();
        Debug.Assert(robotArmSetup != null);
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
        hasPreviousBatch = false;
    }

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
        Vector3 currentPos = endEffector.position;

        // Log actual vs intended movement since last batch
        if (hasPreviousBatch)
        {
            Vector3 actualMovement = currentPos - previousBatchEEPosition;
            float error = (actualMovement - previousBatchIntendedDelta).magnitude;
            string message =
                $"DeltaPlayback frame {targetFrame}: "
                + $"error={error:F5}, "
                + $"sinceLastBatch: actual=({actualMovement.x:F5}, {actualMovement.y:F5}, {actualMovement.z:F5}), "
                + $"expected=({previousBatchIntendedDelta.x:F5}, {previousBatchIntendedDelta.y:F5}, {previousBatchIntendedDelta.z:F5})";
            if (error > 0.01)
            {
                Debug.LogWarning(message);
            }
            else
            {
                Debug.Log(message);
            }
        }

        // accumulate expected deltas over a batch if multiple frames happened
        Vector3 deltaPosition = Vector3.zero;
        Quaternion deltaRotation = Quaternion.identity;
        bool suctionOn = false;
        // Apply each delta exactly once, catching up if frames were skipped
        for (int f = lastAppliedFrame + 1; f <= targetFrame; f++)
        {
            deltaPosition += deltaPositions[f];
            deltaRotation *= deltaRotations[f];
        }

        // save for next frame logging
        previousBatchEEPosition = currentPos;
        previousBatchIntendedDelta = deltaPosition;
        hasPreviousBatch = true;

        // apply action
        ur5Controller.MoveDelta(deltaPosition, deltaRotation);
        ur5Controller.SetSuction(suctionOn);
    }
}
