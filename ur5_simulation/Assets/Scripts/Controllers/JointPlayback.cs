using UnityEngine;

/// <summary>
/// Replays recorded episodes by setting joint angles directly each frame.
/// </summary>
public class JointPlayback : EpisodePlayback
{
    private UnifiedRobotController robotController;

    /// <summary>
    /// Disable UR5Controller in Awake (before its Start runs) so it never
    /// reconfigures joint drives.
    /// </summary>
    private void Awake()
    {
        var ur5 = GetComponent<UR5Controller>();
        if (ur5 != null)
            ur5.enabled = false;
    }

    protected override void InitializeController()
    {
        robotController = GetComponent<UnifiedRobotController>();
        if (robotController == null)
            Debug.LogError("JointPlayback: No UnifiedRobotController found!");
    }

    protected override void ResolveColumns(string[] headers) { }

    protected override void ParseRow(string[] values) { }

    protected override void ClearSubclassData() { }

    protected override void ApplyFrames(int lastAppliedFrame, int targetFrame)
    {
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
