using UnityEngine;
using System;

public class StoredTrajectoryModel
{

    public float[] jointAngles;
    public bool suctionState;
    public bool attractedState;

    public StoredTrajectoryModel(float[] currentJointAngles, bool currentSuctionState, bool currentAttractedState)
    {
        jointAngles = currentJointAngles;
        suctionState = currentSuctionState;
        attractedState = currentAttractedState;
    }

}
