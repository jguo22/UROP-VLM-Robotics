using System;
using UnityEngine;
using static ConstantsUR5;

[DefaultExecutionOrder(-1)]
public class RobotArmSetup : MonoBehaviour
{
    public ArticulationBody[] robotJoints;
    private ArticulationBody[] articulationChain; // Automatically populated articulation chain
    private SuctionController suctionController;

    private void Start()
    {
        ConfigureArticulationBodies();
        ResetArmPosition();
    }

    private void ConfigureArticulationBodies()
    {
        suctionController = GetComponent<SuctionController>();
        if (suctionController == null)
        {
            Debug.LogError("SuctionController component not found on RobotArmSetup GameObject.");
            return;
        }

        articulationChain = GetComponentsInChildren<ArticulationBody>();
        Debug.Log($"Found {articulationChain.Length} articulation bodies in the robot arm.");
        if (articulationChain.Length == 0)
        {
            Debug.LogError("No articulation bodies found in the robot arm hierarchy.");
            return;
        }

        for (int i = BaseIndex; i < BaseIndex + JointCount; i++)
        {
            ArticulationBody joint = articulationChain[i];
            joint.jointFriction = JointFriction;
            joint.angularDamping = AngularDamping;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = ForceLimit;
            joint.xDrive = currentDrive;
        }

        robotJoints = new ArticulationBody[JointCount + 1];
        Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);
        robotJoints[JointCount] = articulationChain[EndEffectorIndex];
    }

    public void ResetArmPosition()
    {
        for (int i = 0; i < JointCount; i++)
        {
            ArticulationDrive currentDrive = robotJoints[i].xDrive;
            currentDrive.target = StableStartingRotations[i] * Mathf.Rad2Deg;
            robotJoints[i].xDrive = currentDrive;
        }

        Debug.Log("Arm reset to stable position");
    }
}