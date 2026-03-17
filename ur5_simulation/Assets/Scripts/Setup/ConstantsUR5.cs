using UnityEngine;

//using Preliy.Flange;

public static class ConstantsUR5
{
    public const int JointStiffness = 10000;
    public const int JointDamping = 100;
    public const int JointFriction = 10;
    public const int AngularDamping = 10;
    public const int ForceLimit = 1000;
    public const float JointSpeed = 15f; //15 deg/s
    public const float JointSlowSpeed = 1f; //1 deg/s
    public const float JointSuperSlowSpeed = 0.01f; //0.01 deg/s
    public const float JointAcceleration = 30f; //20 deg/s^2
    public const float JointSlowAcceleration = 2f; //2 deg/s^2
    public const float JointSuperSlowAcceleration = 0.01f; //0.01 deg/s^2
    public const float JointVelocityLimit = 1f; // theoretical max is 3.15
    public const float SmallestJointAngle = -359f;
    public const float LargestJointAngle = 359f;

    public const int JointCount = 6;
    public const int BaseIndex = 3;
    public const int EndEffectorIndex = 11;
    public const int CoordinateRecordLength = JointCount + 3; //no of joints + end effector + block + platform

    public static readonly float[] StableStartingRotations = new float[] //rotations in radians
    {
        Mathf.PI, // Base rotation
        -0.8f, // Shoulder (lowered to better reach cubes)
        1.0f, // Elbow (adjusted to position arm lower)
        -1.5f, // Wrist 1 (adjusted to position suction cup better)
        -1.57f, // Wrist 2
        0f // Wrist 3
        ,
    };

    // Suction params

    public const float SuctionDistance = 0.005f; //distance from suction cup to object to activate suction
    public const float SuctionOffsetY = -0.01f; //offset from end effector to suction cup along Y axis
    public const float SuctionIKOffsetY = 0.0f; //0.002f //offset from end effector to suction cup along Y axis for IK calculations
}
