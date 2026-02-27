#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;
using System;
using System.Collections.Generic;

using static ConstantsUR5;

[RequireComponent(typeof(RobotArmSetup))]
public class RobotController : MonoBehaviour
{
    private RobotArmSetup robotSetup;
    private ArticulationBody[] articulationChain;
    private ArticulationBody[] robotJoints;
    private GameObject targetBlock;
    private bool isMovingToBlock = false;
    private float targetReachedThreshold = 0.01f; // Distance threshold to consider target reached

    /// <summary>
    /// Target the chain should bent to
    /// </summary>
    public Transform Target;
    public Transform Pole;

    /// <summary>
    /// Solver iterations per update
    /// </summary>
    [Header("Solver Parameters")]
    public int Iterations = 10;

    /// <summary>
    /// Distance when the solver stops
    /// </summary>
    public float Delta = 0.001f;

    /// <summary>
    /// Strength of going back to the start position.
    /// </summary>
    [Range(0, 1)]
    public float SnapBackStrength = 1f;


    protected float[] LinksLength; //Target to Origin
    protected float CompleteLength;
    protected Transform[] Links;
    protected Vector3[] Positions;
    protected Vector3[] StartDirectionSucc;
    protected Quaternion[] StartRotationBone;
    protected Quaternion StartRotationTarget;
    protected Transform Root;
    protected Transform EndEffector;

    void Start()
    {
        Init();
    }

    void Init()
    {
        // Get references to required components
        robotSetup = GetComponent<RobotArmSetup>();
        targetBlock = robotSetup.blockToPickup;

        if (robotSetup == null || targetBlock == null)
        {
            Debug.LogError("Missing required components in RobotController!");
            enabled = false;
            return;
        }

        //initial array
        Links = new Transform[JointCount + 1];
        Positions = new Vector3[JointCount + 1];
        LinksLength = new float[JointCount];
        StartDirectionSucc = new Vector3[JointCount + 1];
        StartRotationBone = new Quaternion[JointCount + 1];

        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        
        if (articulationChain == null || articulationChain.Length == 0)
        {
            Debug.LogError("Articulation chain is not properly set up in RobotController.");
            return;
        }

        foreach (ArticulationBody joint in articulationChain)
        {
            if (JointNames.ContainsKey(joint.name))
            {
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = JointFriction;
                joint.angularDamping = AngularDamping;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = ForceLimit;
                joint.xDrive = currentDrive;
            }
        }

        robotJoints = new ArticulationBody[JointCount + 1];
        Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);
        robotJoints[JointCount] = articulationChain[EndEffectorIndex];

        EndEffector = robotJoints[JointCount].transform;
        Root = EndEffector;
        for (var i = 0; i < JointCount; i++)
        {
            if (Root == null)
                throw new UnityException("The chain value is longer than the ancestor chain!");
            Root = Root.parent;
        }
        Debug.Log($"Arm root: {Root.name}, End Effector: {EndEffector.name}");

        Target = targetBlock.transform;
        StartRotationTarget = GetRotationRootSpace(Target);

        //init data
        var current = EndEffector;
        CompleteLength = 0;
        for (var i = Links.Length - 1; i >= 0; i--)
        {
            Links[i] = current;
            StartRotationBone[i] = GetRotationRootSpace(current);

            if (i == Links.Length - 1)
            {
                //leaf
                StartDirectionSucc[i] = GetPositionRootSpace(Target) - GetPositionRootSpace(current);
            }
            else
            {
                //mid bone
                StartDirectionSucc[i] = GetPositionRootSpace(Links[i + 1]) - GetPositionRootSpace(current);
                LinksLength[i] = StartDirectionSucc[i].magnitude;
                CompleteLength += LinksLength[i];
            }
            current = current.parent;
        }
    }

    void Update()
    {
        // Start moving to block when 'S' key is pressed
        if (Input.GetKeyDown(KeyCode.S) && !isMovingToBlock)
        {
            StartMovingToBlock();
        }

        // Check if we're moving and have reached the target
        if (isMovingToBlock)
        {
            CheckTargetReached();
        }
    }

    void LateUpdate()
    {
        if (isMovingToBlock) ResolveIK();
    }

    private void StartMovingToBlock()
    {
        Debug.Log("Moving robot arm to block position...");
        isMovingToBlock = true;

        // Set the IK target to the block's position
        if (Target != null)
        {
            //Target.position = targetBlock.transform.position;
        }
        else
        {
            Debug.LogError("IK Target is not set!");
            isMovingToBlock = false;
        }
    }

    private void CheckTargetReached()
    {
        if (Target != null)
        {
            // Check if we've reached the target
            float distance = Vector3.Distance(EndEffector.position, targetBlock.transform.position);
            if (distance <= targetReachedThreshold)
            {
                Debug.Log("Robot arm has reached the block!");
                //isMovingToBlock = false; //if not commented out arm will instantly reset
            }
        }
    }

    private void ResolveIK()
    {
        if (Target == null)
            return;

        if (LinksLength.Length != JointCount)
        {
            Debug.Log($"LinksLength.Length: {LinksLength.Length}, JointCount: {JointCount}, re-initialising IK due to chain length mismatch");
            Init();
        }
            

        //Fabric

        //  root
        //  (bone0) (bonelen 0) (bone1) (bonelen 1) (bone2)...
        //   x--------------------x--------------------x---...

        //get position
        for (int i = 0; i < Links.Length; i++)
            Positions[i] = GetPositionRootSpace(Links[i]);

        var targetPosition = GetPositionRootSpace(Target);
        var targetRotation = GetRotationRootSpace(Target);

        //1st is possible to reach?
        if ((targetPosition - GetPositionRootSpace(Links[0])).sqrMagnitude >= CompleteLength * CompleteLength)
        {
            //just strech it
            var direction = (targetPosition - Positions[0]).normalized;
            //set everything after root
            for (int i = 1; i < Positions.Length; i++)
                Positions[i] = Positions[i - 1] + direction * LinksLength[i - 1];
        }
        else
        {
            for (int i = 0; i < Positions.Length - 1; i++)
                Positions[i + 1] = Vector3.Lerp(Positions[i + 1], Positions[i] + StartDirectionSucc[i], SnapBackStrength);

            for (int iteration = 0; iteration < Iterations; iteration++)
            {
                //https://www.youtube.com/watch?v=UNoX65PRehA
                //back
                for (int i = Positions.Length - 1; i > 0; i--)
                {
                    if (i == Positions.Length - 1)
                        Positions[i] = targetPosition; //set it to target
                    else
                        Positions[i] = Positions[i + 1] + (Positions[i] - Positions[i + 1]).normalized * LinksLength[i]; //set in line on distance
                }

                //forward
                for (int i = 1; i < Positions.Length; i++)
                    Positions[i] = Positions[i - 1] + (Positions[i] - Positions[i - 1]).normalized * LinksLength[i - 1];

                //close enough?
                if ((Positions[Positions.Length - 1] - targetPosition).sqrMagnitude < Delta * Delta)
                    break;
            }
        }

        //move towards pole
        if (Pole != null)
        {
            var polePosition = GetPositionRootSpace(Pole);
            for (int i = 1; i < Positions.Length - 1; i++)
            {
                var plane = new Plane(Positions[i + 1] - Positions[i - 1], Positions[i - 1]);
                var projectedPole = plane.ClosestPointOnPlane(polePosition);
                var projectedBone = plane.ClosestPointOnPlane(Positions[i]);
                var angle = Vector3.SignedAngle(projectedBone - Positions[i - 1], projectedPole - Positions[i - 1], plane.normal);
                Positions[i] = Quaternion.AngleAxis(angle, plane.normal) * (Positions[i] - Positions[i - 1]) + Positions[i - 1];
            }
        }

        //set position & rotation
        for (int i = 0; i < Positions.Length; i++)
        {
            if (i == Positions.Length - 1)
                SetRotationRootSpace(Links[i], Quaternion.Inverse(targetRotation) * StartRotationTarget * Quaternion.Inverse(StartRotationBone[i]));
            else
                SetRotationRootSpace(Links[i], Quaternion.FromToRotation(StartDirectionSucc[i], Positions[i + 1] - Positions[i]) * Quaternion.Inverse(StartRotationBone[i]));
            SetPositionRootSpace(Links[i], Positions[i]);
        }
    }

    private Vector3 GetPositionRootSpace(Transform current)
    {
        if (Root == null)
            return current.position;
        else
            return Quaternion.Inverse(Root.rotation) * (current.position - Root.position);
    }

    private void SetPositionRootSpace(Transform current, Vector3 position)
    {
        if (Root == null)
            current.position = position;
        else
            current.position = Root.rotation * position + Root.position;
    }

    private Quaternion GetRotationRootSpace(Transform current)
    {
        //inverse(after) * before => rot: before -> after
        if (Root == null)
            return current.rotation;
        else
            return Quaternion.Inverse(current.rotation) * Root.rotation;
    }

    private void SetRotationRootSpace(Transform current, Quaternion rotation)
    {
        if (Root == null)
            current.rotation = rotation;
        else
            current.rotation = Root.rotation * rotation;
    }

    void OnDrawGizmos()
    {
#if UNITY_EDITOR
        var current = this.transform;
        for (int i = 0; i < JointCount && current != null && current.parent != null; i++)
        {
            var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
            Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
            Handles.color = Color.green;
            Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
            current = current.parent;
        }
#endif
    }
}
