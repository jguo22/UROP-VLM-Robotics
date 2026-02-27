using UnityEngine;
using System;
using System.Collections;
using System.IO;
using System.Collections.Generic;

[RequireComponent(typeof(RobotArmController))]
public class PickAndPlaceController : MonoBehaviour
{
    [Header("Scene References")]
    public GameObject blockToPickup;        // Reference to the block
    public GameObject platform;             // Reference to the platform
    public Transform suctionEndEffector;    // Reference to the suction end effector

    [Header("Movement Settings")]
    public float moveSpeed = 2.0f;         // Speed of arm movement
    public float pickupHeight = 0.1f;      // Height above block for approach
    public float suctionDistance = 0.02f;  // Distance to activate suction
    public KeyCode activationKey = KeyCode.Space;  // Hotkey to start the pick and place operation

    [Header("Task Positions")]
    public Vector3 platformPlacePosition;   // Position on platform to place block

    // Private references and state
    private RobotArmController robotController;
    private bool isTaskActive = false;
    private bool isSuctionActive = false;
    private PickAndPlaceState currentState = PickAndPlaceState.Idle;

    private enum PickAndPlaceState
    {
        Idle,
        MovingToPickup,
        Approaching,
        Picking,
        Lifting,
        MovingToPlace,
        Placing,
        Returning,
        Complete
    }

    private void Start()
    {
        // Get reference to the RobotArmController
        robotController = GetComponent<RobotArmController>();
        if (robotController == null)
        {
            Debug.LogError("RobotArmController not found!");
            enabled = false;
            return;
        }

        // Validate required references
        if (blockToPickup == null || platform == null || suctionEndEffector == null)
        {
            Debug.LogError("Required scene references not set in PickAndPlaceController!");
            enabled = false;
            return;
        }

        // Calculate platform place position (center of platform, slightly above surface)
        platformPlacePosition = platform.transform.position + Vector3.up * 0.05f;
    }

    private void Update()
    {
        // Check for hotkey press to start the task
        if (Input.GetKeyDown(activationKey) && !isTaskActive)
        {
            StartPickAndPlace();
        }

        // Update the picking process
        if (isTaskActive)
        {
            UpdatePickAndPlace();
        }
    }

    private void StartPickAndPlace()
    {
        isTaskActive = true;
        currentState = PickAndPlaceState.MovingToPickup;
        robotController.StartRecording(); // Start recording coordinates
        Debug.Log("Pick and Place task started");
    }

    private void UpdatePickAndPlace()
    {
        switch (currentState)
        {
            case PickAndPlaceState.MovingToPickup:
                MoveToPickupPosition();
                break;
            case PickAndPlaceState.Approaching:
                ApproachBlock();
                break;
            case PickAndPlaceState.Picking:
                PickupBlock();
                break;
            case PickAndPlaceState.Lifting:
                LiftBlock();
                break;
            case PickAndPlaceState.MovingToPlace:
                MoveToPlacePosition();
                break;
            case PickAndPlaceState.Placing:
                PlaceBlock();
                break;
            case PickAndPlaceState.Returning:
                ReturnToHome();
                break;
            case PickAndPlaceState.Complete:
                CompleteTask();
                break;
        }
    }

    private void MoveToPickupPosition()
    {
        Vector3 targetPosition = blockToPickup.transform.position + Vector3.up * pickupHeight;
        if (MoveEndEffectorTo(targetPosition))
        {
            currentState = PickAndPlaceState.Approaching;
            Debug.Log("Moving to approach position");
        }
    }

    private void ApproachBlock()
    {
        Vector3 targetPosition = blockToPickup.transform.position + Vector3.up * 0.02f;
        if (MoveEndEffectorTo(targetPosition))
        {
            currentState = PickAndPlaceState.Picking;
            Debug.Log("Approaching block");
        }
    }

    private void PickupBlock()
    {
        // Check if we're close enough to activate suction
        if (Vector3.Distance(suctionEndEffector.position, blockToPickup.transform.position) <= suctionDistance)
        {
            ActivateSuction();
            currentState = PickAndPlaceState.Lifting;
            Debug.Log("Picking up block");
        }
    }

    private void LiftBlock()
    {
        Vector3 liftPosition = blockToPickup.transform.position + Vector3.up * pickupHeight;
        if (MoveEndEffectorTo(liftPosition))
        {
            currentState = PickAndPlaceState.MovingToPlace;
            Debug.Log("Lifting block");
        }
    }

    private void MoveToPlacePosition()
    {
        Vector3 placeApproachPosition = platformPlacePosition + Vector3.up * pickupHeight;
        if (MoveEndEffectorTo(placeApproachPosition))
        {
            currentState = PickAndPlaceState.Placing;
            Debug.Log("Moving to place position");
        }
    }

    private void PlaceBlock()
    {
        if (MoveEndEffectorTo(platformPlacePosition))
        {
            DeactivateSuction();
            currentState = PickAndPlaceState.Returning;
            Debug.Log("Placing block");
        }
    }

    private void ReturnToHome()
    {
        // Move to a safe position above the platform
        Vector3 homePosition = platformPlacePosition + Vector3.up * 0.2f;
        if (MoveEndEffectorTo(homePosition))
        {
            currentState = PickAndPlaceState.Complete;
            Debug.Log("Returning to home position");
        }
    }

    private void CompleteTask()
    {
        robotController.StopRecording(); // Stop recording coordinates
        isTaskActive = false;
        currentState = PickAndPlaceState.Idle;
        Debug.Log("Pick and Place task completed");
    }

    [Header("IK References")]
    private UR5IKSolver ikSolver;
    private Vector3 currentTargetPosition;
    private bool isMoving = false;
    private float moveStartTime;
    private Vector3 moveStartPosition;
    private float journeyLength;

    private void Awake()
    {
        ikSolver = GetComponent<UR5IKSolver>();
        if (ikSolver == null)
        {
            Debug.LogError("UR5IKSolver component not found!");
            enabled = false;
        }
    }

    private bool MoveEndEffectorTo(Vector3 targetPosition)
    {
        // Initialize movement if we're not already moving to this target
        if (!isMoving || currentTargetPosition != targetPosition)
        {
            StartMovement(targetPosition);
        }

        // Calculate movement progress
        float distanceCovered = (Time.time - moveStartTime) * moveSpeed;
        float fractionOfJourney = distanceCovered / journeyLength;

        // Smoothly interpolate position
        Vector3 currentTarget = Vector3.Lerp(moveStartPosition, targetPosition, fractionOfJourney);

        // Apply IK
        bool ikSuccess = ikSolver.SolveIK(currentTarget, suctionEndEffector.rotation);

        // Check if we've reached the target
        float remainingDistance = Vector3.Distance(suctionEndEffector.position, targetPosition);
        if (remainingDistance < 0.01f || fractionOfJourney >= 1.0f)
        {
            isMoving = false;
            return true;
        }

        return false;
    }

    private void StartMovement(Vector3 targetPosition)
    {
        isMoving = true;
        currentTargetPosition = targetPosition;
        moveStartTime = Time.time;
        moveStartPosition = suctionEndEffector.position;
        journeyLength = Vector3.Distance(moveStartPosition, targetPosition);
    }

    private void ActivateSuction()
    {
        isSuctionActive = true;
        // Implement suction logic here
        // This might involve parenting the block to the end effector
        blockToPickup.transform.parent = suctionEndEffector;
        blockToPickup.GetComponent<Rigidbody>().isKinematic = true;
    }

    private void DeactivateSuction()
    {
        isSuctionActive = false;
        // Implement suction deactivation logic here
        blockToPickup.transform.parent = null;
        blockToPickup.GetComponent<Rigidbody>().isKinematic = false;
    }

    private void OnDrawGizmos()
    {
        // Draw debug visualization
        if (platform != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(platformPlacePosition, 0.02f);
        }
    }
    
    public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press space key to move robot to pick up block.", centeredStyle);
    }
}
