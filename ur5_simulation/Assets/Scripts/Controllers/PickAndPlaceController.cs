using UnityEngine;
using System;
using System.Collections;
using System.IO;
//using System.Collections.Generic;
//using System.Text;

using static ConstantsUR5;

[RequireComponent(typeof(RobotArmSetup))]
public class PickAndPlaceController : MonoBehaviour
{
    private GameObject blockToPickup;        // Reference to the block
    private GameObject platform;             // Reference to the platform
    private Transform suctionEndEffector;    // Reference to the suction end effector
    private ArticulationBody[] articulationChain; // Automatically populated articulation chain

    [Header("Movement Settings")]
    public float moveSpeed = 2.0f;         // Speed of arm movement
    public float pickupHeight = 0.1f;      // Height above block for approach
    public float suctionDistance = 0.05f;  // Distance to activate suction
    public float placementHeight = 0.05f;  // Height above platform for placement

    // Component references
    private RobotArmSetup robotArmSetup;
    private bool isPickAndPlaceActive = false;
    private bool isSuctionActive = false;
    private bool keyPressed = false;
    private string logFilePath;
    
    // Waypoint system
    private Vector3[] currentWaypoints;
    private Vector3[] pickupWaypoints;
    private Vector3[] placeWaypoints;
    private int currentWaypointIndex;
    private bool isExecutingWaypoints;
    private bool isInPickupSequence;

    private void LogToFile(string message)
    {
        if (string.IsNullOrEmpty(logFilePath)) return;
        
        try
        {
            using (StreamWriter writer = File.AppendText(logFilePath))
            {
                writer.WriteLine($"[{DateTime.Now:HH:mm:ss.fff}] {message}");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to write to log file: {e.Message}");
        }
    }
    private const float waypointThreshold = 0.01f;  // Distance to consider a waypoint reached
    
    // State tracking
    private PickAndPlaceState currentState = PickAndPlaceState.Idle;

    // State machine for pick and place operation
    private enum PickAndPlaceState
    {
        Idle,
        ExecutingWaypoints,
        Complete
    }
    
    // Define standard waypoint sequences
    private Vector3[] GeneratePickupWaypoints()
    {
        Vector3 blockPos = blockToPickup.transform.position;
        pickupWaypoints = new Vector3[]
        {
            blockPos + Vector3.up * 0.2f,              // Move above block
            blockPos + Vector3.up * 0.1f,              // First approach
            blockPos + Vector3.up * 0.001f,            // Touch block - much closer approach
            blockPos + Vector3.up * 0.05f,             // Initial lift
            blockPos + Vector3.up * 0.1f,              // Continue lift
            blockPos + Vector3.up * 0.2f,              // Full lift
        };
        return pickupWaypoints;
    }

    private Vector3[] GeneratePlaceWaypoints()
    {
        Vector3 platformPos = platform.transform.position;
        placeWaypoints = new Vector3[]
        {
            platformPos + Vector3.up * 0.2f,           // Move above platform
            platformPos + Vector3.up * 0.1f,           // First approach
            platformPos + Vector3.up * placementHeight, // Final placement
            platformPos + Vector3.up * 0.1f,           // Lift after release
            platformPos + Vector3.up * 0.2f,           // Full lift
            transform.position + Vector3.up * 0.5f     // Return home
        };
        return placeWaypoints;
    }

    private void Start()
    {
        // Setup logging
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        logFilePath = Path.Combine(Application.dataPath, "..", "Logs", $"RobotArm_{timestamp}.log");
        Directory.CreateDirectory(Path.GetDirectoryName(logFilePath));
        LogToFile("=== Robot Arm Controller Started ===");

        robotArmSetup = GetComponent<RobotArmSetup>();
        if (robotArmSetup == null)
        {
            LogToFile("ERROR: RobotArmSetup component not found!");
            Debug.LogError("RobotArmSetup component not found!");
            enabled = false;
            return;
        }

        blockToPickup = robotArmSetup.blockToPickup;
        //platform = robotArmSetup.platform;
        articulationChain = robotArmSetup.articulationChain;
        suctionEndEffector = articulationChain[8].transform;

        LogToFile($"Block position: {blockToPickup.transform.position}");
        //LogToFile($"Platform position: {platform.transform.position}");
        Debug.Log($"Block position: {blockToPickup.transform.position}");
        //Debug.Log($"Platform position: {platform.transform.position}");
    }

    private void Update()
    {
        // Start pick and place operation when 's' is pressed
        if (Input.GetKeyDown(KeyCode.S) && !isPickAndPlaceActive && !keyPressed)
        {
            StartPickAndPlace();
            keyPressed = true;
        }

        // Update the pick and place operation
        if (isPickAndPlaceActive)
        {
            UpdatePickAndPlace();
        }
    }

    private void StartPickAndPlace()
    {
        isPickAndPlaceActive = true;
        robotArmSetup.StartRecording();
        
        // Start with pickup waypoints
        currentWaypoints = GeneratePickupWaypoints();
        currentWaypointIndex = 0;
        isExecutingWaypoints = true;
        isInPickupSequence = true;
        currentState = PickAndPlaceState.ExecutingWaypoints;
        
        LogToFile("Starting pick and place operation");
        Debug.Log("Starting pick and place operation with waypoints");
    }

    private void UpdatePickAndPlace()
    {
        if (!isExecutingWaypoints) return;

        // Get current waypoint
        Vector3 currentTarget = currentWaypoints[currentWaypointIndex];
        bool reached = MoveEndEffectorTo(currentTarget);

        if (reached)
        {
            HandleWaypointReached();
        }
    }

    private bool IsPickupSequence()
    {
        Vector3[] pickupWaypoints = GeneratePickupWaypoints();
        if (currentWaypoints.Length != pickupWaypoints.Length) return false;
        
        // Check first and last waypoint to identify the sequence
        return Vector3.Distance(currentWaypoints[0], pickupWaypoints[0]) < 0.01f &&
               Vector3.Distance(currentWaypoints[currentWaypoints.Length - 1], pickupWaypoints[pickupWaypoints.Length - 1]) < 0.01f;
    }

    private void HandleWaypointReached()
    {
        LogToFile($"Reached waypoint {currentWaypointIndex}");
        Debug.Log($"Reached waypoint {currentWaypointIndex}");
        
        // Handle pickup sequence
        if (isInPickupSequence)
        {
            switch (currentWaypointIndex)
            {
                case 2: // At block
                    float distanceToBlock = Vector3.Distance(suctionEndEffector.position, blockToPickup.transform.position);
                    LogToFile($"Distance to block: {distanceToBlock}");
                    Debug.Log($"Distance to block: {distanceToBlock}");
                    
                    if (!isSuctionActive && distanceToBlock <= suctionDistance)
                    {
                        LogToFile($"Attempting to pick up block at distance {distanceToBlock}");
                        ActivateSuction();
                        // Stay at current waypoint if pickup failed
                        if (!isSuctionActive) return;
                    }
                    break;
                case 5: // Finished pickup sequence
                    if (!isSuctionActive)
                    {
                        LogToFile("Failed to pick up block, retrying sequence");
                        currentWaypointIndex = 0;
                        return;
                    }
                    // Switch to place sequence
                    currentWaypoints = GeneratePlaceWaypoints();
                    currentWaypointIndex = 0;
                    isInPickupSequence = false;
                    return;
            }
        }
        // Handle place sequence
        else
        {
            switch (currentWaypointIndex)
            {
                case 2: // At platform
                    DeactivateSuction();
                    break;
                case 5: // Finished place sequence
                    CompleteOperation();
                    return;
            }
        }

        // Move to next waypoint
        currentWaypointIndex++;
        if (currentWaypointIndex >= currentWaypoints.Length)
        {
            isExecutingWaypoints = false;
        }
    }

    private void CompleteOperation()
    {
        robotArmSetup.StopRecording();
        isPickAndPlaceActive = false;
        isExecutingWaypoints = false;
        currentState = PickAndPlaceState.Complete;
        Debug.Log("Pick and place operation completed");
        keyPressed = false;
    }

    public bool MoveEndEffectorTo(Vector3 targetPosition)
    {
        // Get the indices of the joints we want to control (3-8)
        ArticulationBody[] joints = robotArmSetup.articulationChain;
        Transform endEffector = joints[8].transform;
        Transform baseJoint = joints[3].transform;

        // Calculate distance to target
        float distance = Vector3.Distance(endEffector.position, targetPosition);
        if (distance < 0.01f)
        {
            return true; // Reached target
        }

        // Debug logging
        LogToFile("\n=== Movement Update ===");
        LogToFile($"End effector position: {endEffector.position}");
        LogToFile($"Target position: {targetPosition}");
        Debug.Log($"End effector position: {endEffector.position}");
        Debug.Log($"Target position: {targetPosition}");

        // Calculate relative position from base
        Vector3 baseToTarget = targetPosition - baseJoint.position;
        LogToFile($"Base to target vector: {baseToTarget}");
        Debug.Log($"Base to target vector: {baseToTarget}");

        // Calculate base rotation using the correct coordinate system
        float baseAngle = Mathf.Atan2(baseToTarget.x, baseToTarget.z);
        // Ensure the base angle is in the correct quadrant
        if (baseAngle < 0)
            baseAngle += 2 * Mathf.PI;
            
        LogToFile($"Base angle: {baseAngle * Mathf.Rad2Deg:F2} degrees");
        Debug.Log($"Base angle: {baseAngle * Mathf.Rad2Deg} degrees");

        // Calculate shoulder and elbow angles using inverse kinematics
        Vector3 shoulderPos = joints[4].transform.position;
        
        // Get the vertical offset from base to shoulder
        float baseToShoulderHeight = shoulderPos.y - joints[3].transform.position.y;
        
        // Transform target into shoulder space, keeping the height offset in mind
        Vector3 localTarget = Quaternion.Euler(0, -baseAngle * Mathf.Rad2Deg, 0) * (targetPosition - shoulderPos);
        localTarget.y += baseToShoulderHeight;  // Account for shoulder height offset
        
        Vector3 targetVec = localTarget;
        float targetDist = new Vector2(targetVec.x, targetVec.z).magnitude; // Use horizontal distance for IK
        
        Debug.Log($"Target Position: {targetPosition}, Base Angle: {baseAngle * Mathf.Rad2Deg}°");

        // Link lengths (adjust these based on your UR5 model)
        float upperArmLength = 0.425f;  // Length of the upper arm
        float forearmLength = 0.392f;   // Length of the forearm

        // Calculate shoulder and elbow angles using cosine law
        float shoulderAngle, elbowAngle;

        // Calculate the height difference for shoulder angle
        float heightDiff = targetVec.y;
        float horizontalDist = targetDist;

        // Using law of cosines to calculate elbow angle
        float cosElbow = (horizontalDist * horizontalDist + heightDiff * heightDiff 
                         - upperArmLength * upperArmLength - forearmLength * forearmLength) 
                        / (2 * upperArmLength * forearmLength);
        cosElbow = Mathf.Clamp(cosElbow, -1f, 1f);
        elbowAngle = Mathf.Acos(cosElbow);

        // Calculate shoulder angle considering both horizontal distance and height
        float elevationAngle = Mathf.Atan2(heightDiff, horizontalDist);
        float k1 = upperArmLength + forearmLength * cosElbow;
        float k2 = forearmLength * Mathf.Sin(elbowAngle);
        float reachAngle = Mathf.Atan2(k2, k1);
        shoulderAngle = elevationAngle - reachAngle;

        // Calculate wrist angles to maintain end effector orientation
        float wrist1Angle = -(shoulderAngle + elbowAngle);  // Compensate for arm angles
        float wrist2Angle = -Mathf.PI/2;  // Keep end effector pointed downward
        float wrist3Angle = baseAngle;    // Align with approach direction

        // Convert angles to degrees
        var jointAngles = new[] {
            (3, baseAngle * Mathf.Rad2Deg),
            (4, shoulderAngle * Mathf.Rad2Deg),
            (5, elbowAngle * Mathf.Rad2Deg),
            (6, wrist1Angle * Mathf.Rad2Deg),
            (7, wrist2Angle * Mathf.Rad2Deg),
            (8, wrist3Angle * Mathf.Rad2Deg)
        };

        // Apply the calculated angles to each joint
        foreach (var (jointIndex, targetAngle) in jointAngles)
        {
            var joint = joints[jointIndex];
            var drive = joint.xDrive;
            float currentAngle = joint.jointPosition[0] * Mathf.Rad2Deg;
            
            // For base joint (index 3), we need to handle the rotation direction differently
            float angleToSet;
            if (jointIndex == 3)
            {
                // Use the calculated base angle directly, already in the correct coordinate system
                angleToSet = targetAngle;
                Debug.Log($"Setting base joint angle to: {angleToSet}");
            }
            else
            {
                angleToSet = targetAngle;
            }
            
            float angleDiff = Mathf.DeltaAngle(currentAngle, angleToSet);
            
            // Smooth movement
            float newAngle = currentAngle + Mathf.Clamp(angleDiff * moveSpeed * Time.deltaTime, -5f, 5f);
            
            // Update joint drive
            drive.target = newAngle;
            drive.stiffness = 10000f; // High stiffness for precise movement
            drive.damping = 100f;     // Damping to prevent oscillation
            joint.xDrive = drive;
            
            // Log the angle changes
            LogToFile($"Joint {jointIndex} - Current: {currentAngle:F2}, Target: {angleToSet:F2}, New: {newAngle:F2}");
            Debug.Log($"Joint {jointIndex} - Current: {currentAngle:F2}, Target: {angleToSet:F2}, New: {newAngle:F2}");
        }

        // Check if end effector is close enough to target
        float distanceToTarget = Vector3.Distance(endEffector.position, targetPosition);
        LogToFile($"Distance to target: {distanceToTarget}");
        
        if (distanceToTarget <= waypointThreshold)
        {
            LogToFile("Target position reached");
            return true;
        }

        // Check if all joints are close to their target positions
        bool allJointsInPosition = true;
        foreach (var (jointIndex, targetAngle) in jointAngles)
        {
            float currentAngle = joints[jointIndex].jointPosition[0] * Mathf.Rad2Deg;
            if (Mathf.Abs(Mathf.DeltaAngle(currentAngle, targetAngle)) > 1f)
            {
                allJointsInPosition = false;
                break;
            }
        }

        return allJointsInPosition;
    }

    private void ActivateSuction()
    {
        if (isSuctionActive) return; // Prevent multiple activations
        
        isSuctionActive = true;
        LogToFile("Activating suction");
        Debug.Log("Activating suction");

        // Ensure the block is close enough
        float distanceToBlock = Vector3.Distance(suctionEndEffector.position, blockToPickup.transform.position);
        LogToFile($"Distance to block at suction: {distanceToBlock}");
        
        if (distanceToBlock <= suctionDistance * 1.5f)
        {
            blockToPickup.transform.position = suctionEndEffector.position; // Snap to position
            blockToPickup.transform.parent = suctionEndEffector;

            // Make the block kinematic while held
            Rigidbody blockRb = blockToPickup.GetComponent<Rigidbody>();
            if (blockRb != null)
            {
                blockRb.isKinematic = true;
            }

            LogToFile("Suction activated successfully");
            Debug.Log("Suction activated successfully");
        }
        else
        {
            isSuctionActive = false;
            LogToFile($"Failed to activate suction - block too far away ({distanceToBlock})");
            Debug.Log($"Failed to activate suction - block too far away ({distanceToBlock})");
        }
    }

    private void DeactivateSuction()
    {
        isSuctionActive = false;
        blockToPickup.transform.parent = null;

        // Restore the block's physics
        Rigidbody blockRb = blockToPickup.GetComponent<Rigidbody>();
        if (blockRb != null)
        {
            blockRb.isKinematic = false;
        }

        Debug.Log("Suction deactivated");
    }

    private void OnDrawGizmos()
    {
        if (isPickAndPlaceActive)
        {
            // Draw debug visualization
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(suctionEndEffector.position, suctionDistance);
        }
    }
    
    public void OnGUI()
    {
        GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
        centeredStyle.alignment = TextAnchor.UpperCenter;
        GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press s key to move robot to pick up block.", centeredStyle);
    }
}
