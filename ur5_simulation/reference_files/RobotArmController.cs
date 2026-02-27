/*
def reset_arm_position(self):
    """Reset the arm to a stable starting position"""
    # Define a stable starting configuration for the UR5 arm
    # These values are in radians for each joint
    stable_positions = [
        0,              # Base rotation
        -0.8,           # Shoulder (lowered to better reach cubes)
        1.0,            # Elbow (adjusted to position arm lower)
        -1.5,           # Wrist 1 (adjusted to position suction cup better)
        -1.57,          # Wrist 2
        0               # Wrist 3
    ]
    
    # Apply the joint positions
    for i in range(min(len(stable_positions), self.num_joints)):
        p.resetJointState(self.arm_id, i, stable_positions[i])
    
    # Update the position if it exists
    if hasattr(self, 'position'):
        self.position = np.array([0.5, 0.0], dtype=np.float32)
    
    # Update rotation if rotation_options exists
    if hasattr(self, 'rotation_options') and hasattr(self, 'rotation_index'):
        self.rotation_index = 1  # Use a higher position to avoid ground clipping
        self.rotation = self.rotation_options[self.rotation_index]
    
    # Let the simulation settle
    for _ in range(20):
        p.stepSimulation()
        time.sleep(0.01)
    
    print("Arm reset to stable position")

def load_platform_and_cubes(self, assets_root):
    """Load a square platform and multiple colored cubes into the scene"""
    try:
        # Create a square platform for stacking instead of using the bowl
        platform_size = [0.15, 0.15, 0.01]  # Size of the platform (width, length, height)
        platform_position = [0.6, 0.0, 0.005]  # Position of the platform
        platform_color = [0.2, 0.2, 0.8, 1]  # Blue color
        
        # Create a visual shape for the platform
        platform_visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[platform_size[0]/2, platform_size[1]/2, platform_size[2]/2],
            rgbaColor=platform_color
        )
        
        # Create a collision shape for the platform
        platform_collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[platform_size[0]/2, platform_size[1]/2, platform_size[2]/2]
        )
        
        # Create the platform as a multibody
        self.platform_id = p.createMultiBody(
            baseMass=0,  # Mass of 0 makes it static
            baseCollisionShapeIndex=platform_collision_shape,
            baseVisualShapeIndex=platform_visual_shape,
            basePosition=platform_position,
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        print(f"Created square platform at position {platform_position}")
        
        # Increase solver iterations for better physics stability
        p.setPhysicsEngineParameter(numSolverIterations=100)
        
        # Set cube properties for better physics
        cube_mass = 0.05  # Reduced mass for better stability (50 grams)
        cube_lateral_friction = 0.9  # High friction to prevent sliding
        cube_spinning_friction = 0.3  # High spinning friction to prevent rotation
        cube_rolling_friction = 0.3  # High rolling friction to prevent rolling
        cube_restitution = 0.01  # Very low bounciness
        
        # Load the cube URDF
        cube_urdf_path = os.path.join(assets_root, "block/block.urdf")
        if not os.path.exists(cube_urdf_path):
            print(f"Could not find cube URDF at {cube_urdf_path}")
            return
            
        # Create multiple cubes with different colors and positions
        self.cubes = {}  # Dictionary to store cube IDs and their colors
        
        # Define cube colors and positions
        cube_configs = [
            {"color": "red", "position": [0.5, 0.0, 0.05], "rgba": [1, 0, 0, 1]},
            {"color": "green", "position": [0.5, 0.15, 0.05], "rgba": [0, 1, 0, 1]},
            {"color": "blue", "position": [0.5, -0.15, 0.05], "rgba": [0, 0, 1, 1]},
            {"color": "yellow", "position": [0.35, 0.0, 0.05], "rgba": [1, 1, 0, 1]}
        ]
        
        # Load each cube
        for config in cube_configs:
            # Add small random offset for more natural placement
            rand_x = np.random.uniform(-0.01, 0.01)
            rand_y = np.random.uniform(-0.01, 0.01)
            position = [
                config["position"][0] + rand_x,
                config["position"][1] + rand_y,
                config["position"][2]
            ]
            
            # Load the cube
            cube_id = p.loadURDF(
                cube_urdf_path,
                position,
                p.getQuaternionFromEuler([0, 0, 0])  # Perfect alignment
            )
            
            # Change the color of the cube
            p.changeVisualShape(cube_id, -1, rgbaColor=config["rgba"])
            
            # Set physical properties for the cube
            p.changeDynamics(
                cube_id, 
                -1,  # Base link
                mass=cube_mass,
                lateralFriction=cube_lateral_friction,
                spinningFriction=cube_spinning_friction,
                rollingFriction=cube_rolling_friction,
                restitution=cube_restitution,
                linearDamping=0.5,  # Higher damping to reduce oscillations
                angularDamping=0.8,  # Higher angular damping to prevent spinning
                contactStiffness=20000,  # Stiffer contacts for stability
                contactDamping=200,  # Higher contact damping
                frictionAnchor=1  # Enable friction anchors for more stable friction
            )
            
            # Store the cube in our dictionary
            self.cubes[config["color"]] = {
                "id": cube_id,
                "color": config["color"],
                "position": position
            }
            
            print(f"Loaded {config['color']} cube at position {position}")
            
        # Store the red cube ID separately for backward compatibility
        self.cube_id = self.cubes["red"]["id"]
        
        # IMPORTANT: Ensure proper collision handling between all cubes
        print("Setting up proper collision handling between cubes...")
        for color1, cube_info1 in self.cubes.items():
            for color2, cube_info2 in self.cubes.items():
                if color1 != color2:
                    # Enable collision between this pair of cubes
                    p.setCollisionFilterPair(
                        cube_info1["id"], cube_info2["id"], 
                        -1, -1, 
                        1  # 1 = enable collision
                    )
                    print(f"Enabled collision between {color1} and {color2} cubes")
        
        # Let the cubes settle with gravity
        for _ in range(100):  # Increased settling time
            p.stepSimulation()
            time.sleep(0.01)
            
            # Check and correct any cubes that might be on their edges
            for color, cube_info in self.cubes.items():
                cube_id = cube_info["id"]
                pos, orn = p.getBasePositionAndOrientation(cube_id)
                euler = p.getEulerFromQuaternion(orn)
                
                # If cube is not flat (has significant roll or pitch)
                if abs(euler[0]) > 0.1 or abs(euler[1]) > 0.1:
                    # Apply a small random impulse to tip it over onto a face
                    rand_force = [np.random.uniform(-0.1, 0.1), np.random.uniform(-0.1, 0.1), -0.5]
                    p.applyExternalForce(
                        objectUniqueId=cube_id,
                        linkIndex=-1,
                        forceObj=rand_force,
                        posObj=[pos[0], pos[1], pos[2] + 0.025],
                        flags=p.WORLD_FRAME
                    )
            
    except Exception as e:
        print(f"Error loading platform and cubes: {e}")
*/


/*
Joint 0: world_joint
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0
Joint 1: rotated_base-base_fixed_joint
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0
Joint 2: shoulder_pan_joint
Joint type: 0
Lower limit: -6.28318530718, Upper limit: 6.28318530718
Max force: 150.0, Max velocity: 3.15
Joint 3: shoulder_lift_joint
Joint type: 0
Lower limit: -6.28318530718, Upper limit: 6.28318530718
Max force: 150.0, Max velocity: 3.15
Joint 4: elbow_joint
Joint type: 0
Lower limit: -3.14159265359, Upper limit: 3.14159265359
Max force: 150.0, Max velocity: 3.15
Joint 5: wrist_1_joint
Joint type: 0
Lower limit: -6.28318530718, Upper limit: 6.28318530718
Max force: 28.0, Max velocity: 3.2
Joint 6: wrist_2_joint
Joint type: 0
Lower limit: -6.28318530718, Upper limit: 6.28318530718
Max force: 28.0, Max velocity: 3.2
Joint 7: wrist_3_joint
Joint type: 0
Lower limit: -6.28318530718, Upper limit: 6.28318530718
Max force: 28.0, Max velocity: 3.2
Joint 8: ee_fixed_joint
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0
Joint 9: wrist_3_link-tool0_fixed_joint
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0
Joint 10: tool0-tool_tip
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0
Joint 11: base_link-base_fixed_joint
Joint type: 4
Lower limit: 0.0, Upper limit: -1.0
Max force: 0.0, Max velocity: 0.0

// Apply the joint positions
        for (int i = 0; i < Mathf.Min(stablePositions.Length, articulationChain.Length); i++)
        {
            var joint = articulationChain[i];
            if (joint.jointType == ArticulationJointType.FixedJoint)
                continue;

            var drive = joint.xDrive;
            drive.target = stablePositions[i] * Mathf.Rad2Deg; // Convert radians to degrees
            joint.xDrive = drive;

            // Set the initial joint position
            joint.jointPosition = new ArticulationReducedSpace(stablePositions[i] * Mathf.Rad2Deg);
        }

        Debug.Log("Arm reset to stable position");
*/

using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;
using System.Text;

public class RobotArmController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public ArticulationBody[] robotJoints; // Assign the UR5 joints in order in the inspector
    public ArticulationBody[] articulationChain; // Automatically populated articulation chain
    private readonly float[] stablePositions = new float[]
    {
        0f,     // Base rotation
        -0.8f,  // Shoulder (lowered to better reach cubes)
        1.0f,   // Elbow (adjusted to position arm lower)
        -1.5f,  // Wrist 1 (adjusted to position suction cup better)
        -1.57f, // Wrist 2
        0f      // Wrist 3
    };
    private int shoulderJointIdx = 3; // Index of the shoulder joint in the articulation chain
    private readonly string coordinateHeaders = "Timestamp,BasePosX,BasePosY,BasePosZ,BaseRotX,BaseRotY,BaseRotZ,BaseRotW,ShoulderPosX,ShoulderPosY,ShoulderPosZ,ShoulderRotX,ShoulderRotY,ShoulderRotZ,ShoulderRotW,ElbowPosX,ElbowPosY,ElbowPosZ,ElbowRotX,ElbowRotY,ElbowRotZ,ElbowRotW,Wrist1PosX,Wrist1PosY,Wrist1PosZ,Wrist1RotX,Wrist1RotY,Wrist1RotZ,Wrist1RotW,Wrist2PosX,Wrist2PosY,Wrist2PosZ,Wrist2RotX,Wrist2RotY,Wrist2RotZ,Wrist2RotW,EndEffectorPosX,EndEffectorPosY,EndEffectorPosZ,EndEffectorRotX,EndEffectorRotY,EndEffectorRotZ,EndEffectorRotW";

    [Header("Robot Configuration")]
    public Transform baseTransform; // Reference to the robot base transform
    public Transform shoulderTransform; // Reference to the shoulder transform
    public Transform elbowTransform; // Reference to the elbow transform
    public Transform wrist1Transform; // Reference to the wrist 1 transform
    public Transform wrist2Transform; // Reference to the wrist 2 transform
    public Transform endEffectorTransform; // Reference to the end effector transform
    public float samplingRate = 0.1f; // Time between coordinate samples in seconds
    public bool exportCoordinates = false; // Toggle coordinate export

    [Header("Export Settings")]
    public string exportFilePath = "RobotCoordinates"; // Base name for the export file
    public bool appendTimestamp = true; // Whether to append timestamp to filename

    private float nextSampleTime;
    private List<CoordinateRecord> coordinates;
    private string currentFilePath;
    private bool isRecording;

    // Structure to store coordinate data with timestamp
    private struct CoordinateRecord
    {
        public float timestamp;
        public Vector3 basePosition;
        public Quaternion baseRotation;
        public Vector3 shoulderPosition;
        public Quaternion shoulderRotation;
        public Vector3 elbowPosition;
        public Quaternion elbowRotation;
        public Vector3 wrist1Position;
        public Quaternion wrist1Rotation;
        public Vector3 wrist2Position;
        public Quaternion wrist2Rotation;
        public Vector3 endEffectorPosition;
        public Quaternion endEffectorRotation;

        public CoordinateRecord(float time, Vector3 basePos, Quaternion baseRot, Vector3 shoulderPos, Quaternion shoulderRot, Vector3 elbowPos, Quaternion elbowRot, Vector3 wrist1Pos, Quaternion wrist1Rot, Vector3 wrist2Pos, Quaternion wrist2Rot, Vector3 endPos, Quaternion endRot)
        {
            timestamp = time;
            basePosition = basePos;
            baseRotation = baseRot;
            shoulderPosition = shoulderPos;
            shoulderRotation = shoulderRot;
            elbowPosition = elbowPos;
            elbowRotation = elbowRot;
            wrist1Position = wrist1Pos;
            wrist1Rotation = wrist1Rot;
            wrist2Position = wrist2Pos;
            wrist2Rotation = wrist2Rot;
            endEffectorPosition = endPos;
            endEffectorRotation = endRot;
        }

        public override string ToString()
        {
            return $"{timestamp:F3},{basePosition.x:F6},{basePosition.y:F6},{basePosition.z:F6}," +
                   $"{baseRotation.x:F6},{baseRotation.y:F6},{baseRotation.z:F6},{baseRotation.w:F6}," +
                   $"{shoulderPosition.x:F6},{shoulderPosition.y:F6},{shoulderPosition.z:F6}," +
                   $"{shoulderRotation.x:F6},{shoulderRotation.y:F6},{shoulderRotation.z:F6},{shoulderRotation.w:F6}," +
                   $"{elbowPosition.x:F6},{elbowPosition.y:F6},{elbowPosition.z:F6}," +
                   $"{elbowRotation.x:F6},{elbowRotation.y:F6},{elbowRotation.z:F6},{elbowRotation.w:F6}," +
                   $"{wrist1Position.x:F6},{wrist1Position.y:F6},{wrist1Position.z:F6}," +
                   $"{wrist1Rotation.x:F6},{wrist1Rotation.y:F6},{wrist1Rotation.z:F6},{wrist1Rotation.w:F6}," +
                   $"{wrist2Position.x:F6},{wrist2Position.y:F6},{wrist2Position.z:F6}," +
                   $"{wrist2Rotation.x:F6},{wrist2Rotation.y:F6},{wrist2Rotation.z:F6},{wrist2Rotation.w:F6}," +
                   $"{endEffectorPosition.x:F6},{endEffectorPosition.y:F6},{endEffectorPosition.z:F6}," +
                   $"{endEffectorRotation.x:F6},{endEffectorRotation.y:F6},{endEffectorRotation.z:F6},{endEffectorRotation.w:F6}";
        }
    }

    [Header("Joint Control Settings")]
    public float jointStiffness = 10000f;
    public float jointDamping = 100f;
    public float forceLimit = 1000f;
    public float jointVelocityLimit = 3.15f;

    private void ConfigureArticulationBodies()
    {
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }

        if (robotJoints == null || robotJoints.Length == 0)
        {
            Debug.LogError("Robot joints not assigned!");
            return;
        }
    }

    public void ResetArmPosition()
    {
        for (int i = 0; i < stablePositions.Length; i++)
        {
            int idx = i + shoulderJointIdx;
            //Debug.Log($"{articulationChain[idx].name}");
            ArticulationDrive currentDrive = articulationChain[idx].xDrive;
            currentDrive.target = stablePositions[i] * Mathf.Rad2Deg;
            articulationChain[idx].xDrive = currentDrive;
        }
        Debug.Log("Arm reset to stable position");
    }

    private void Start()
    {
        isRecording = false;
        if (endEffectorTransform == null)
        {
            Debug.LogError("End effector transform not assigned to RobotArmController!");
            enabled = false;
            return;
        }

        coordinates = new List<CoordinateRecord>();
        SetupExportFile();

        // Configure the articulation bodies before setting positions
        ConfigureArticulationBodies();
        ResetArmPosition();
        isRecording = true;
    }

    // private void FixedUpdate()
    // {
    //     // Ensure joints maintain their positions
    //     if (robotJoints != null)
    //     {
    //         for (int i = 0; i < robotJoints.Length; i++)
    //         {
    //             var joint = robotJoints[i];
    //             if (joint.jointType != ArticulationJointType.FixedJoint)
    //             {
    //                 var drive = joint.xDrive;
    //                 if (Mathf.Abs(drive.target - joint.jointPosition[0]) > 0.1f)
    //                 {
    //                     joint.xDrive = drive;
    //                 }
    //             }
    //         }
    //     }
    // }

    private void Update()
    {
        // Handle coordinate recording in Update
        if (exportCoordinates && isRecording && Time.time >= nextSampleTime)
        {
            RecordCoordinates();
            nextSampleTime = Time.time + samplingRate;
        }
    }

    public void StartRecording()
    {
        if (isRecording) return;

        SetupExportFile();
        isRecording = true;
        nextSampleTime = Time.time;
        Debug.Log("Started recording joint coordinates.");
    }

    public void StopRecording()
    {
        if (!isRecording) return;

        isRecording = false;
        SaveCoordinates();
        coordinates.Clear();
        Debug.Log("Stopped recording joint coordinates.");
    }

    private void SetupExportFile()
    {
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string filename = appendTimestamp ? $"{exportFilePath}_{timestamp}.csv" : $"{exportFilePath}.csv";
        currentFilePath = Path.Combine(Application.dataPath, "..", "Exports", filename);

        // Create exports directory if it doesn't exist
        Directory.CreateDirectory(Path.GetDirectoryName(currentFilePath));

        // Write header
        using (StreamWriter writer = new StreamWriter(currentFilePath, false))
        {
            writer.WriteLine(coordinateHeaders);
        }
    }

    private void RecordCoordinates()
    {
        if (endEffectorTransform == null) return;

        CoordinateRecord record = new CoordinateRecord(
            Time.time,
            baseTransform.position,
            baseTransform.rotation,
            shoulderTransform.position,
            shoulderTransform.rotation,
            elbowTransform.position,
            elbowTransform.rotation,
            wrist1Transform.position,
            wrist1Transform.rotation,
            wrist2Transform.position,
            wrist2Transform.rotation,
            endEffectorTransform.position,
            endEffectorTransform.rotation
        );

        coordinates.Add(record);
        //Debug.Log($"Base: {record.basePosition} | Shoulder: {record.shoulderPosition} | Elbow: {record.elbowPosition} | wrist1: {record.wrist1Position} | wrist2: {record.wrist2Position} | End effector: {record.endEffectorPosition}, Count: {coordinates.Count}");

        // Save periodically to prevent data loss
        // if (coordinates.Count >= 100)
        // {
        //     SaveCoordinates();
        //     coordinates.Clear();
        // }
    }

    private void SaveCoordinates()
    {
        if (coordinates.Count == 0) return;

        try
        {
            using (StreamWriter writer = new StreamWriter(currentFilePath, true))
            {
                foreach (var record in coordinates)
                {
                    writer.WriteLine(record.ToString());
                }
            }
            Debug.Log($"Saved {coordinates.Count} coordinates to {currentFilePath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error saving coordinates: {e.Message}");
        }
    }

    private void OnDisable()
    {
        if (isRecording)
        {
            StopRecording();
        }
    }
    
}