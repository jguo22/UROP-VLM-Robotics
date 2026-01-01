using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;
using System.Text;

using static ConstantsUR5;
using static Config;

public class RobotArmSetup : MonoBehaviour
{
    // Coordinate export variables
    private string currentFilePath;
    private float nextSampleTime;
    private bool isRecording;
    private List<JointCoordinateRecord> coordinates;
    private readonly string coordinateHeaders = "Timestamp,base_PosX,base_PosY,base_PosZ,base_RotX,base_RotY,base_RotZ,base_RotW,base_jointAngle," +
                                              "shoulder_PosX,shoulder_PosY,shoulder_PosZ,shoulder_RotX,shoulder_RotY,shoulder_RotZ,shoulder_RotW,shoulder_jointAngle," +
                                              "elbow_PosX,elbow_PosY,elbow_PosZ,elbow_RotX,elbow_RotY,elbow_RotZ,elbow_RotW,elbow_jointAngle," +
                                              "wrist1_PosX,wrist1_PosY,wrist1_PosZ,wrist1_RotX,wrist1_RotY,wrist1_RotZ,wrist1_RotW,wrist1_jointAngle," +
                                              "wrist2_PosX,wrist2_PosY,wrist2_PosZ,wrist2_RotX,wrist2_RotY,wrist2_RotZ,wrist2_RotW,wrist2_jointAngle," +
                                              "wrist3_PosX,wrist3_PosY,wrist3_PosZ,wrist3_RotX,wrist3_RotY,wrist3_RotZ,wrist3_RotW,wrist3_jointAngle," +
                                              "ee_PosX,ee_PosY,ee_PosZ,ee_RotX,ee_RotY,ee_RotZ,ee_RotW,suctionOn,blockAttracted"; //+
                                                                                                                                  //"block_PosX,block_PosY,block_PosZ,block_RotX,block_RotY,block_RotZ,block_RotW";

    [HideInInspector]
    public ArticulationBody[] articulationChain; // Automatically populated articulation chain
    [HideInInspector]
    public ArticulationBody[] robotJoints;
    //private BlockSetup blockSetup;
    [HideInInspector]
    public GameObject blockToPickup;        // Reference to the block
    private SuctionController suctionController;

    [Header("Export Settings")]
    public bool exportCoordinates = false;
    public string exportFilePath = "RobotJointCoordinates";
    public bool appendTimestamp = true;
    public float samplingRate = 0.1f; // Time between samples in seconds

    private void ConfigureArticulationBodies()
    {
        suctionController = GetComponent<SuctionController>();
        if (suctionController == null)
        {
            Debug.LogError("SuctionController component not found on RobotArmSetup GameObject.");
            return;
        }

        //blockSetup = GetComponent<BlockSetup>();
        //blockToPickup = blockSetup.targetBlock;

        // if (blockToPickup == null)
        // {
        //     Debug.LogError("Block to pick up is not assigned in RobotArmSetup.");
        //     return;
        // }

        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        Debug.Log($"Found {articulationChain.Length} articulation bodies in the robot arm.");
        if (articulationChain.Length == 0)
        {
            Debug.LogError("No articulation bodies found in the robot arm hierarchy.");
            return;
        }

        foreach (ArticulationBody joint in articulationChain)
        {
            if (JointNames.ContainsKey(joint.name))
            {
                Debug.Log($"Adding JointControl to {JointNames[joint.name]}");
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
        for (int i = 0; i < JointCount; i++)
        {
            robotJoints[i].name = JointNames[robotJoints[i].name];
        }
        robotJoints[JointCount] = articulationChain[EndEffectorIndex];

    }

    public void ResetArmPosition()
    {
        for (int i = 0; i < JointCount; i++)
        {
            //int idx = i + shoulderJointIdx;
            //Debug.Log($"{articulationChain[idx].name}");
            ArticulationDrive currentDrive = robotJoints[i].xDrive;
            currentDrive.target = StableStartingRotations[i] * Mathf.Rad2Deg;
            robotJoints[i].xDrive = currentDrive;
        }
        Debug.Log("Arm reset to stable position");
    }

    private void Update()
    {
        if (exportCoordinates && isRecording && Time.time >= nextSampleTime)
        {
            RecordJointCoordinates();
            nextSampleTime = Time.time + samplingRate;
        }
    }

    // Structure to store joint coordinate data
    private struct JointCoordinateRecord
    {
        public float timestamp;
        public Vector3[] positions;
        public Quaternion[] rotations;
        public float[] jointAngles;
        public bool suctionState;
        public bool attractedState;

        public JointCoordinateRecord(float time, bool currentSuctionState, bool currentAttractedState)
        {
            timestamp = time;
            positions = new Vector3[7];  // 6 joints (indices 3-8) + ee
            rotations = new Quaternion[7];
            jointAngles = new float[JointCount];
            suctionState = currentSuctionState;
            attractedState = currentAttractedState;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append(timestamp.ToString("F3"));

            for (int i = 0; i < 7; i++)
            {
                switch (i)
                {
                    case 6: // End Effector
                        sb.Append($",{positions[i].x:F3},{positions[i].y:F3},{positions[i].z:F3}");
                        sb.Append($",{rotations[i].x:F3},{rotations[i].y:F3},{rotations[i].z:F3},{rotations[i].w:F3}");
                        sb.Append($",{suctionState}"); // Suction state for end effector
                        sb.Append($",{attractedState}"); // Block attracted state
                        break;
                    // case 7: // Block
                    //     sb.Append($",{positions[i].x:F3},{positions[i].y:F3},{positions[i].z:F3}");
                    //     sb.Append($",{rotations[i].x:F3},{rotations[i].y:F3},{rotations[i].z:F3},{rotations[i].w:F3}");
                    //     break;
                    default:
                        sb.Append($",{positions[i].x:F3},{positions[i].y:F3},{positions[i].z:F3}");
                        sb.Append($",{rotations[i].x:F3},{rotations[i].y:F3},{rotations[i].z:F3},{rotations[i].w:F3}");
                        sb.Append($",{jointAngles[i]:F3}");
                        break;
                }
            }

            return sb.ToString();
        }
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

        Debug.Log($"Created coordinate export file: {currentFilePath}");
    }

    private void RecordJointCoordinates()
    {
        JointCoordinateRecord record = new JointCoordinateRecord(Time.time, suctionController.enableSuction, suctionController.isBlockAttached);

        // Record joint positions and rotations for actual joints (0 to JointCount-1)
        for (int i = 0; i < JointCount; i++)
        {
            record.positions[i] = robotJoints[i].transform.position;
            record.rotations[i] = robotJoints[i].transform.rotation;
            record.jointAngles[i] = robotJoints[i].jointPosition[0] * Mathf.Rad2Deg;
        }

        // Record end effector position and rotation (no joint angle for end effector)
        record.positions[JointCount] = robotJoints[JointCount].transform.position;
        record.rotations[JointCount] = robotJoints[JointCount].transform.rotation;

        //record.positions[JointCount + 1] = blockToPickup.transform.position;
        //record.rotations[JointCount + 1] = blockToPickup.transform.rotation;

        coordinates.Add(record);

        //Save periodically to prevent data loss
        if (coordinates.Count >= MaxStoreCoordinates)
        {
            SaveCoordinates();
            coordinates.Clear();
        }
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
            Debug.Log($"Saved {coordinates.Count} coordinate records to file");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error saving coordinates: {e.Message}");
        }
    }

    public void StartRecording()
    {
        if (isRecording) return;

        SetupExportFile();
        isRecording = true;
        nextSampleTime = Time.time;
        Debug.Log("Started recording joint coordinates");
    }

    public void StopRecording()
    {
        if (!isRecording) return;

        SaveCoordinates();
        coordinates.Clear();
        isRecording = false;
        Debug.Log("Stopped recording joint coordinates");
    }

    private void OnDisable()
    {
        if (isRecording)
        {
            StopRecording();
        }
    }

    private void Start()
    {
        coordinates = new List<JointCoordinateRecord>();
        ConfigureArticulationBodies();
        ResetArmPosition();

        exportFilePath = "RobotJointCoordinates_" + this.gameObject.name;

        if (exportCoordinates)
        {
            StartRecording();
        }
    }
}
