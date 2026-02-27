using UnityEngine;
using System.IO;
using System.Collections.Generic;

using static ConstantsUR5;

/// <summary>
/// Example script demonstrating how to use CSV trajectory functionality
/// This script shows how to load and control CSV trajectories in Unity
/// </summary>
/// 

[DefaultExecutionOrder(-1)] // Ensure this runs after AgentRobotController
public class AgentTrajectory : MonoBehaviour
{
    [HideInInspector]
    public AgentRobotController robotController;
    [HideInInspector]
    public AgentTrajectoryController agentController;

    [Header("CSV Files")]
    [Tooltip("List of CSV files found in Exports folder (auto-populated)")]
    public List<string> csvFileNames = new List<string>();
    public int selectedFileIndex = 0;

    [Header("Demo Settings")]
    public bool showDemoGUI = true;
    public string customFilePath = "";

    [HideInInspector]
    public GameObject[] robots;

    //[HideInInspector]
    //public string defaultSampleFile; //"SampleTrajectory_Test3.csv";

    private void Start()
    {
        if (robots == null)
        {
            Debug.LogError("AgentTrajectory: Robots not assigned");
            return;
        }

        InitializeComponents();

        // Auto-populate CSV files from Exports folder
        PopulateCSVFiles();
    }

    void InitializeComponents()
    {
        if (robotController == null)
            //robotController = FindObjectOfType<UnifiedRobotController>();
            robotController = GetComponent<AgentRobotController>();

        if (agentController == null)
            //csvController = FindObjectOfType<AgentTrajectoryController>();
            agentController = GetComponent<AgentTrajectoryController>();

    }


    void PopulateCSVFiles()
    {
        // Clear previous list
        csvFileNames.Clear();

        // Define search folders (both Exports and Imports)
        string[] searchFolders = new string[]
        {
            Path.Combine(Application.dataPath, "..", "AgenticAIPaths")
        };

        int totalFilesFound = 0;

        foreach (string folderPath in searchFolders)
        {
            if (Directory.Exists(folderPath))
            {
                string[] csvFilesInFolder = Directory.GetFiles(folderPath, "*.csv");

                foreach (string csvFile in csvFilesInFolder)
                {
                    string fileName = Path.GetFileName(csvFile);

                    foreach (GameObject robot in robots)
                    {
                        if (fileName == (robot.name + ".csv"))
                        {
                            // Add folder prefix to distinguish files from different folders
                            string folderName = Path.GetFileName(folderPath);
                            string displayName = $"[{folderName}] {fileName}";
                            csvFileNames.Add(displayName);
                            Debug.Log($"Found CSV file: {displayName}");
                            totalFilesFound++;
                            break;
                        }
                    }
                }
            }
            else
            {
                Debug.Log($"Folder not found: {folderPath}");
            }
        }

        Debug.Log($"Total CSV files found across all folders: {totalFilesFound}");

        if (totalFilesFound == 0)
        {
            Debug.LogWarning("No CSV files found in Exports or Imports folders!");
            Debug.Log("Create CSV files in: ur5_simulation/Exports/ or ur5_simulation/Imports/");
        }
    }

    void Update()
    {
        
        if (Input.GetKeyDown(KeyCode.F1))
        {
            ForceLoadSampleTrajectory();
        }

    }

    /// <summary>
    /// Debug method to check CSV system status
    /// </summary>
    void DebugCSVSystem()
    {
        Debug.Log("=== CSV Trajectory System Debug ===");

        // Check references
        Debug.Log($"Robot Controller: {(robotController != null ? "Found" : "MISSING")}");
        Debug.Log($"CSV Controller: {(agentController != null ? "Found" : "MISSING")}");

        // Check CSV files
        Debug.Log($"CSV Files Count: {csvFileNames.Count}");
        for (int i = 0; i < csvFileNames.Count; i++)
        {
            Debug.Log($"  [{i}] {csvFileNames[i]}");
        }

        // Check selected file
        if (selectedFileIndex >= 0 && selectedFileIndex < csvFileNames.Count)
        {
            Debug.Log($"Selected File: {csvFileNames[selectedFileIndex]}");
        }
        else
        {
            Debug.Log($"Selected File Index: {selectedFileIndex} (INVALID)");
        }

        // Check trajectory status
        if (agentController != null)
        {
            Debug.Log($"Trajectory Loaded: {(agentController.totalFrames > 0 ? "YES" : "NO")}");
            Debug.Log($"Frames: {agentController.totalFrames}, Duration: {agentController.totalDuration:F2}s");
            Debug.Log($"Playback Status: {(agentController.isPlaying ? (agentController.isPaused ? "Paused" : "Playing") : "Stopped")}");
        }

        // Check file paths for both folders
        string[] folderNames = { "AgenticAIPaths" };
        foreach (string folder in folderNames)
        {
            string folderPath = Path.Combine(Application.dataPath, "..", folder);
            Debug.Log($"{folder} Path: {folderPath}");
            Debug.Log($"{folder} Exists: {Directory.Exists(folderPath)}");

            if (Directory.Exists(folderPath))
            {
                string[] files = Directory.GetFiles(folderPath, "*.csv");
                Debug.Log($"CSV Files in {folder}: {files.Length}");
                foreach (string file in files)
                {
                    Debug.Log($"  - {Path.GetFileName(file)}");
                }
            }
        }
    }

    /// <summary>
    /// Force load the sample trajectory directly
    /// </summary>
    void ForceLoadSampleTrajectory()
    {
        string[] samplePaths = new string[robots.Length];
        int idx = 0;
        foreach (GameObject robot in robots)
        {
            string defaultFile = robot.name + ".csv";
            string samplePath = Path.Combine(Application.dataPath, "..", "AgenticAIPaths", defaultFile);

            Debug.Log($"=== FORCE LOADING SAMPLE TRAJECTORY ===");
            Debug.Log($"Sample path: {samplePath}");
            Debug.Log($"File exists: {File.Exists(samplePath)}");

            samplePaths[idx] = samplePath;
            idx += 1;
        }

        if (robotController != null)
        {
            robotController.LoadCSVTrajectory(samplePaths);
            Debug.Log("Force load command sent to robot controller");
        }
        else
        {
            Debug.LogError("Robot controller is null!");
        }
    }
    
}
