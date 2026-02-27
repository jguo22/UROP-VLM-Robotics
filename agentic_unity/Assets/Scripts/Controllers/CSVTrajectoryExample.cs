using UnityEngine;
using System.IO;
using System.Collections.Generic;

using static ConstantsUR5;

/// <summary>
/// Example script demonstrating how to use CSV trajectory functionality
/// This script shows how to load and control CSV trajectories in Unity
/// </summary>
public class CSVTrajectoryExample : MonoBehaviour
{
    [HideInInspector]
    public UnifiedRobotController robotController;
    [HideInInspector]
    public CSVTrajectoryController csvController;

    [Header("CSV Files")]
    [Tooltip("List of CSV files found in Exports folder (auto-populated)")]
    public List<string> csvFileNames = new List<string>();
    public int selectedFileIndex = 0;

    [Header("Demo Settings")]
    public bool showDemoGUI = true;
    public string customFilePath = "";

    private string defaultSampleFile = "SampleTrajectory_Test3.csv";

    private void Start()
    {
        InitializeComponents();

        // Auto-populate CSV files from Exports folder
        PopulateCSVFiles();
    }

    void InitializeComponents()
    {
        if (robotController == null)
            //robotController = FindObjectOfType<UnifiedRobotController>();
            robotController = GetComponent<UnifiedRobotController>();

        if (csvController == null)
            //csvController = FindObjectOfType<CSVTrajectoryController>();
            csvController = GetComponent<CSVTrajectoryController>();
    }

    void PopulateCSVFiles()
    {
        // Clear previous list
        csvFileNames.Clear();

        // Define search folders (both Exports and Imports)
        string[] searchFolders = new string[]
        {
            // Path.Combine(Application.dataPath, "..", "Exports"),
            Path.Combine(Application.dataPath, "..", "Imports")
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
                    // Add folder prefix to distinguish files from different folders
                    string folderName = Path.GetFileName(folderPath);
                    string displayName = $"[{folderName}] {fileName}";
                    csvFileNames.Add(displayName);
                    Debug.Log($"Found CSV file: {displayName}");
                    totalFilesFound++;
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
        // Demo controls
        if (Input.GetKeyDown(KeyCode.F1))
        {
            showDemoGUI = !showDemoGUI;
            Debug.Log($"Demo GUI: {(showDemoGUI ? "shown" : "hidden")}");
        }

        if (Input.GetKeyDown(KeyCode.F2))
        {
            LoadSelectedTrajectory();
        }

        if (Input.GetKeyDown(KeyCode.F3))
        {
            LoadCustomTrajectory();
        }

        // Debug key for troubleshooting
        if (Input.GetKeyDown(KeyCode.F12))
        {
            DebugCSVSystem();
        }

        // Quick test keys for debugging
        if (Input.GetKeyDown(KeyCode.F9))
        {
            ForceLoadSampleTrajectory();
        }

        if (Input.GetKeyDown(KeyCode.F10))
        {
            TestFilePaths();
        }

        if (Input.GetKeyDown(KeyCode.F11))
        {
            DebugRobotState();
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
        Debug.Log($"CSV Controller: {(csvController != null ? "Found" : "MISSING")}");

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
        if (csvController != null)
        {
            Debug.Log($"Trajectory Loaded: {(csvController.totalFrames > 0 ? "YES" : "NO")}");
            Debug.Log($"Frames: {csvController.totalFrames}, Duration: {csvController.totalDuration:F2}s");
            Debug.Log($"Playback Status: {(csvController.isPlaying ? (csvController.isPaused ? "Paused" : "Playing") : "Stopped")}");
        }

        // Check file paths for both folders
        string[] folderNames = { "Exports", "Imports" };
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
        string samplePath = Path.Combine(Application.dataPath, "..", "Imports", defaultSampleFile);

        Debug.Log($"=== FORCE LOADING SAMPLE TRAJECTORY ===");
        Debug.Log($"Sample path: {samplePath}");
        Debug.Log($"File exists: {File.Exists(samplePath)}");

        if (robotController != null)
        {
            robotController.LoadCSVTrajectory(samplePath);
            Debug.Log("Force load command sent to robot controller");
        }
        else
        {
            Debug.LogError("Robot controller is null!");
        }
    }

    /// <summary>
    /// Test file path resolution
    /// </summary>
    void TestFilePaths()
    {
        Debug.Log($"=== FILE PATH TEST ===");
        Debug.Log($"Application.dataPath: {Application.dataPath}");

        string exportsPath = Path.Combine(Application.dataPath, "..", "Exports");
        string importsPath = Path.Combine(Application.dataPath, "..", "Imports");
        string samplePath = Path.Combine(importsPath, defaultSampleFile);

        Debug.Log($"Exports path: {exportsPath}");
        Debug.Log($"Imports path: {importsPath}");
        Debug.Log($"Sample file path: {samplePath}");

        Debug.Log($"Exports folder exists: {Directory.Exists(exportsPath)}");
        Debug.Log($"Imports folder exists: {Directory.Exists(importsPath)}");
        Debug.Log($"Sample file exists: {File.Exists(samplePath)}");

        if (Directory.Exists(importsPath))
        {
            string[] files = Directory.GetFiles(importsPath, "*.csv");
            Debug.Log($"CSV files in Imports: {files.Length}");
            foreach (string file in files)
            {
                Debug.Log($"  - {Path.GetFileName(file)}");
            }
        }
    }

    /// <summary>
    /// Debug method to check robot state and joint setup
    /// </summary>
    void DebugRobotState()
    {
        Debug.Log("=== ROBOT STATE DEBUG ===");

        // Check UnifiedRobotController
        if (robotController == null)
        {
            Debug.LogError("UnifiedRobotController is null!");
            return;
        }

        Debug.Log("✓ UnifiedRobotController found");

        // Check robot arm setup
        if (robotController.robotArmSetup == null)
        {
            Debug.LogError("robotArmSetup is null!");
        }
        else
        {
            Debug.Log("✓ robotArmSetup found");
            //Debug.Log($"  - Shoulder joint index: {robotController.robotArmSetup.shoulderJointIdx}");
            Debug.Log($"  - Stable positions length: {StableStartingRotations.Length}");

            if (robotController.robotArmSetup.articulationChain != null)
            {
                Debug.Log($"  - Articulation chain length: {robotController.robotArmSetup.articulationChain.Length}");
                for (int i = 0; i < robotController.robotArmSetup.articulationChain.Length; i++)
                {
                    var joint = robotController.robotArmSetup.articulationChain[i];
                    Debug.Log($"    Joint {i}: {joint.name} - Type: {joint.jointType}");
                }
            }
            else
            {
                Debug.LogError("  - Articulation chain is null!");
            }
        }

        // Check joints array
        if (robotController.Joints == null)
        {
            Debug.LogError("joints array is null!");
        }
        else
        {
            Debug.Log($"✓ joints array found, length: {robotController.Joints.Length}");
            for (int i = 0; i < robotController.Joints.Length; i++)
            {
                var joint = robotController.Joints[i];
                if (joint != null)
                {
                    Debug.Log($"  Joint {i}: {joint.name} - Type: {joint.jointType}");
                }
                else
                {
                    Debug.LogError($"  Joint {i} is null!");
                }
            }
        }

        // Check CSVTrajectoryController
        if (csvController == null)
        {
            Debug.LogError("CSVTrajectoryController is null!");
        }
        else
        {
            Debug.Log("✓ CSVTrajectoryController found");
            Debug.Log($"  - Total frames: {csvController.totalFrames}");
            Debug.Log($"  - Is playing: {csvController.isPlaying}");
            Debug.Log($"  - Is paused: {csvController.isPaused}");
            Debug.Log($"  - Current frame: {csvController.currentFrame}");
            Debug.Log($"  - Playback speed: {csvController.playbackSpeed}");

            if (csvController.robotController == null)
            {
                Debug.LogError("  - CSV controller's robotController reference is null!");
            }
            else
            {
                Debug.Log("  - CSV controller has robotController reference");
            }
        }

        // Check current control mode
        Debug.Log($"Current control mode: {robotController.currentMode}");

        // Test joint movement
        Debug.Log("=== TESTING JOINT MOVEMENT ===");
        if (robotController != null && robotController.Joints != null && robotController.Joints.Length > 0)
        {
            float[] testAngles = new float[] { 0.5f, 0.3f, -0.2f, 0.8f, -0.4f, 0.1f };
            Debug.Log($"Sending test angles: [{string.Join(", ", testAngles)}]");
            robotController.SetJointAngles(testAngles);
        }
        else
        {
            Debug.LogError("Cannot test joint movement - joints not properly set up!");
        }
    }

    void OnGUI()
    {
        if (!showDemoGUI) return;

        GUIStyle style = new GUIStyle(GUI.skin.button);
        style.fontSize = 12;

        int panelWidth = 300;
        int panelHeight = 400;
        int startX = Screen.width - panelWidth - 10;
        int startY = 100;

        // Background panel
        GUI.Box(new Rect(startX - 5, startY - 5, panelWidth + 10, panelHeight + 10), "CSV Trajectory Demo");

        int currentY = startY;
        int buttonHeight = 30;
        int spacing = 5;

        // Title
        //GUI.Label(new Rect(startX, currentY, panelWidth, 20),
        //    "CSV Trajectory Controller", new GUIStyle(GUI.skin.label) { fontSize = 14, fontStyle = FontStyle.Bold });
        currentY += 25;

        // File selection
        GUI.Label(new Rect(startX, currentY, panelWidth, 20), "Available CSV Files:");
        currentY += 25;

        if (csvFileNames.Count > 0)
        {
            string[] fileNames = csvFileNames.ToArray();

            selectedFileIndex = GUI.SelectionGrid(new Rect(startX, currentY, panelWidth, 120),
                selectedFileIndex, fileNames, 1);
            currentY += 130;

            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Load Selected Trajectory"))
            {
                LoadSelectedTrajectory();
            }
            currentY += buttonHeight + spacing;

            // Show selected file info
            if (selectedFileIndex >= 0 && selectedFileIndex < csvFileNames.Count)
            {
                GUI.Label(new Rect(startX, currentY, panelWidth, 20),
                    $"Selected: {csvFileNames[selectedFileIndex]}", GUI.skin.label);
                currentY += 25;
            }
        }
        else
        {
            GUI.Label(new Rect(startX, currentY, panelWidth, 40),
                "No CSV files found.\nPlace CSV files in Exports folder\nor use custom path below.");
            currentY += 50;

            // Add a refresh button
            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Refresh CSV Files"))
            {
                PopulateCSVFiles();
            }
            currentY += buttonHeight + spacing;
        }

        // Custom file path
        //GUI.Label(new Rect(startX, currentY, panelWidth, 20), "Custom File Path:");
        //currentY += 25;

        //customFilePath = GUI.TextField(new Rect(startX, currentY, panelWidth, 25), customFilePath);
        //currentY += 30;

        //if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Load Custom Trajectory"))
        //{
        //    LoadCustomTrajectory();
        //}
        //currentY += buttonHeight + spacing;

        // Playback controls
        GUI.Label(new Rect(startX, currentY, panelWidth, 20), "Playback Controls:",
            new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        currentY += 25;

        // Play/Pause button
        string firstPlayButtonText = "Play";
        if (csvController != null)
        {
            if (csvController.isPlaying)
            {
                firstPlayButtonText = csvController.isPaused ? "Resume" : "Pause";
            }
        }

        if (GUI.Button(new Rect(startX, currentY, panelWidth / 2 - 2, buttonHeight), firstPlayButtonText))
        {
            if (robotController != null)
            {
                if (csvController != null && csvController.isPlaying)
                {
                    robotController.PauseCSVPlayback();
                }
                else
                {
                    robotController.StartCSVPlayback();
                }
            }
        }

        if (GUI.Button(new Rect(startX + panelWidth / 2 + 2, currentY, panelWidth / 2 - 2, buttonHeight), "Stop"))
        {
            if (robotController != null)
            {
                robotController.StopCSVPlayback();
            }
        }
        currentY += buttonHeight + spacing;

        // Speed controls
        if (csvController != null)
        {
            // Trajectory status
            string statusText = "No trajectory loaded";
            if (csvController.totalFrames > 0)
            {
                statusText = $"Loaded: {csvController.totalFrames} frames, {csvController.totalDuration:F2}s";
            }
            GUI.Label(new Rect(startX, currentY, panelWidth, 20), statusText);
            currentY += 25;

            GUI.Label(new Rect(startX, currentY, panelWidth, 20),
                $"Speed: {csvController.playbackSpeed:F1}x");
            currentY += 25;

            if (GUI.Button(new Rect(startX, currentY, panelWidth / 2 - 2, buttonHeight), "Slower"))
            {
                robotController.SetCSVPlaybackSpeed(csvController.playbackSpeed / 1.2f);
            }

            if (GUI.Button(new Rect(startX + panelWidth / 2 + 2, currentY, panelWidth / 2 - 2, buttonHeight), "Faster"))
            {
                robotController.SetCSVPlaybackSpeed(csvController.playbackSpeed * 1.2f);
            }
            currentY += buttonHeight + spacing;

            // Test joint movement button
            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Test Joint Movement (T)"))
            {
                if (csvController != null)
                {
                    // Access the private TestJointMovement method via reflection
                    var method = csvController.GetType().GetMethod("TestJointMovement",
                        System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                    if (method != null)
                    {
                        method.Invoke(csvController, null);
                    }
                }
            }
            currentY += buttonHeight + spacing;

            // Play/Pause button
            string secondPlayButtonText = "Play Trajectory";
            if (csvController != null)
            {
                if (csvController.isPlaying)
                {
                    secondPlayButtonText = csvController.isPaused ? "Resume Trajectory" : "Pause Trajectory";
                }
            }

            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), secondPlayButtonText))
            {
                if (csvController != null)
                {
                    if (csvController.isPlaying)
                    {
                        csvController.PausePlayback();
                    }
                    else
                    {
                        csvController.StartPlayback();
                    }
                }
            }
            currentY += buttonHeight + spacing;

            // Stop button
            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Stop Trajectory"))
            {
                if (csvController != null)
                {
                    csvController.StopPlayback();
                }
            }
            currentY += buttonHeight + spacing;

            // Debug robot state button
            if (GUI.Button(new Rect(startX, currentY, panelWidth, buttonHeight), "Debug Robot State (F11)"))
            {
                DebugRobotState();
            }
            currentY += buttonHeight + spacing;

            // Loop toggle
            bool loopTrajectory = csvController.loopTrajectory;
            csvController.loopTrajectory = GUI.Toggle(new Rect(startX, currentY, panelWidth, 20),
                csvController.loopTrajectory, "Loop Trajectory");
            currentY += 25;

            // Progress info
            if (csvController.totalFrames > 0)
            {
                float progress = robotController.GetCSVProgress();
                GUI.Label(new Rect(startX, currentY, panelWidth, 20),
                    $"Progress: {progress:P1} ({csvController.currentFrame}/{csvController.totalFrames})");
                currentY += 25;

                // Progress bar
                GUI.Box(new Rect(startX, currentY, panelWidth, 10), "");
                GUI.Box(new Rect(startX, currentY, panelWidth * progress, 10), "");
                currentY += 15;
            }
        }
        else
        {
            GUI.Label(new Rect(startX, currentY, panelWidth, 20), "CSV Controller not found!", GUI.skin.label);
            currentY += 25;
        }

        // Instructions
        currentY += 10;
        GUI.Label(new Rect(startX, currentY, panelWidth, 20), "Hotkeys:",
            new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold });
        currentY += 20;

        GUI.Label(new Rect(startX, currentY, panelWidth, 125),
            "F1: Toggle Demo GUI\n" +
            "F2: Load Selected File\n" +
            "F3: Load Custom File\n" +
            "F9: Force Load Sample\n" +
            "F10: Test File Paths\n" +
            "F12: Full Debug\n" +
            "T: Test Joint Movement\n" +
            "Space: Play/Pause\n" +
            "S: Stop\n" +
            "+/-: Speed Control\n" +
            "5: Switch to CSV Mode\n" +
            "F11: Debug Robot State");

        currentY += 130;

        // Debug info
        GUI.Label(new Rect(startX, currentY, panelWidth, 20),
            $"Files found: {csvFileNames.Count}", GUI.skin.label);
    }

    void LoadSelectedTrajectory()
    {
        if (csvFileNames.Count == 0 || selectedFileIndex >= csvFileNames.Count)
        {
            Debug.LogWarning("No CSV file selected or available");
            return;
        }

        string selectedDisplayName = csvFileNames[selectedFileIndex];

        // Parse the folder and filename from the display name
        string folderName = "";
        string fileName = selectedDisplayName;

        if (selectedDisplayName.StartsWith("[") && selectedDisplayName.Contains("]"))
        {
            int bracketEnd = selectedDisplayName.IndexOf("]");
            folderName = selectedDisplayName.Substring(1, bracketEnd - 1);
            fileName = selectedDisplayName.Substring(bracketEnd + 2); // Skip "] "
        }

        // Determine the correct folder path
        string folderPath;
        if (folderName == "Imports")
        {
            folderPath = Path.Combine(Application.dataPath, "..", "Imports");
        }
        else if (folderName == "Exports")
        {
            folderPath = Path.Combine(Application.dataPath, "..", "Exports");
        }
        else
        {
            // Default to Exports if no folder specified
            folderPath = Path.Combine(Application.dataPath, "..", "Exports");
            Debug.LogWarning($"Unknown folder '{folderName}', defaulting to Exports");
        }

        string fullPath = Path.Combine(folderPath, fileName);

        if (robotController != null)
        {
            robotController.LoadCSVTrajectory(fullPath);
            Debug.Log($"Loaded trajectory: {fileName}");
            Debug.Log($"From folder: {folderName}");
            Debug.Log($"Full path: {fullPath}");
        }
        else
        {
            Debug.LogError("Robot controller not found!");
        }
    }

    void LoadCustomTrajectory()
    {
        if (string.IsNullOrEmpty(customFilePath))
        {
            Debug.LogWarning("Custom file path is empty");
            return;
        }

        if (robotController != null)
        {
            robotController.LoadCSVTrajectory(customFilePath);
            Debug.Log($"Loaded custom trajectory: {customFilePath}");
        }
    }

    #region Public API Methods

    /// <summary>
    /// Load a specific trajectory by name
    /// </summary>
    public void LoadTrajectoryByName(string trajectoryName)
    {
        int foundIndex = csvFileNames.FindIndex(name => name.Contains(trajectoryName));

        if (foundIndex >= 0)
        {
            selectedFileIndex = foundIndex;
            LoadSelectedTrajectory();
        }
        else
        {
            Debug.LogWarning($"Trajectory '{trajectoryName}' not found");
        }
    }

    /// <summary>
    /// Refresh the list of available CSV files
    /// </summary>
    public void RefreshCSVFiles()
    {
        PopulateCSVFiles();
        Debug.Log("CSV files list refreshed");
    }

    /// <summary>
    /// Get the current loaded trajectory status
    /// </summary>
    public string GetTrajectoryStatus()
    {
        if (csvController == null) return "CSV Controller not found";

        string status = $"Frames: {csvController.totalFrames}, Duration: {csvController.totalDuration:F2}s";
        status += $"\nSpeed: {csvController.playbackSpeed:F1}x";
        status += $"\nStatus: {(csvController.isPlaying ? (csvController.isPaused ? "Paused" : "Playing") : "Stopped")}";

        return status;
    }

    /// <summary>
    /// Start playback of the current trajectory
    /// </summary>
    public void StartTrajectoryPlayback()
    {
        if (robotController != null)
        {
            robotController.StartCSVPlayback();
        }
    }

    /// <summary>
    /// Stop playback of the current trajectory
    /// </summary>
    public void StopTrajectoryPlayback()
    {
        if (robotController != null)
        {
            robotController.StopCSVPlayback();
        }
    }

    /// <summary>
    /// Set playback speed
    /// </summary>
    public void SetPlaybackSpeed(float speed)
    {
        if (robotController != null)
        {
            robotController.SetCSVPlaybackSpeed(speed);
        }
    }

    #endregion
}

