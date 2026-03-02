using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Robotics.UrdfImporter.Control;
using UnityEngine;
using static Config;
using static ConstantsUR5;

/// <summary>
/// CSV Trajectory Controller - Loads and executes robot trajectories from CSV files
/// Compatible with exported RobotJointCoordinates CSV format
/// </summary>
public class AgentTrajectoryController : MonoBehaviour
{
    [HideInInspector]
    public AgentRobotController robotController;

    [HideInInspector]
    public GameObject[] robots;

    [Header("Trajectory Settings")]
    public TextAsset csvFile; // Drag and drop CSV file in Inspector
    public string[] csvFilePaths;
    public string csvFilePath = ""; // Alternative: specify file path
    public float playbackSpeed = DefaultPlaybackSpeed;
    public bool loopTrajectory = false;
    public bool autoStart = false;

    [Header("Playback Control")]
    public bool isPlaying = false;
    public bool isPaused = false;
    public float currentTime = 0f;
    public int currentFrame = 0;

    [Header("Trajectory Data")]
    [Tooltip("Total number of frames in the loaded trajectory")]
    public int totalFrames = 0;

    [Tooltip("Total duration of the trajectory in seconds")]
    public float totalDuration = 0f;

    [Tooltip("Current playback time position")]
    public float currentPlaybackTime = 0f;

    [HideInInspector]
    public Dictionary<float, StoredTrajectoryModel>[] robotTrajectories; //key timestamp, value is StoredTrajectoryModel

    [HideInInspector]
    public List<float> timestamps;

    // Internal data structures
    //private List<float> timestamps = new List<float>();
    //private List<bool> suctionStates = new List<bool>();
    //private List<bool> attractedStates = new List<bool>();
    //private List<float[]> jointAnglesTrajectory = new List<float[]>(); // 6 joints per frame
    //private List<Vector3[]> jointPositionsTrajectory = new List<Vector3[]>(); // For IK fallback
    //private List<Quaternion[]> jointRotationsTrajectory = new List<Quaternion[]>(); // For IK fallback

    // Playback coroutine
    private Coroutine playbackCoroutine;

    #region Unity Methods

    void Start()
    {
        InitializeController();

        if (autoStart && (csvFile != null || !string.IsNullOrEmpty(csvFilePath)))
        {
            LoadTrajectory();
            StartPlayback();
        }
    }

    void Update()
    {
        HandleInput();

        // Update playback time for UI
        if (isPlaying && !isPaused)
        {
            currentPlaybackTime = currentTime;
        }
    }

    void OnGUI()
    {
        DrawTrajectoryGUI();
    }

    #endregion

    #region Initialization

    void InitializeController()
    {
        if (robotController == null)
        {
            robotController = GetComponent<AgentRobotController>();
        }

        if (robotController == null)
        {
            Debug.LogError("AgentTrajectoryController: No AgentRobotController found!");
        }
    }

    #endregion

    #region CSV Loading and Parsing

    /// <summary>
    /// Load trajectory from CSV file
    /// </summary>
    public bool LoadTrajectory()
    {
        string[] csvContents = new string[robots.Length];
        int idx = 0;

        // Try to load from file path
        foreach (string csvFilePath in csvFilePaths)
        {
            if (!string.IsNullOrEmpty(csvFilePath))
            {
                try
                {
                    string csvContent = File.ReadAllText(csvFilePath);
                    csvContents[idx] = csvContent;
                    idx++;
                }
                catch (System.Exception e)
                {
                    Debug.LogError(
                        $"Failed to load CSV file from path: {csvFilePath}\n{e.Message}"
                    );
                    return false;
                }
            }
            else
            {
                Debug.LogError("No CSV file specified. Set csvFile or csvFilePath.");
                return false;
            }
        }

        return ParseCSVContent(csvContents);
    }

    /// <summary>
    /// Parse CSV content into trajectory data
    /// </summary>
    private bool ParseCSVContent(string[] csvContents)
    {
        int robotCount = robots.Length;
        robotTrajectories = new Dictionary<float, StoredTrajectoryModel>[robotCount];
        List<float>[] arrayOfTimestamps = new List<float>[robotCount];

        try
        {
            // Clear previous data
            //ClearTrajectoryData();

            for (int idx = 0; idx < robotCount; idx++)
            {
                Dictionary<float, StoredTrajectoryModel> robotTrajectory =
                    new Dictionary<float, StoredTrajectoryModel>();
                List<float> robotTimestamps = new List<float>();

                string csvContent = csvContents[idx];
                string[] lines = csvContent.Split('\n');

                if (lines.Length < 2)
                {
                    Debug.LogError("CSV file must have at least header and one data row");
                    return false;
                }

                // Parse header to understand column structure
                string[] headers = lines[0].Split(',').Select(h => h.Trim()).ToArray();

                // Find joint angle columns (we're looking for joint position data)
                // The CSV has PosX, PosY, PosZ for each joint, but we need to convert to joint angles
                Dictionary<string, int> jointColumnMap = new Dictionary<string, int>();

                // Map joint names to their position column indices
                string[] jointNames = { "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" };
                foreach (string jointName in jointNames)
                {
                    string posXCol = $"{jointName}_PosX";
                    string posYCol = $"{jointName}_PosY";
                    string posZCol = $"{jointName}_PosZ";
                    string rotXCol = $"{jointName}_RotX";
                    string rotYCol = $"{jointName}_RotY";
                    string rotZCol = $"{jointName}_RotZ";
                    string rotWCol = $"{jointName}_RotW";
                    string jointAngleCol = $"{jointName}_jointAngle";

                    if (Array.IndexOf(headers, posXCol) >= 0)
                    {
                        jointColumnMap[jointName] = Array.IndexOf(headers, posXCol);
                    }
                }

                string suctionCol = "suctionOn";
                int suctionColIndex = Array.IndexOf(headers, suctionCol);

                string attractedCol = "blockAttracted";
                int attractedColIndex = Array.IndexOf(headers, attractedCol);

                // Find timestamp column
                int timestampCol = Array.IndexOf(headers, "Timestamp");

                if (timestampCol < 0)
                {
                    Debug.LogError("Timestamp column not found in CSV");
                    return false;
                }

                // Parse data rows
                for (int i = 1; i < lines.Length; i++)
                {
                    if (string.IsNullOrWhiteSpace(lines[i]))
                        continue;

                    string[] values = lines[i].Split(',');

                    if (values.Length < headers.Length)
                        continue;

                    // Parse timestamp
                    if (float.TryParse(values[timestampCol], out float timestamp))
                    {
                        robotTimestamps.Add(timestamp);
                    }
                    else
                        continue; // Skip invalid rows

                    // Parse joint positions and rotations
                    Vector3[] positions = new Vector3[6]; // 6 joints
                    Quaternion[] rotations = new Quaternion[6];
                    float[] jointAngles = new float[6];

                    // For each joint, extract position and rotation
                    for (int j = 0; j < jointNames.Length; j++)
                    {
                        string jointName = jointNames[j];
                        if (jointColumnMap.ContainsKey(jointName))
                        {
                            int baseCol = jointColumnMap[jointName];

                            // Parse position
                            if (baseCol + 2 < values.Length)
                            {
                                float.TryParse(values[baseCol], out positions[j].x);
                                float.TryParse(values[baseCol + 1], out positions[j].y);
                                float.TryParse(values[baseCol + 2], out positions[j].z);
                            }

                            // Parse rotation (quaternion)
                            if (baseCol + 6 < values.Length)
                            {
                                float.TryParse(values[baseCol + 3], out rotations[j].x);
                                float.TryParse(values[baseCol + 4], out rotations[j].y);
                                float.TryParse(values[baseCol + 5], out rotations[j].z);
                                float.TryParse(values[baseCol + 6], out rotations[j].w);
                            }

                            if (baseCol + 7 < values.Length)
                            {
                                float.TryParse(values[baseCol + 7], out jointAngles[j]);
                            }
                        }
                    }

                    bool currentSuctionState =
                        suctionColIndex >= 0
                        && suctionColIndex < values.Length
                        && values[suctionColIndex].Trim() == "True";
                    bool currentAttractedState =
                        attractedColIndex >= 0
                        && attractedColIndex < values.Length
                        && values[attractedColIndex].Trim() == "True";

                    StoredTrajectoryModel storedTrajectory = new StoredTrajectoryModel(
                        jointAngles,
                        currentSuctionState,
                        currentAttractedState
                    );
                    robotTrajectory.Add(timestamp, storedTrajectory);
                }
                //---end of for loop---

                robotTrajectories[idx] = robotTrajectory;
                arrayOfTimestamps[idx] = robotTimestamps;

                // totalFrames = jointAnglesTrajectory.Count;
                // if (timestamps.Count > 1)
                // {
                //     totalDuration = timestamps[timestamps.Count - 1] - timestamps[0];
                // }

                // Debug.Log($"Successfully loaded {robots[idx].name} trajectory: {totalFrames} frames, {totalDuration:F2} seconds");
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error parsing CSV: {e.Message}");
            return false;
        }

        timestamps = arrayOfTimestamps[0];
        for (int i = 1; i < robotCount; i++)
            timestamps = timestamps.Union(arrayOfTimestamps[i]).ToList();
        timestamps.Sort();

        if (timestamps.Count > 1)
            totalDuration = timestamps[timestamps.Count - 1] - timestamps[0];

        totalFrames = timestamps.Count;

        Debug.Log(
            $"Successfully loaded trajectory: {totalFrames} frames, {totalDuration:F2} seconds"
        );

        return true;
    }

    /// <summary>
    /// Convert joint positions/rotations to joint angles (simplified IK approach)
    /// This is a basic implementation - you may want to use proper IK solver
    /// </summary>
    private float[] ConvertPositionsToJointAngles(Vector3[] positions, Quaternion[] rotations)
    {
        float[] angles = new float[JointCount];

        // This is a simplified conversion
        // In a real implementation, you'd use the actual IK solver from your project
        // For now, we'll try to extract meaningful joint angles from the rotation data

        for (int i = 0; i < Mathf.Min(angles.Length, rotations.Length); i++)
        {
            // Extract joint angle from quaternion (simplified - assumes revolute joints)
            // This is a very basic approximation and may need refinement
            Vector3 euler = rotations[i].eulerAngles;
            angles[i] = euler.y; // Use Y component as primary rotation (adjust based on your joint configuration)
        }

        return angles;
    }

    /// <summary>
    /// Clear all trajectory data
    /// </summary>
    // private void ClearTrajectoryData()
    // {
    //     timestamps.Clear();
    //     jointAnglesTrajectory.Clear();
    //     jointPositionsTrajectory.Clear();
    //     jointRotationsTrajectory.Clear();
    //     totalFrames = 0;
    //     totalDuration = 0f;
    //     currentTime = 0f;
    //     currentFrame = 0;
    //     currentPlaybackTime = 0f;
    // }

    #endregion

    #region Trajectory Playback

    /// <summary>
    /// Start trajectory playback
    /// </summary>
    public void StartPlayback()
    {
        if (totalFrames == 0)
        {
            Debug.LogWarning("No trajectory loaded. Call LoadTrajectory() first.");
            return;
        }

        if (playbackCoroutine != null)
        {
            StopCoroutine(playbackCoroutine);
        }

        isPlaying = true;
        isPaused = false;
        currentFrame = 0;
        currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;

        playbackCoroutine = StartCoroutine(PlaybackTrajectory());
        Debug.Log("Started trajectory playback");
    }

    /// <summary>
    /// Pause trajectory playback
    /// </summary>
    public void PausePlayback()
    {
        isPaused = !isPaused;
        Debug.Log(isPaused ? "Trajectory playback paused" : "Trajectory playback resumed");
    }

    /// <summary>
    /// Stop trajectory playback
    /// </summary>
    public void StopPlayback()
    {
        isPlaying = false;
        isPaused = false;

        if (playbackCoroutine != null)
        {
            StopCoroutine(playbackCoroutine);
            playbackCoroutine = null;
        }

        currentFrame = 0;
        currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;
        Debug.Log("Trajectory playback stopped");
    }

    /// <summary>
    /// Jump to specific frame
    /// </summary>
    // public void JumpToFrame(int frameIndex)
    // {
    //     if (frameIndex >= 0 && frameIndex < totalFrames)
    //     {
    //         currentFrame = frameIndex;
    //         currentTime = timestamps[frameIndex];
    //         SetRobotToFrame(frameIndex);
    //     }
    // }

    /// <summary>
    /// Set playback speed
    /// </summary>
    public void SetPlaybackSpeed(float speed)
    {
        playbackSpeed = Mathf.Min(Mathf.Max(0.25f, speed), 4f); //speed between 0.25x and 4x
    }

    /// <summary>
    /// Main playback coroutine
    /// </summary>
    private IEnumerator PlaybackTrajectory()
    {
        float startTime = Time.time;

        while (isPlaying && currentFrame < totalFrames)
        {
            if (!isPaused)
            {
                // Set robot to current frame
                float timestamp = timestamps[currentFrame];

                SetRobotToFrame(timestamp); //timestamps[currentFrame] gives the current time elapsed

                // Wait for next frame based on timestamp difference
                if (currentFrame < totalFrames - 1)
                {
                    float timeToNextFrame =
                        (timestamps[currentFrame + 1] - timestamps[currentFrame]) / playbackSpeed;
                    yield return new WaitForSeconds(timeToNextFrame);
                }
                else
                {
                    yield return new WaitForSeconds(0.1f); // Small delay at end
                }

                currentFrame++;
                if (currentFrame < timestamps.Count)
                {
                    currentTime = timestamps[currentFrame];
                }
            }
            else
            {
                yield return null; // Wait while paused
            }
        }

        // Handle loop or end
        if (isPlaying)
        {
            if (loopTrajectory)
            {
                currentFrame = 0;
                currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;
                playbackCoroutine = StartCoroutine(PlaybackTrajectory());
            }
            else
            {
                StopPlayback();
                Debug.Log("Trajectory playback completed");
            }
        }
    }

    /// <summary>
    /// Set robot joints to specific frame
    /// </summary>
    private void SetRobotToFrame(float timestamp)
    {
        int robotCount = robots.Length;

        for (int idx = 0; idx < robotCount; idx++)
        {
            Dictionary<float, StoredTrajectoryModel> robotTrajectory = robotTrajectories[idx];
            if (robotTrajectories[idx].ContainsKey(timestamp))
            {
                StoredTrajectoryModel storedTrajectory = robotTrajectory[timestamp];
                float[] jointAngles = storedTrajectory.jointAngles;
                bool suctionOn = storedTrajectory.suctionState;
                bool attracted = storedTrajectory.attractedState;

                Debug.Log(
                    $"Setting frame {timestamp}s: joint angles [{string.Join(", ", jointAngles.Select(a => a.ToString("F3")))}]"
                );

                if (robotController != null)
                {
                    robotController.SetJointAngles(robots[idx], jointAngles);
                    robotController.SetSuctionState(robots[idx], suctionOn);
                    robotController.SetBlockAttractedState(robots[idx], attracted); //uncomment this line to enable reliable attraction state setting
                    //Debug.Log($"Joint angles and suction state sent to robot controller");
                    //if (attracted) Debug.Log("Block is attracted to suction");
                }
                else
                    Debug.LogError("CSVTrajectoryController: robotController is null!");
            }
        }
    }

    #endregion

    #region Input Handling

    private void HandleInput()
    {
        // Playback controls
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (isPlaying)
                PausePlayback();
            else
                StartPlayback();
        }

        if (Input.GetKeyDown(KeyCode.S))
        {
            StopPlayback();
        }

        // Speed control
        // if (Input.GetKeyDown(KeyCode.Equals) || Input.GetKeyDown(KeyCode.KeypadPlus))
        // {
        //     SetPlaybackSpeed(playbackSpeed * 1.2f);
        // }

        // if (Input.GetKeyDown(KeyCode.Minus) || Input.GetKeyDown(KeyCode.KeypadMinus))
        // {
        //     SetPlaybackSpeed(playbackSpeed / 1.2f);
        // }

        // Frame jumping (for debugging)
        // if (Input.GetKeyDown(KeyCode.LeftArrow))
        // {
        //     JumpToFrame(Mathf.Max(0, currentFrame - 1));
        // }

        // if (Input.GetKeyDown(KeyCode.RightArrow))
        // {
        //     JumpToFrame(Mathf.Min(totalFrames - 1, currentFrame + 1));
        // }

        // Test joint movement (for debugging)
        // if (Input.GetKeyDown(KeyCode.T))
        // {
        //     TestJointMovement();
        // }
    }

    #endregion

    #region GUI

    private void DrawTrajectoryGUI()
    {
        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.normal.textColor = Color.white;

        int startX = 10;
        int startY = 300;
        int lineHeight = 25;

        string instructionText = "Load trajectory [F1] | Play trajectory [Space]";
        GUI.Label(new Rect(10, 35, 400, 30), instructionText, style);

        // Trajectory info
        GUI.Label(
            new Rect(startX, startY, 300, 20),
            $"Trajectory: {totalFrames} frames, {totalDuration:F2}s",
            style
        );
        GUI.Label(
            new Rect(startX, startY + lineHeight, 300, 20),
            $"Current: Frame {currentFrame}, Time {currentPlaybackTime:F2}s",
            style
        );
        GUI.Label(
            new Rect(startX, startY + 2 * lineHeight, 300, 20),
            $"Speed: {playbackSpeed:F2}x, Status: {(isPlaying ? (isPaused ? "Paused" : "Playing") : "Stopped")}",
            style
        );

        // Progress bar
        if (totalDuration > 0)
        {
            float progress = currentPlaybackTime / totalDuration;
            GUI.Box(new Rect(startX, startY + 3 * lineHeight, 200, 20), "");
            GUI.Box(new Rect(startX, startY + 3 * lineHeight, 200 * progress, 20), "");
        }

        // Control buttons
        int buttonY = startY + 5 * lineHeight;
        int buttonWidth = 80;
        int buttonHeight = 30;
        int buttonSpacing = 5;

        if (
            GUI.Button(
                new Rect(startX, buttonY, buttonWidth, buttonHeight),
                isPlaying ? "Pause" : "Play"
            )
        )
        {
            if (isPlaying)
                PausePlayback();
            else
                StartPlayback();
        }

        if (
            GUI.Button(
                new Rect(startX + buttonWidth + buttonSpacing, buttonY, buttonWidth, buttonHeight),
                "Stop"
            )
        )
        {
            StopPlayback();
        }

        if (
            GUI.Button(
                new Rect(
                    startX + 2 * (buttonWidth + buttonSpacing),
                    buttonY,
                    buttonWidth,
                    buttonHeight
                ),
                "Reload"
            )
        )
        {
            if (!string.IsNullOrEmpty(csvFilePath) || csvFile != null)
            {
                LoadTrajectory();
            }
        }

        // Speed controls
        GUI.Label(
            new Rect(startX, buttonY + buttonHeight + 10, 200, 20),
            $"Playback Speed: {playbackSpeed:F2}x",
            style
        );

        if (GUI.Button(new Rect(startX, buttonY + buttonHeight + 35, 40, 25), "-"))
        {
            SetPlaybackSpeed(playbackSpeed - 0.25f);
        }

        if (GUI.Button(new Rect(startX + 45, buttonY + buttonHeight + 35, 40, 25), "+"))
        {
            SetPlaybackSpeed(playbackSpeed + 0.25f);
        }

        if (GUI.Button(new Rect(startX + 90, buttonY + buttonHeight + 35, 40, 25), "o"))
        {
            SetPlaybackSpeed(DefaultPlaybackSpeed);
        }

        // Instructions
        GUI.Label(
            new Rect(startX, buttonY + buttonHeight + 70, 400, 20),
            "Controls: Space=Play/Pause, S=Stop, R=Reload, +/-=Speed",
            style
        );
        GUI.Label(
            new Rect(startX, buttonY + buttonHeight + 95, 400, 20),
            "Arrow Keys: Frame stepping",
            style
        );
    }

    #endregion

    #region Public API Methods

    /// <summary>
    /// Load trajectory from file path
    /// </summary>
    public void LoadTrajectoryFromPath(string[] paths)
    {
        csvFilePaths = paths;
        LoadTrajectory();
    }

    /// <summary>
    /// Load trajectory from TextAsset
    /// </summary>
    public void LoadTrajectoryFromTextAsset(TextAsset textAsset)
    {
        csvFile = textAsset;
        LoadTrajectory();
    }

    /// <summary>
    /// Get current trajectory progress (0-1)
    /// </summary>
    public float GetProgress()
    {
        return totalDuration > 0 ? currentPlaybackTime / totalDuration : 0f;
    }

    /// <summary>
    /// Check if trajectory is currently playing
    /// </summary>
    public bool IsPlaying()
    {
        return isPlaying && !isPaused;
    }

    #endregion
}
