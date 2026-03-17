using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// CSV Trajectory Controller - Loads and executes robot trajectories from CSV files
/// Compatible with exported RobotJointCoordinates CSV format
/// </summary>
public class CSVTrajectoryController : MonoBehaviour
{
    [HideInInspector]
    public UnifiedRobotController robotController;

    [Header("Trajectory Settings")]
    public TextAsset csvFile; // Drag and drop CSV file in Inspector
    public string csvFilePath = ""; // Alternative: specify file path
    public float playbackSpeed = 1.0f;
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

    // Internal data structures
    private List<float> timestamps = new List<float>();
    private List<bool> suctionStates = new List<bool>();
    private List<bool> attractedStates = new List<bool>();

    // 6 joints per frame
    private List<float[]> jointAnglesTrajectory = new List<float[]>();

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
            //robotController = FindObjectOfType<UnifiedRobotController>();
            robotController = GetComponent<UnifiedRobotController>();
        }

        if (robotController == null)
        {
            Debug.LogError("CSVTrajectoryController: No UnifiedRobotController found!");
        }
    }

    #endregion

    #region CSV Loading and Parsing

    /// <summary>
    /// Load trajectory from CSV file
    /// </summary>
    public bool LoadTrajectory()
    {
        string csvContent = "";

        // Try to load from TextAsset first
        if (csvFile != null)
        {
            csvContent = csvFile.text;
        }
        // Try to load from file path
        else if (!string.IsNullOrEmpty(csvFilePath))
        {
            try
            {
                csvContent = File.ReadAllText(csvFilePath);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Failed to load CSV file from path: {csvFilePath}\n{e.Message}");
                return false;
            }
        }
        else
        {
            Debug.LogError("No CSV file specified. Set csvFile or csvFilePath.");
            return false;
        }

        return ParseCSVContent(csvContent);
    }

    /// <summary>
    /// Parse CSV content into trajectory data
    /// </summary>
    private bool ParseCSVContent(string csvContent)
    {
        try
        {
            // Clear previous data
            ClearTrajectoryData();

            string[] lines = csvContent.Split('\n');

            if (lines.Length < 2)
            {
                Debug.LogError("CSV file must have at least header and one data row");
                return false;
            }

            // Parse header to understand column structure
            string[] headers = lines[0].Split(',').Select(h => h.Trim()).ToArray();

            string[] jointNames = { "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3" };

            // Map joint names directly to their jointAngle column indices
            int[] jointAngleCols = new int[jointNames.Length];
            for (int j = 0; j < jointNames.Length; j++)
                jointAngleCols[j] = Array.IndexOf(headers, $"{jointNames[j]}_jointAngle");

            int suctionColIndex = Array.IndexOf(headers, "suctionOn");
            int attractedColIndex = Array.IndexOf(headers, "blockAttracted");

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

                if (!float.TryParse(values[timestampCol], out float timestamp))
                    continue;

                timestamps.Add(timestamp);

                float[] jointAngles = new float[jointNames.Length];
                for (int j = 0; j < jointNames.Length; j++)
                {
                    if (jointAngleCols[j] >= 0 && jointAngleCols[j] < values.Length)
                        float.TryParse(values[jointAngleCols[j]], out jointAngles[j]);
                }

                suctionStates.Add(
                    suctionColIndex >= 0
                        && suctionColIndex < values.Length
                        && values[suctionColIndex].Trim() == "True"
                );

                attractedStates.Add(
                    attractedColIndex >= 0
                        && attractedColIndex < values.Length
                        && values[attractedColIndex].Trim() == "True"
                );

                jointAnglesTrajectory.Add(jointAngles);
            }

            totalFrames = jointAnglesTrajectory.Count;
            if (timestamps.Count > 1)
            {
                totalDuration = timestamps[timestamps.Count - 1] - timestamps[0];
            }

            Debug.Log(
                $"Successfully loaded trajectory: {totalFrames} frames, {totalDuration:F2} seconds"
            );
            return true;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error parsing CSV: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// Clear all trajectory data
    /// </summary>
    private void ClearTrajectoryData()
    {
        timestamps.Clear();
        jointAnglesTrajectory.Clear();
        suctionStates.Clear();
        attractedStates.Clear();
        totalFrames = 0;
        totalDuration = 0f;
        currentTime = 0f;
        currentFrame = 0;
        currentPlaybackTime = 0f;
    }

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
    public void JumpToFrame(int frameIndex)
    {
        if (frameIndex >= 0 && frameIndex < totalFrames)
        {
            currentFrame = frameIndex;
            currentTime = timestamps[frameIndex];
            SetRobotToFrame(frameIndex);
        }
    }

    /// <summary>
    /// Jump to specific time
    /// </summary>
    public void JumpToTime(float time)
    {
        if (timestamps.Count == 0)
            return;

        // Find closest timestamp
        int closestFrame = 0;
        float minDiff = Mathf.Abs(timestamps[0] - time);

        for (int i = 1; i < timestamps.Count; i++)
        {
            float diff = Mathf.Abs(timestamps[i] - time);
            if (diff < minDiff)
            {
                minDiff = diff;
                closestFrame = i;
            }
        }

        JumpToFrame(closestFrame);
    }

    /// <summary>
    /// Set playback speed
    /// </summary>
    public void SetPlaybackSpeed(float speed)
    {
        playbackSpeed = Mathf.Max(0.1f, speed);
    }

    /// <summary>
    /// Main playback coroutine
    /// </summary>
    private IEnumerator PlaybackTrajectory()
    {
        while (isPlaying && currentFrame < totalFrames)
        {
            if (!isPaused)
            {
                // Set robot to current frame
                SetRobotToFrame(currentFrame);

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
    private void SetRobotToFrame(int frameIndex)
    {
        if (frameIndex >= 0 && frameIndex < jointAnglesTrajectory.Count)
        {
            float[] jointAngles = jointAnglesTrajectory[frameIndex];
            bool suctionOn = suctionStates[frameIndex];
            bool attracted = attractedStates[frameIndex];

            Debug.Log(
                $"Setting frame {frameIndex}: joint angles [{string.Join(", ", jointAngles.Select(a => a.ToString("F3")))}]"
            );

            if (robotController != null)
            {
                robotController.SetJointAngles(jointAngles);
                robotController.SetSuctionState(suctionOn);
                robotController.SetBlockAttractedState(attracted); //uncomment this line to enable reliable attraction state setting
                Debug.Log($"Joint angles and suction state sent to robot controller");
                if (attracted)
                    Debug.Log("Block is attracted to suction");
            }
            else
            {
                Debug.LogError("CSVTrajectoryController: robotController is null!");
            }
        }
        else
        {
            Debug.LogWarning(
                $"CSVTrajectoryController: Invalid frame index {frameIndex}, trajectory has {jointAnglesTrajectory.Count} frames"
            );
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

        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!string.IsNullOrEmpty(csvFilePath) || csvFile != null)
            {
                LoadTrajectory();
            }
        }

        // Speed control
        if (Input.GetKeyDown(KeyCode.Equals) || Input.GetKeyDown(KeyCode.KeypadPlus))
        {
            SetPlaybackSpeed(playbackSpeed * 1.2f);
        }

        if (Input.GetKeyDown(KeyCode.Minus) || Input.GetKeyDown(KeyCode.KeypadMinus))
        {
            SetPlaybackSpeed(playbackSpeed / 1.2f);
        }

        // Frame jumping (for debugging)
        if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            JumpToFrame(Mathf.Max(0, currentFrame - 1));
        }

        if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            JumpToFrame(Mathf.Min(totalFrames - 1, currentFrame + 1));
        }

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
            $"Speed: {playbackSpeed:F1}x, Status: {(isPlaying ? (isPaused ? "Paused" : "Playing") : "Stopped")}",
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
            $"Playback Speed: {playbackSpeed:F1}x",
            style
        );

        if (GUI.Button(new Rect(startX, buttonY + buttonHeight + 35, 40, 25), "-"))
        {
            SetPlaybackSpeed(playbackSpeed / 1.2f);
        }

        if (GUI.Button(new Rect(startX + 45, buttonY + buttonHeight + 35, 40, 25), "+"))
        {
            SetPlaybackSpeed(playbackSpeed * 1.2f);
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
    public void LoadTrajectoryFromPath(string path)
    {
        csvFilePath = path;
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
