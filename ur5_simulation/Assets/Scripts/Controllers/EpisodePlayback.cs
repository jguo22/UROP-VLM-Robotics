using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Episode Playback - Loads and replays recorded episodes from EpisodeRecorder.
/// Reads blocks.csv to restore block starting positions before playback begins.
/// Compatible with exported poses.csv format.
/// </summary>
[DefaultExecutionOrder(100)]
public class EpisodePlayback : MonoBehaviour
{
    private UnifiedRobotController robotController;

    [Header("Trajectory Settings")]
    public string episodeFolderPath = ""; // Path to episode folder (contains poses.csv and blocks.csv)
    private string csvFilePath =>
        string.IsNullOrEmpty(episodeFolderPath) ? "" : Path.Combine(episodeFolderPath, "poses.csv");
    public float playbackSpeed = 1.0f;
    public bool loopTrajectory = false;
    public bool autoStart = false;

    private bool isPlaying = false;
    private bool isPaused = false;
    private float currentTime = 0f;
    private int currentFrame = 0;
    private int totalFrames = 0;
    private float totalDuration = 0f;
    private float currentPlaybackTime = 0f;

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

        if (autoStart && !string.IsNullOrEmpty(csvFilePath))
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
            robotController = GetComponent<UnifiedRobotController>();

        if (robotController == null)
            Debug.LogError("EpisodePlayback: No UnifiedRobotController found!");
    }

    #endregion

    #region Block Setup

    /// <summary>
    /// Reads blocks.csv from the same episode folder as csvFilePath and
    /// teleports each named block in SceneSetup.targets to its recorded position.
    /// </summary>
    private void LoadAndApplyBlockPositions()
    {
        string episodeFolder = GetEpisodeFolder();
        if (string.IsNullOrEmpty(episodeFolder))
            return;

        string blocksPath = Path.Combine(episodeFolder, "blocks.csv");
        if (!File.Exists(blocksPath))
        {
            Debug.LogWarning($"EpisodePlayback: blocks.csv not found at {blocksPath}");
            return;
        }

        SceneSetup sceneSetup = FindObjectOfType<SceneSetup>();
        if (sceneSetup == null || sceneSetup.targets == null)
        {
            Debug.LogWarning(
                "EpisodePlayback: SceneSetup or targets not found, skipping block setup."
            );
            return;
        }

        // Build a name → GameObject lookup from scene targets
        Dictionary<string, GameObject> blockMap = new Dictionary<string, GameObject>();
        foreach (GameObject block in sceneSetup.targets)
        {
            if (block != null)
                blockMap[block.name] = block;
        }

        string[] lines = File.ReadAllLines(blocksPath);
        for (int i = 1; i < lines.Length; i++) // skip header
        {
            if (string.IsNullOrWhiteSpace(lines[i]))
                continue;

            string[] cols = lines[i].Split(',');
            if (cols.Length < 4)
                continue;

            string blockName = cols[0].Trim();
            if (
                !float.TryParse(cols[1], out float x)
                || !float.TryParse(cols[2], out float y)
                || !float.TryParse(cols[3], out float z)
            )
                continue;

            if (blockMap.TryGetValue(blockName, out GameObject block))
            {
                block.transform.position = new Vector3(x, y, z);
                Debug.Log($"EpisodePlayback: moved {blockName} to ({x:F3}, {y:F3}, {z:F3})");
            }
            else
            {
                Debug.LogWarning(
                    $"EpisodePlayback: block '{blockName}' not found in scene targets."
                );
            }
        }
    }

    /// <summary>Returns the episode folder path.</summary>
    private string GetEpisodeFolder()
    {
        if (!string.IsNullOrEmpty(episodeFolderPath))
            return episodeFolderPath;

        return null;
    }

    #endregion

    #region CSV Loading and Parsing

    /// <summary>
    /// Load trajectory from CSV file
    /// </summary>
    public bool LoadTrajectory()
    {
        string csvContent = "";

        if (!string.IsNullOrEmpty(csvFilePath))
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
            Debug.LogError("No episode folder specified. Set episodeFolderPath.");
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

            // Map joint indices directly to their column indices
            int[] jointAngleCols = new int[JointCount];
            for (int j = 0; j < JointCount; j++)
                jointAngleCols[j] = Array.IndexOf(headers, $"joint_{j}");

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

                float[] jointAngles = new float[JointCount];
                for (int j = 0; j < JointCount; j++)
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
            StopCoroutine(playbackCoroutine);

        LoadAndApplyBlockPositions();

        isPlaying = true;
        isPaused = false;
        currentFrame = 0;
        currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;

        playbackCoroutine = StartCoroutine(PlaybackTrajectory());
        Debug.Log("Started episode playback");
    }

    /// <summary>
    /// Pause trajectory playback
    /// </summary>
    public void PausePlayback()
    {
        isPaused = !isPaused;
        Debug.Log(isPaused ? "Episode playback paused" : "Episode playback resumed");
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
        Debug.Log("Episode playback stopped");
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
                SetRobotToFrame(currentFrame);

                if (currentFrame < totalFrames - 1)
                {
                    float timeToNextFrame =
                        (timestamps[currentFrame + 1] - timestamps[currentFrame]) / playbackSpeed;
                    yield return new WaitForSeconds(timeToNextFrame);
                }
                else
                {
                    yield return new WaitForSeconds(0.1f);
                }

                currentFrame++;
                if (currentFrame < timestamps.Count)
                    currentTime = timestamps[currentFrame];
            }
            else
            {
                yield return null;
            }
        }

        if (isPlaying)
        {
            if (loopTrajectory)
            {
                LoadAndApplyBlockPositions();
                currentFrame = 0;
                currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;
                playbackCoroutine = StartCoroutine(PlaybackTrajectory());
            }
            else
            {
                StopPlayback();
                Debug.Log("Episode playback completed");
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

            if (robotController != null)
            {
                robotController.SetJointAngles(jointAngles);
                robotController.SetSuctionState(suctionOn);
                // robotController.SetBlockAttractedState(attracted);
            }
            else
            {
                Debug.LogError("EpisodePlayback: robotController is null!");
            }
        }
        else
        {
            Debug.LogWarning(
                $"EpisodePlayback: Invalid frame index {frameIndex}, trajectory has {jointAnglesTrajectory.Count} frames"
            );
        }
    }

    #endregion

    #region Input Handling

    private void HandleInput()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (isPlaying)
                PausePlayback();
            else
                StartPlayback();
        }

        if (Input.GetKeyDown(KeyCode.S))
            StopPlayback();

        if (Input.GetKeyDown(KeyCode.R))
        {
            if (!string.IsNullOrEmpty(csvFilePath))
                LoadTrajectory();
        }

        if (Input.GetKeyDown(KeyCode.Equals) || Input.GetKeyDown(KeyCode.KeypadPlus))
            SetPlaybackSpeed(playbackSpeed * 1.2f);

        if (Input.GetKeyDown(KeyCode.Minus) || Input.GetKeyDown(KeyCode.KeypadMinus))
            SetPlaybackSpeed(playbackSpeed / 1.2f);

        if (Input.GetKeyDown(KeyCode.LeftArrow))
            JumpToFrame(Mathf.Max(0, currentFrame - 1));

        if (Input.GetKeyDown(KeyCode.RightArrow))
            JumpToFrame(Mathf.Min(totalFrames - 1, currentFrame + 1));
    }

    #endregion

    #region GUI

    private void DrawTrajectoryGUI()
    {
        float scale = Screen.height / 1080f;

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = Mathf.RoundToInt(14 * scale);
        style.normal.textColor = Color.white;

        GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
        buttonStyle.fontSize = Mathf.RoundToInt(13 * scale);

        float startX = 10 * scale;
        float lineHeight = 25 * scale;
        float labelW = 350 * scale;
        float labelH = lineHeight;

        float totalH = 8 * lineHeight + 30 * scale + 95 * scale;
        float startY = Screen.height - totalH - 10 * scale;

        GUI.Label(
            new Rect(startX, startY, labelW, labelH),
            $"Trajectory: {totalFrames} frames, {totalDuration:F2}s",
            style
        );
        GUI.Label(
            new Rect(startX, startY + lineHeight, labelW, labelH),
            $"Current: Frame {currentFrame}, Time {currentPlaybackTime:F2}s",
            style
        );
        GUI.Label(
            new Rect(startX, startY + 2 * lineHeight, labelW, labelH),
            $"Speed: {playbackSpeed:F1}x, Status: {(isPlaying ? (isPaused ? "Paused" : "Playing") : "Stopped")}",
            style
        );

        if (totalDuration > 0)
        {
            float progress = currentPlaybackTime / totalDuration;
            float barW = 200 * scale;
            float barH = 18 * scale;
            GUI.Box(new Rect(startX, startY + 3 * lineHeight, barW, barH), "");
            GUI.Box(new Rect(startX, startY + 3 * lineHeight, barW * progress, barH), "");
        }

        float buttonY = startY + 5 * lineHeight;
        float buttonWidth = 80 * scale;
        float buttonHeight = 30 * scale;
        float buttonSpacing = 5 * scale;

        if (
            GUI.Button(
                new Rect(startX, buttonY, buttonWidth, buttonHeight),
                isPlaying ? "Pause" : "Play",
                buttonStyle
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
                "Stop",
                buttonStyle
            )
        )
            StopPlayback();

        if (
            GUI.Button(
                new Rect(
                    startX + 2 * (buttonWidth + buttonSpacing),
                    buttonY,
                    buttonWidth,
                    buttonHeight
                ),
                "Reload",
                buttonStyle
            )
        )
        {
            if (!string.IsNullOrEmpty(csvFilePath))
                LoadTrajectory();
        }

        float speedY = buttonY + buttonHeight + 10 * scale;
        GUI.Label(
            new Rect(startX, speedY, labelW, labelH),
            $"Playback Speed: {playbackSpeed:F1}x",
            style
        );

        float smallBtnW = 40 * scale;
        float smallBtnH = 25 * scale;
        if (
            GUI.Button(
                new Rect(startX, speedY + lineHeight, smallBtnW, smallBtnH),
                "-",
                buttonStyle
            )
        )
            SetPlaybackSpeed(playbackSpeed / 1.2f);
        if (
            GUI.Button(
                new Rect(startX + smallBtnW + 5 * scale, speedY + lineHeight, smallBtnW, smallBtnH),
                "+",
                buttonStyle
            )
        )
            SetPlaybackSpeed(playbackSpeed * 1.2f);

        float infoY = speedY + lineHeight + smallBtnH + 5 * scale;
        GUI.Label(
            new Rect(startX, infoY, labelW, labelH),
            "Controls: Space=Play/Pause, S=Stop, R=Reload, +/-=Speed",
            style
        );
        GUI.Label(
            new Rect(startX, infoY + lineHeight, labelW, labelH),
            "Arrow Keys: Frame stepping",
            style
        );
    }

    #endregion

    #region Public API Methods

    public void LoadTrajectoryFromPath(string path)
    {
        episodeFolderPath = path;
        LoadTrajectory();
    }

    public float GetProgress()
    {
        return totalDuration > 0 ? currentPlaybackTime / totalDuration : 0f;
    }

    public bool IsPlaying()
    {
        return isPlaying && !isPaused;
    }

    #endregion
}
