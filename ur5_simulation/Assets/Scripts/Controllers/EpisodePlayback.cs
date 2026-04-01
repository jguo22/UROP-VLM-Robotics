using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Abstract base for episode playback. Handles CSV loading, timing, GUI, and input.
/// Subclasses implement how frames are applied to the robot and what data they parse.
/// </summary>
[DefaultExecutionOrder(100)]
public abstract class EpisodePlayback : MonoBehaviour
{
    [Header("Trajectory Settings")]
    public string episodeFolderPath = "";
    public string csvFilePath = "";
    public string blocksFilePath = "";
    public float playbackSpeed = 1.0f;
    public bool loopTrajectory = false;
    public bool autoStart = false;
    public float minFrameInterval = 0f;

    // Playback state
    protected bool isPlaying = false;
    protected bool isPaused = false;
    protected float currentTime = 0f;
    protected int currentFrame = 0;
    protected int totalFrames = 0;
    protected float totalDuration = 0f;
    protected float currentPlaybackTime = 0f;

    // Shared trajectory data
    protected List<float> timestamps = new List<float>();
    protected List<bool> suctionStates = new List<bool>();
    protected List<bool> attractedStates = new List<bool>();

    // Timing
    private float playbackElapsed = 0f;
    private int lastAppliedFrame = -1;
    private float timeSinceLastFrameAdvance = 0f;

    #region Abstract / Virtual

    protected abstract void InitializeController();
    protected abstract void ApplyFrames(int lastAppliedFrame, int targetFrame);

    /// <summary>Called once with the header array. Subclass should resolve its column indices.</summary>
    protected abstract void ResolveColumns(string[] headers);

    /// <summary>Called per data row. Subclass should parse and store its specific data.</summary>
    protected abstract void ParseRow(string[] values);

    /// <summary>Called when trajectory data is cleared. Subclass should clear its own lists.</summary>
    protected abstract void ClearSubclassData();

    protected virtual void OnPlaybackStart() { }
    protected virtual void OnPlaybackStop() { }

    #endregion

    #region Unity Methods

    void Start()
    {
        InitializeController();

        if (
            autoStart
            && (!string.IsNullOrEmpty(csvFilePath) || !string.IsNullOrEmpty(episodeFolderPath))
        )
        {
            LoadTrajectory();
            StartPlayback();
        }
    }

    void Update()
    {
        HandleInput();
    }

    void FixedUpdate()
    {
        if (!isPlaying || isPaused)
            return;

        playbackElapsed += Time.fixedDeltaTime * playbackSpeed;
        timeSinceLastFrameAdvance += Time.fixedDeltaTime * playbackSpeed;
        float targetTime = timestamps[0] + playbackElapsed;

        if (minFrameInterval > 0f && timeSinceLastFrameAdvance < minFrameInterval)
            return;

        int prevFrame = currentFrame;
        while (currentFrame < totalFrames - 1 && timestamps[currentFrame + 1] <= targetTime)
            currentFrame++;

        if (currentFrame != prevFrame)
            timeSinceLastFrameAdvance = 0f;

        currentTime = timestamps[currentFrame];
        currentPlaybackTime = currentTime;

        int applyFrame = Mathf.Min(currentFrame + 1, totalFrames - 1);
        ApplyFrames(lastAppliedFrame, applyFrame);
        lastAppliedFrame = applyFrame;

        if (currentFrame >= totalFrames - 1)
        {
            if (loopTrajectory)
            {
                LoadAndApplyBlockPositions();
                OnPlaybackStart();
                currentFrame = 0;
                playbackElapsed = 0f;
                timeSinceLastFrameAdvance = 0f;
                lastAppliedFrame = -1;
                currentTime = timestamps[0];
            }
            else
            {
                StopPlayback();
                // Debug.Log("Episode playback completed");
            }
        }
    }

    void OnGUI()
    {
        DrawTrajectoryGUI();
    }

    #endregion

    #region Block Setup

    protected void LoadAndApplyBlockPositions()
    {
        string resolvedBlocksPath =
            !string.IsNullOrEmpty(blocksFilePath) ? blocksFilePath
            : !string.IsNullOrEmpty(episodeFolderPath)
                ? Path.Combine(episodeFolderPath, "blocks.csv")
            : "";

        if (string.IsNullOrEmpty(resolvedBlocksPath))
            return;

        if (!File.Exists(resolvedBlocksPath))
        {
            Debug.LogWarning($"EpisodePlayback: blocks.csv not found at {resolvedBlocksPath}");
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

        Dictionary<string, GameObject> blockMap = new Dictionary<string, GameObject>();
        foreach (GameObject block in sceneSetup.targets)
        {
            if (block != null)
                blockMap[block.name] = block;
        }

        string[] lines = File.ReadAllLines(resolvedBlocksPath);
        for (int i = 1; i < lines.Length; i++)
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
                block.GetComponent<Rigidbody>().WakeUp();
                // Debug.Log($"EpisodePlayback: moved {blockName} to ({x:F3}, {y:F3}, {z:F3})");
            }
            else
            {
                Debug.LogWarning(
                    $"EpisodePlayback: block '{blockName}' not found in scene targets."
                );
            }
        }
    }

    #endregion

    #region CSV Loading and Parsing

    public bool LoadTrajectory()
    {
        string resolvedCsvPath =
            !string.IsNullOrEmpty(csvFilePath) ? csvFilePath
            : !string.IsNullOrEmpty(episodeFolderPath)
                ? Path.Combine(episodeFolderPath, "poses.csv")
            : "";

        if (string.IsNullOrEmpty(resolvedCsvPath))
        {
            Debug.LogError("No trajectory CSV specified. Set csvFilePath or episodeFolderPath.");
            return false;
        }

        try
        {
            string csvContent = File.ReadAllText(resolvedCsvPath);
            return ParseCSVContent(csvContent);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to load CSV file from path: {resolvedCsvPath}\n{e.Message}");
            return false;
        }
    }

    private bool ParseCSVContent(string csvContent)
    {
        try
        {
            ClearTrajectoryData();

            string[] lines = csvContent.Split('\n');

            if (lines.Length < 2)
            {
                Debug.LogError("CSV file must have at least header and one data row");
                return false;
            }

            string[] headers = lines[0].Split(',').Select(h => h.Trim()).ToArray();

            int timestampCol = Array.IndexOf(headers, "Timestamp");
            if (timestampCol < 0)
            {
                Debug.LogError("Timestamp column not found in CSV");
                return false;
            }

            int suctionColIndex = Array.IndexOf(headers, "suctionOn");
            int attractedColIndex = Array.IndexOf(headers, "blockAttracted");

            ResolveColumns(headers);

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

                ParseRow(values);
            }

            totalFrames = timestamps.Count;
            if (timestamps.Count > 1)
            {
                totalDuration = timestamps[timestamps.Count - 1] - timestamps[0];
            }

            // Debug.Log(
            //     $"Successfully loaded trajectory: {totalFrames} frames, {totalDuration:F2} seconds"
            // );
            return true;
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error parsing CSV: {e.Message}");
            return false;
        }
    }

    private void ClearTrajectoryData()
    {
        timestamps.Clear();
        suctionStates.Clear();
        attractedStates.Clear();
        ClearSubclassData();
        totalFrames = 0;
        totalDuration = 0f;
        currentTime = 0f;
        currentFrame = 0;
        currentPlaybackTime = 0f;
    }

    #endregion

    #region Trajectory Playback

    public void StartPlayback()
    {
        if (totalFrames == 0)
        {
            Debug.LogWarning("No trajectory loaded. Call LoadTrajectory() first.");
            return;
        }

        LoadAndApplyBlockPositions();
        OnPlaybackStart();

        isPlaying = true;
        isPaused = false;
        currentFrame = 0;
        playbackElapsed = 0f;
        timeSinceLastFrameAdvance = 0f;
        lastAppliedFrame = -1;
        currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;

        // Debug.Log("Started episode playback");
    }

    public void PausePlayback()
    {
        isPaused = !isPaused;
        // Debug.Log(isPaused ? "Episode playback paused" : "Episode playback resumed");
    }

    public void StopPlayback()
    {
        isPlaying = false;
        isPaused = false;

        OnPlaybackStop();

        currentFrame = 0;
        playbackElapsed = 0f;
        timeSinceLastFrameAdvance = 0f;
        lastAppliedFrame = -1;
        currentTime = timestamps.Count > 0 ? timestamps[0] : 0f;
        // Debug.Log("Episode playback stopped");
    }

    public virtual void JumpToFrame(int frameIndex)
    {
        if (frameIndex >= 0 && frameIndex < totalFrames)
        {
            currentFrame = frameIndex;
            currentTime = timestamps[frameIndex];
            ApplyFrames(frameIndex - 1, frameIndex);
            lastAppliedFrame = frameIndex;
        }
    }

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

    public void SetPlaybackSpeed(float speed)
    {
        playbackSpeed = Mathf.Max(0.1f, speed);
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
            if (!string.IsNullOrEmpty(csvFilePath) || !string.IsNullOrEmpty(episodeFolderPath))
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
            if (!string.IsNullOrEmpty(csvFilePath) || !string.IsNullOrEmpty(episodeFolderPath))
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
        if (path.EndsWith(".csv", System.StringComparison.OrdinalIgnoreCase))
        {
            csvFilePath = path;
            episodeFolderPath = "";
            blocksFilePath = "";
        }
        else
        {
            episodeFolderPath = path;
            csvFilePath = "";
            blocksFilePath = "";
        }
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
