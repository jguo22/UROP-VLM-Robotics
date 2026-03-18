using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Records per-step RLDS-compatible data: joint state, delta EE action, suction,
/// episode boundary flags, reward, and a camera screenshot at an adjustable FPS.
/// Also saves starting block positions at the beginning of each episode.
/// Attach to any GameObject. Assign references in Inspector.
/// StartRecording/StopRecording can be called repeatedly without reinitializing.
/// language_instruction is left empty — populate externally before training.
/// </summary>
public class EpisodeRecorder : MonoBehaviour
{
    [Header("Recording Settings")]
    public bool recordOnStart = false;
    public float recordingFPS = 10f;

    [Header("References")]
    public Camera recordingCamera;
    public SuctionController suctionController;
    public RobotArmSetup robotArmSetup;

    [Header("Export Settings")]
    public string sessionName = "recording";
    public bool appendTimestamp = true;

    // Runtime state
    private bool isRecording = false;
    private bool isPaused = false;
    private float nextSampleTime = 0f;
    private string sessionFolder;
    private int frameIndex = 0;

    // Render resources — allocated once, reused across episodes
    private RenderTexture renderTexture;
    private Texture2D screenshotBuffer;
    private bool renderResourcesReady = false;

    // Cached end effector transform
    private Transform endEffector;

    // Previous frame pose for delta calculation
    private Vector3 prevPos;
    private Quaternion prevRot;

    // Buffer rows — is_last/is_terminal/reward only known on StopRecording()
    private List<string> frameBuffer = new List<string>();

    private static string BuildPoseHeader()
    {
        string h =
            "frame,Timestamp,"
            + "delta_pos_x,delta_pos_y,delta_pos_z,"
            + "delta_rot_x,delta_rot_y,delta_rot_z,delta_rot_w,"
            + "suctionOn,";
        for (int i = 0; i < JointCount; i++)
            h += $"joint_{i},";
        h += "blockAttracted,";
        h += "is_first,is_last,is_terminal,reward,discount";
        return h;
    }

    private static readonly string PoseHeader = BuildPoseHeader();

    // -------------------------------------------------------------------------

    private void Start()
    {
        if (recordOnStart)
            StartRecording();
    }

    private void Update()
    {
        if (!isRecording || isPaused)
            return;
        if (Time.time < nextSampleTime)
            return;

        CaptureFrame();
        nextSampleTime = Time.time + (1f / recordingFPS);
    }

    private void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
        if (screenshotBuffer != null)
        {
            Destroy(screenshotBuffer);
        }
    }

    // -------------------------------------------------------------------------

    public void StartRecording()
    {
        if (isRecording)
            return;
        if (recordingCamera == null)
        {
            Debug.LogError("EpisodeRecorder: recordingCamera not assigned.");
            return;
        }
        if (robotArmSetup == null)
        {
            Debug.LogError("EpisodeRecorder: robotArmSetup not assigned.");
            return;
        }

        endEffector = robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform;

        InitRenderResources();
        SetupEpisode();

        isRecording = true;
        nextSampleTime = Time.time;

        Debug.Log($"EpisodeRecorder: started recording → {sessionFolder}");
    }

    public void StopRecording()
    {
        if (!isRecording)
            return;
        isRecording = false;
        isPaused = false;

        FlushBufferToDisk();

        Debug.Log($"EpisodeRecorder: stopped. {frameIndex} frames saved → {sessionFolder}");
    }

    /// <summary>Pauses frame capture without ending the episode.</summary>
    public void PauseRecording()
    {
        isPaused = true;
    }

    /// <summary>Resumes frame capture after a pause.</summary>
    public void ResumeRecording()
    {
        if (!isRecording)
            return;
        isPaused = false;
        nextSampleTime = Time.time; // capture immediately on resume
    }

    // -------------------------------------------------------------------------

    /// <summary>Allocates render texture and screenshot buffer once. Safe to call repeatedly.</summary>
    private void InitRenderResources()
    {
        if (renderResourcesReady)
            return;

        int width = recordingCamera.pixelWidth > 0 ? recordingCamera.pixelWidth : 640;
        int height = recordingCamera.pixelHeight > 0 ? recordingCamera.pixelHeight : 480;
        renderTexture = new RenderTexture(width, height, 24);
        screenshotBuffer = new Texture2D(width, height, TextureFormat.RGB24, false);
        renderResourcesReady = true;
    }

    /// <summary>Creates a new episode folder, saves starting block positions, and resets per-episode state.</summary>
    private void SetupEpisode()
    {
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string folderName = appendTimestamp ? $"{sessionName}_{timestamp}" : sessionName;
        sessionFolder = Path.Combine(Application.dataPath, "..", "Exports", "dataset", folderName);

        Directory.CreateDirectory(Path.Combine(sessionFolder, "images"));

        SaveBlockPositions();

        frameBuffer.Clear();
        frameIndex = 0;
        prevPos = endEffector.position;
        prevRot = endEffector.rotation;
    }

    /// <summary>Writes blocks.csv with the name and world position of each target block at episode start.</summary>
    private void SaveBlockPositions()
    {
        SceneSetup sceneSetup = FindObjectOfType<SceneSetup>();
        if (sceneSetup == null || sceneSetup.targets == null || sceneSetup.targets.Length == 0)
            return;

        string blocksPath = Path.Combine(sessionFolder, "blocks.csv");
        using (StreamWriter writer = new StreamWriter(blocksPath, false))
        {
            writer.WriteLine("name,pos_x,pos_y,pos_z");
            foreach (GameObject block in sceneSetup.targets)
            {
                if (block == null)
                    continue;
                Vector3 p = block.transform.position;
                writer.WriteLine($"{block.name},{p.x:F5},{p.y:F5},{p.z:F5}");
            }
        }
    }

    private void CaptureFrame()
    {
        // --- Delta pose ---
        Vector3 pos = endEffector.position;
        Quaternion rot = endEffector.rotation;
        Vector3 deltaPos = pos - prevPos;
        Quaternion deltaRot = rot * Quaternion.Inverse(prevRot);
        prevPos = pos;
        prevRot = rot;

        // --- Suction ---
        bool suction = suctionController != null && suctionController.enableSuction;

        // --- Joint angles (degrees) ---
        ArticulationBody[] joints = robotArmSetup.robotJoints;
        string jointAngleCols = "";
        for (int i = 0; i < JointCount; i++)
            jointAngleCols += $"{joints[i].jointPosition[0] * Mathf.Rad2Deg:F5},";

        // --- Block attracted ---
        bool blockAttracted = suctionController != null && suctionController.isBlockAttached;

        // --- Episode flags (is_last/is_terminal/reward appended at flush) ---
        int isFirst = frameIndex == 0 ? 1 : 0;

        string row =
            $"{frameIndex},{Time.time:F4},"
            + $"{deltaPos.x:F5},{deltaPos.y:F5},{deltaPos.z:F5},"
            + $"{deltaRot.x:F5},{deltaRot.y:F5},{deltaRot.z:F5},{deltaRot.w:F5},"
            + $"{suction},"
            + jointAngleCols
            + $"{blockAttracted},"
            + $"{isFirst}";

        frameBuffer.Add(row);

        // --- Screenshot ---
        RenderTexture prevRT = recordingCamera.targetTexture;
        recordingCamera.targetTexture = renderTexture;
        recordingCamera.Render();

        RenderTexture.active = renderTexture;
        screenshotBuffer.ReadPixels(
            new Rect(0, 0, renderTexture.width, renderTexture.height),
            0,
            0
        );
        screenshotBuffer.Apply();
        RenderTexture.active = null;
        recordingCamera.targetTexture = prevRT;

        byte[] png = screenshotBuffer.EncodeToPNG();
        string imagePath = Path.Combine(sessionFolder, "images", $"{frameIndex:D6}.png");
        File.WriteAllBytes(imagePath, png);

        frameIndex++;
    }

    private void FlushBufferToDisk()
    {
        if (frameBuffer.Count == 0)
            return;

        string posesPath = Path.Combine(sessionFolder, "poses.csv");
        using (StreamWriter writer = new StreamWriter(posesPath, false))
        {
            writer.WriteLine(PoseHeader);
            for (int i = 0; i < frameBuffer.Count; i++)
            {
                bool isLast = i == frameBuffer.Count - 1;
                string suffix = isLast
                    ? ",1,1,1.0,1.0" // final step: is_last, is_terminal, reward=1
                    : ",0,0,0.0,1.0";
                writer.WriteLine(frameBuffer[i] + suffix);
            }
        }

        frameBuffer.Clear();
    }
}
