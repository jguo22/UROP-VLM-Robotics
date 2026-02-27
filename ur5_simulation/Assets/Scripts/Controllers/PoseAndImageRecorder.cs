using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;
using static ConstantsUR5;

/// <summary>
/// Records per-step RLDS-compatible data: joint state, delta EE action, suction,
/// episode boundary flags, reward, and a camera screenshot at an adjustable FPS.
/// Attach to any GameObject. Assign references in Inspector.
/// StartRecording/StopRecording can be called repeatedly without reinitializing.
/// language_instruction is left empty — populate externally before training.
/// </summary>
public class PoseAndImageRecorder : MonoBehaviour
{
    [Header("Recording Settings")]
    public bool recordOnStart = false;
    public float recordingFPS = 5f;

    [Header("References")]
    public Transform endEffectorTransform;
    public Camera recordingCamera;
    public SuctionController suctionController;
    public RobotArmSetup robotArmSetup;

    [Header("Export Settings")]
    public string sessionName = "recording";
    public bool appendTimestamp = true;

    // Runtime state
    private bool isRecording = false;
    private float nextSampleTime = 0f;
    private string sessionFolder;
    private int frameIndex = 0;

    // Render resources — allocated once, reused across episodes
    private RenderTexture renderTexture;
    private Texture2D screenshotBuffer;
    private bool renderResourcesReady = false;

    // Previous frame pose for delta calculation
    private Vector3    prevPos;
    private Quaternion prevRot;

    // Buffer rows — is_last/is_terminal/reward only known on StopRecording()
    private List<string> frameBuffer = new List<string>();

    private const string PoseHeader =
        "frame,timestamp," +
        "joint_0,joint_1,joint_2,joint_3,joint_4,joint_5," +
        "delta_pos_x,delta_pos_y,delta_pos_z," +
        "delta_rot_x,delta_rot_y,delta_rot_z,delta_rot_w," +
        "suction_on," +
        "is_first,is_last,is_terminal,reward,discount";

    // -------------------------------------------------------------------------

    private void Start()
    {
        if (recordOnStart)
            StartRecording();
    }

    private void Update()
    {
        if (!isRecording) return;
        if (Time.time < nextSampleTime) return;

        CaptureFrame();
        nextSampleTime = Time.time + (1f / recordingFPS);
    }

    private void OnDisable()
    {
        if (isRecording)
            StopRecording();
    }

    private void OnDestroy()
    {
        if (renderTexture != null)    { renderTexture.Release(); Destroy(renderTexture); }
        if (screenshotBuffer != null) { Destroy(screenshotBuffer); }
    }

    // -------------------------------------------------------------------------

    public void StartRecording()
    {
        if (isRecording) return;
        if (endEffectorTransform == null) { Debug.LogError("PoseAndImageRecorder: endEffectorTransform not assigned."); return; }
        if (recordingCamera == null)      { Debug.LogError("PoseAndImageRecorder: recordingCamera not assigned."); return; }
        if (robotArmSetup == null)        { Debug.LogError("PoseAndImageRecorder: robotArmSetup not assigned."); return; }

        InitRenderResources();
        SetupEpisode();

        isRecording = true;
        nextSampleTime = Time.time;

        Debug.Log($"PoseAndImageRecorder: started recording → {sessionFolder}");
    }

    public void StopRecording()
    {
        if (!isRecording) return;
        isRecording = false;

        FlushBufferToDisk();

        Debug.Log($"PoseAndImageRecorder: stopped. {frameIndex} frames saved → {sessionFolder}");
    }

    // -------------------------------------------------------------------------

    /// <summary>Allocates render texture and screenshot buffer once. Safe to call repeatedly.</summary>
    private void InitRenderResources()
    {
        if (renderResourcesReady) return;

        int width  = recordingCamera.pixelWidth  > 0 ? recordingCamera.pixelWidth  : 640;
        int height = recordingCamera.pixelHeight > 0 ? recordingCamera.pixelHeight : 480;
        renderTexture    = new RenderTexture(width, height, 24);
        screenshotBuffer = new Texture2D(width, height, TextureFormat.RGB24, false);
        renderResourcesReady = true;
    }

    /// <summary>Creates a new episode folder and resets per-episode state.</summary>
    private void SetupEpisode()
    {
        string timestamp  = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string folderName = appendTimestamp ? $"{sessionName}_{timestamp}" : sessionName;
        sessionFolder     = Path.Combine(Application.dataPath, "..", "Exports", "dataset", folderName);

        Directory.CreateDirectory(Path.Combine(sessionFolder, "images"));

        frameBuffer.Clear();
        frameIndex = 0;
        prevPos = endEffectorTransform.position;
        prevRot = endEffectorTransform.rotation;
    }

    private void CaptureFrame()
    {
        // --- Joint state (observation) ---
        ArticulationBody[] joints = robotArmSetup.robotJoints;
        string jointCols = "";
        for (int i = 0; i < JointCount; i++)
            jointCols += $"{joints[i].jointPosition[0]:F5},";  // radians

        // --- Delta pose (action) ---
        Vector3    pos      = endEffectorTransform.position;
        Quaternion rot      = endEffectorTransform.rotation;
        Vector3    deltaPos = pos - prevPos;
        Quaternion deltaRot = rot * Quaternion.Inverse(prevRot);
        prevPos = pos;
        prevRot = rot;

        // --- Suction (action) ---
        int suction = (suctionController != null && suctionController.enableSuction) ? 1 : 0;

        // --- Episode flags (is_last/is_terminal/reward appended at flush) ---
        int isFirst = frameIndex == 0 ? 1 : 0;

        string row =
            $"{frameIndex},{Time.time:F4}," +
            jointCols +
            $"{deltaPos.x:F5},{deltaPos.y:F5},{deltaPos.z:F5}," +
            $"{deltaRot.x:F5},{deltaRot.y:F5},{deltaRot.z:F5},{deltaRot.w:F5}," +
            $"{suction}," +
            $"{isFirst}";

        frameBuffer.Add(row);

        // --- Screenshot ---
        RenderTexture prevRT = recordingCamera.targetTexture;
        recordingCamera.targetTexture = renderTexture;
        recordingCamera.Render();

        RenderTexture.active = renderTexture;
        screenshotBuffer.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        screenshotBuffer.Apply();
        RenderTexture.active = null;
        recordingCamera.targetTexture = prevRT;

        byte[] png       = screenshotBuffer.EncodeToPNG();
        string imagePath = Path.Combine(sessionFolder, "images", $"{frameIndex:D6}.png");
        File.WriteAllBytes(imagePath, png);

        frameIndex++;
    }

    private void FlushBufferToDisk()
    {
        if (frameBuffer.Count == 0) return;

        string posesPath = Path.Combine(sessionFolder, "poses.csv");
        using (StreamWriter writer = new StreamWriter(posesPath, false))
        {
            writer.WriteLine(PoseHeader);
            for (int i = 0; i < frameBuffer.Count; i++)
            {
                bool isLast  = i == frameBuffer.Count - 1;
                string suffix = isLast
                    ? ",1,1,1.0,1.0"   // final step: is_last, is_terminal, reward=1
                    : ",0,0,0.0,1.0";
                writer.WriteLine(frameBuffer[i] + suffix);
            }
        }

        frameBuffer.Clear();
    }
}
