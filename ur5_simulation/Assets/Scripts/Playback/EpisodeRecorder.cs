using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
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
    [SerializeField]
    private ObservationCapture observation;

    [Header("Export Settings")]
    public string sessionName = "recording";
    public bool appendTimestamp = true;

    // Runtime state
    private bool isRecording = false;
    private bool isPaused = false;
    private float nextSampleTime = 0f;
    private string sessionFolder;
    private int frameIndex = 0;

    private SceneSetup sceneSetup;

    // Per-frame pose for delta calculation at flush time
    private List<Vector3> framePositions = new List<Vector3>();
    private List<Quaternion> frameRotations = new List<Quaternion>();

    // Buffer rows (without deltas) — deltas and is_last/is_terminal/reward applied at flush
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
        sceneSetup = FindAnyObjectByType<SceneSetup>();

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

    // -------------------------------------------------------------------------

    public void StartRecording()
    {
        if (isRecording)
            return;
        if (observation == null)
        {
            Debug.LogError("EpisodeRecorder: observation not assigned.");
            return;
        }

        observation.Initialize();

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

        observation.ReleaseCameraTarget();

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

    /// <summary>Creates a new episode folder, saves starting block positions, and resets per-episode state.</summary>
    private void SetupEpisode()
    {
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string folderName = appendTimestamp ? $"{sessionName}_{timestamp}" : sessionName;
        sessionFolder = ProjectPaths.Get(
            Path.Combine("ur5_simulation", "Exports", "dataset", folderName)
        );

        Directory.CreateDirectory(Path.Combine(sessionFolder, "images"));

        SaveBlockPositions();

        frameBuffer.Clear();
        framePositions.Clear();
        frameRotations.Clear();
        frameIndex = 0;
    }

    /// <summary>Writes blocks.csv with the name and world position of each target block at episode start.</summary>
    private void SaveBlockPositions()
    {
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
        ObservationCapture.Observation obs = observation.CaptureState();

        framePositions.Add(obs.endEffectorPosition);
        frameRotations.Add(obs.endEffectorRotation);

        string jointAngleCols = "";
        for (int i = 0; i < JointCount; i++)
            jointAngleCols += $"{obs.jointAnglesDeg[i]:F5},";

        int isFirst = frameIndex == 0 ? 1 : 0;

        // Row without deltas — deltas computed at flush as (next frame - current frame)
        string row =
            $"{frameIndex},{Time.time:F4},"
            + $"{obs.suctionOn},"
            + jointAngleCols
            + $"{obs.blockAttracted},"
            + $"{isFirst}";

        frameBuffer.Add(row);

        WriteScreenshotAsync(frameIndex);

        frameIndex++;
    }

    private void WriteScreenshotAsync(int index)
    {
        byte[] png = observation.CaptureScreenshotPng();
        string imagePath = Path.Combine(sessionFolder, "images", $"{index:D6}.png");
        Task.Run(() => File.WriteAllBytes(imagePath, png));
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
                // Delta to next frame; last frame gets zero delta
                Vector3 deltaPos;
                Quaternion deltaRot;
                if (i < frameBuffer.Count - 1)
                {
                    deltaPos = framePositions[i + 1] - framePositions[i];
                    deltaRot = frameRotations[i + 1] * Quaternion.Inverse(frameRotations[i]);
                }
                else
                {
                    deltaPos = Vector3.zero;
                    deltaRot = Quaternion.identity;
                }

                // Row format: "frame,Timestamp,<rest...>"
                // Insert deltas after the timestamp
                string row = frameBuffer[i];
                int firstComma = row.IndexOf(',');
                int secondComma = row.IndexOf(',', firstComma + 1);
                string prefix = row.Substring(0, secondComma + 1);
                string rest = row.Substring(secondComma + 1);

                string deltaStr =
                    $"{deltaPos.x:F5},{deltaPos.y:F5},{deltaPos.z:F5},"
                    + $"{deltaRot.x:F5},{deltaRot.y:F5},{deltaRot.z:F5},{deltaRot.w:F5},";

                bool isLast = i == frameBuffer.Count - 1;
                string suffix = isLast
                    ? ",1,1,1.0,1.0" // final step: is_last, is_terminal, reward=1
                    : ",0,0,0.0,1.0";

                writer.WriteLine(prefix + deltaStr + rest + suffix);
            }
        }

        frameBuffer.Clear();
        framePositions.Clear();
        frameRotations.Clear();
    }
}
