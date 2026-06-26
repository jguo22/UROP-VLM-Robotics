using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Serialization;
using static ConstantsUR5;

// Records per-episode RLDS-compatible data.
// Consumed by python/rlds_dataset_builder/ur5_unity/ur5_unity_dataset_builder.py.
//
// Output: ur5_simulation/Exports/dataset/<sessionName>[_<timestamp>]/
//   poses.csv     — one row per sampled frame (columns below)
//   images/NNNNNN.png — RGB capture per frame, indexed by `frame`
//   blocks.csv    — world position of each SceneSetup.targets at episode start
//
// Frame: Unity world (left-handed, Y-up, Z-forward). Positions in meters.
// Rotations are Hamiltonian quaternions (x,y,z,w) straight from Transform.rotation.
//
// poses.csv columns:
//   frame           — zero-based int index
//   Timestamp       — Unity Time.time seconds at capture (F4)
//   delta_pos_x/y/z — world-frame meters; pos[i+1] - pos[i]. Last row = 0.
//   delta_rot_x/y/z/w — world-frame quaternion delta (xyzw);
//                      rot[i+1] * Inverse(rot[i]) so R_next = R_delta * R_curr.
//                      Last row = identity.
//   suctionOn       — bool serialized as "True"/"False".
//   joint_0..joint_(JointCount-1) — joint angles in DEGREES (F5).
//   blockAttracted  — bool, true while suction is holding a block.
//   is_first        — 1 only on frame==0, else 0.
//   is_last, is_terminal — 1 only on the final row (successful-demo convention).
//   reward          — 1.0 only on the final row, else 0.0.
//   discount        — 1.0 on every row.
//
// Timing: capture is gated by recordingFPS (default 10 Hz) and quantized to Unity's
// render tick.
public class EpisodeRecorder : MonoBehaviour
{
    private static readonly string PoseHeader = BuildPoseHeader();

    [Header("Recording Settings")] public bool recordOnStart = false;

    public float recordingFPS = 10f;

    [Header("References")] [FormerlySerializedAs("observation")] [SerializeField]
    private ObservationCapture observationCapture;

    [Header("Export Settings")] public string sessionName = "recording";

    // Buffer rows (without deltas) — deltas and is_last/is_terminal/reward applied at flush
    private readonly List<string> frameBuffer = new();

    // Per-frame pose for delta calculation at flush time
    private readonly List<Vector3> framePositions = new();
    private readonly List<Quaternion> frameRotations = new();
    private int frameIndex = 0;
    private bool isPaused = false;

    // Runtime state
    private bool isRecording = false;
    private float nextSampleTime = 0f;

    private SceneSetup sceneSetup;
    private string sessionFolder;

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
        nextSampleTime = Time.time + 1f / recordingFPS;
    }

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

    // starts recording and returns session folder
    public string StartRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("EpisodeRecorder: Recording already in progress.");
            return sessionFolder;
        }

        observationCapture.Initialize();

        // Create a new episode folder
        string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        string folderName = $"{sessionName}_{timestamp}";
        sessionFolder = ProjectPaths.Get(
            Path.Combine("ur5_simulation", "Exports", "dataset", folderName)
        );

        Directory.CreateDirectory(Path.Combine(sessionFolder, "images"));

        // save starting block positions
        SaveBlockPositions();

        // reset state
        frameBuffer.Clear();
        framePositions.Clear();
        frameRotations.Clear();
        frameIndex = 0;
        nextSampleTime = Time.time;
        isRecording = true;

        return sessionFolder;
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

    public static void DeleteRecording(string folder)
    {
        Directory.Delete(folder, true);
        Debug.Log($"EpisodeRecorder: Deleted trajectory {folder}");
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

    /// <summary>Writes blocks.csv with the name and world position of each target block at episode start.</summary>
    private void SaveBlockPositions()
    {
        if (sceneSetup == null || sceneSetup.targets == null || sceneSetup.targets.Length == 0)
            return;

        string blocksPath = Path.Combine(sessionFolder, "blocks.csv");
        using var writer = new StreamWriter(blocksPath, false);
        writer.WriteLine("name,pos_x,pos_y,pos_z");
        foreach (GameObject block in sceneSetup.targets)
        {
            if (block == null)
                continue;
            Vector3 p = block.transform.position;
            writer.WriteLine($"{block.name},{p.x:F5},{p.y:F5},{p.z:F5}");
        }
    }

    private void CaptureFrame()
    {
        ObservationCapture.Observation obs = observationCapture.CaptureState();

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
        byte[] png = observationCapture.CaptureScreenshotPng();
        string imagePath = Path.Combine(sessionFolder, "images", $"{index:D6}.png");
        Task.Run(() => File.WriteAllBytes(imagePath, png));
    }

    private void FlushBufferToDisk()
    {
        if (frameBuffer.Count == 0)
            return;

        string posesPath = Path.Combine(sessionFolder, "poses.csv");
        using (var writer = new StreamWriter(posesPath, false))
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
                string prefix = row[..(secondComma + 1)];
                string rest = row[(secondComma + 1)..];

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