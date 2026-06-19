using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class TeacherController : MonoBehaviour
{
    private const float ToleranceDistance = 0.005f;
    private const float MaxDelta = 0.1f;
    private const float FrameTime = 0.1f;
    private const float MaxTime = 3f;
    private const float IdleVelocityThreshold = 0.002f;

    private static readonly Dictionary<string, Vector3> GoalGearPositions =
        new()
        {
            { "gear_grey", new Vector3(0.83146f, 0.89f, -0.05706f) },
            { "gear_green", new Vector3(0.83146f, 0.89f, 0.05706f) },
            { "gear_blue", new Vector3(0.79f, 0.89f, 0.0f) },
            { "gear_orange", new Vector3(0.89854f, 0.89f, 0.03527f) },
            { "gear_red", new Vector3(0.89854f, 0.89f, -0.03527f) }
        };

    [SerializeField] private EpisodeRecorder recorder;

    [SerializeField] private UR5Controller controller;

    [SerializeField] private SceneSetup sceneSetup;

    [SerializeField] private SuctionController suctionController;

    private readonly Vector3 approachOffset = Vector3.up * 0.05f;
    private readonly Quaternion eeDownRotation = Quaternion.Euler(180, 0, 0);

    private bool episodeSuccess = true;

    private void Start()
    {
        StartCoroutine(RunLoopCR());
    }

    // ReSharper disable Unity.PerformanceAnalysis
    private IEnumerator RunLoopCR()
    {
        int episode = 0;
        for (int i = 0; i < 100; i++)
        {
            episodeSuccess = true;

            sceneSetup.ResetSceneState();
            yield return new WaitForSeconds(1);
            yield return WaitForIdle();
            print(string.Join(", ", controller.GetJointAngles()));

            print("starting episode " + episode);
            string sessionFolder = recorder.StartRecording();
            yield return RunEpisodeCR();
            recorder.StopRecording();

            if (!episodeSuccess) recorder.DeleteRecording(sessionFolder);

            episode++;
        }
    }

    private IEnumerator WaitForIdle()
    {
        yield return new WaitForSeconds(FrameTime);
        for (int i = 0;; i++)
        {
            float[] vels = controller.GetJointVelocities();
            bool isIdle = vels.All(vel => vel <= IdleVelocityThreshold);

            if (isIdle)
                yield break;

            if (i * FrameTime > MaxTime)
            {
                episodeSuccess = false;
                yield break;
            }

            yield return new WaitForSeconds(FrameTime);
        }
    }

    private IEnumerator RunEpisodeCR()
    {
        var gearData = GetGearData();
        print(string.Join(", ", gearData.Select(kv => $"{kv.Key}: {kv.Value}")));
        string[] gearNames = { "gear_blue", "gear_red", "gear_green" };
        foreach (string gearName in gearNames)
        {
            Vector3 initialPosition = gearData[gearName];
            Vector3 goalPosition = GoalGearPositions[gearName];

            print(initialPosition.ToString("F6"));
            yield return MoveArmCR(initialPosition + approachOffset);
            suctionController.SetSuctionState(true);
            print(controller.GetEndEffectorPoseWorld().position.ToString("F6"));
            print(initialPosition.ToString("F6"));
            yield return MoveArmCR(initialPosition);
            yield return MoveArmCR(initialPosition + approachOffset);
            yield return MoveArmCR(goalPosition + approachOffset);
            yield return MoveArmCR(goalPosition);
            suctionController.SetSuctionState(false);
            yield return MoveArmCR(goalPosition + approachOffset);
        }
    }

    private IEnumerator MoveArmCR(Vector3 targetPosition)
    {
        for (int i = 0;; i++)
        {
            Vector3 currentPosition = controller.GetEndEffectorPoseWorld().position;
            Vector3 delta = targetPosition - currentPosition;
            float distance = delta.magnitude;

            if (distance < ToleranceDistance)
            {
                yield break;
            }
            else if (i * FrameTime > MaxTime)
            {
                Debug.LogError("Moving arm to location failed. Episode fail.");
                episodeSuccess = false;
                yield break;
            }

            if (distance > MaxDelta)
                delta *= MaxDelta / distance;

            controller.MoveToTarget(currentPosition + delta, eeDownRotation);
            yield return new WaitForSeconds(FrameTime);
        }
    }

    private Dictionary<string, Vector3> GetGearData()
    {
        var result = new Dictionary<string, Vector3>();

        var targets = suctionController.targetBlocks;
        foreach (GameObject target in targets)
            // for some reason, the collider center isn't at the transform location
            // and is offset by like 0.04 on the x axis. very confusing.
            result[target.name] = target.GetComponent<Collider>().bounds.center;

        return result;
    }
}