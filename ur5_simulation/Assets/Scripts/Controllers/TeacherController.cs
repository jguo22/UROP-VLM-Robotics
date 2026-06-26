using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class TeacherController : MonoBehaviour
{
    private const float ToleranceDistance = 0.01f;
    private const float MaxDelta = 0.1f;
    private const float FrameTime = 0.1f;
    private const float MaxTime = 3f;
    private const float IdleVelocityThreshold = 0.005f;
    private const float ApproachDistance = 0.08f;
    private const float CollisionDistance = 0.01f;
    private const float EndingToleranceDistance = 0.15f;

    private const float EndYLevel = 0.875f;

    private static readonly Dictionary<string, Vector3> GoalPositions =
        new()
        {
            { "gear_grey", new Vector3(0.83146f, EndYLevel, -0.05706f) },
            { "gear_green", new Vector3(0.83146f, EndYLevel, 0.05706f) },
            { "gear_blue", new Vector3(0.79f, EndYLevel, 0.0f) },
            { "gear_orange", new Vector3(0.89854f, EndYLevel, 0.03527f) },
            { "gear_red", new Vector3(0.89854f, EndYLevel, -0.03527f) }
        };

    [SerializeField] private EpisodeRecorder recorder;
    [SerializeField] private UR5Controller controller;
    [SerializeField] private SceneSetup sceneSetup;
    [SerializeField] private SuctionController suctionController;

    private readonly Vector3 approachOffset = Vector3.up * ApproachDistance;
    private readonly Vector3 collisionOffset = Vector3.up * CollisionDistance;
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

            print("starting episode " + episode);
            string sessionFolder = recorder.StartRecording();
            yield return RunEpisodeCR();
            recorder.StopRecording();

            if (!episodeSuccess) EpisodeRecorder.DeleteRecording(sessionFolder);

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
                Debug.LogError("Waiting for idle failed. Episode fail.");
                episodeSuccess = false;
                yield break;
            }

            yield return new WaitForSeconds(FrameTime);
        }
    }

    private IEnumerator RunEpisodeCR()
    {
        var gearPositions = GetGearPositions();
        string[] gearNames = { "gear_blue", "gear_red", "gear_green" };
        foreach (string gearName in gearNames)
        {
            Vector3 initialPosition = gearPositions[gearName];
            Vector3 goalPosition = GoalPositions[gearName];

            yield return MoveArmCR(initialPosition + approachOffset);
            suctionController.SetSuctionState(true);
            yield return MoveArmCR(initialPosition + collisionOffset);
            yield return MoveArmCR(initialPosition + approachOffset);
            yield return MoveArmCR(goalPosition + approachOffset);
            yield return MoveArmCR(goalPosition + collisionOffset);
            suctionController.SetSuctionState(false);
            yield return MoveArmCR(goalPosition + approachOffset);
        }

        // verify that they are in the correct position
        gearPositions = GetGearPositions();
        foreach (string gearName in gearNames)
        {
            float distance = Vector3.Distance(gearPositions[gearName], GoalPositions[gearName]);
            Debug.Log(
                $"Distance From Goal {gearName}: {distance}, gear position {gearPositions[gearName]}, goal position {GoalPositions[gearName]}");

            if (distance > EndingToleranceDistance)
            {
                Debug.LogError($"Gear {gearName} in wrong spot. Episode fail.");
                episodeSuccess = false;
            }
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

    private Dictionary<string, Vector3> GetGearPositions()
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