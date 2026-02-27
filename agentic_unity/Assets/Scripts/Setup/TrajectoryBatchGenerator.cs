using UnityEngine;
using System.Collections;

public class TrajectoryBatchGenerator : MonoBehaviour
{
    [Header("Trajectory Batch Generation")]
    [SerializeField] private PoseAndImageRecorder[] trajectoryRecorders;
    [SerializeField] private int defaultTrajectoryCount = 10;
    [SerializeField] private float settleAfterResetSeconds = 0.25f;
    [SerializeField] private float betweenTrajectoryDelaySeconds = 0.25f;
    [SerializeField] private float maxTrajectoryRunSeconds = 10f;
    [SerializeField] private float minTrajectoryRunSeconds = 0.5f;
    [SerializeField] private float settleDistanceThreshold = 0.0005f;
    [SerializeField] private float requiredSettledSeconds = 0.5f;

    private SceneSetup sceneSetup;
    private Coroutine trajectoryBatchRoutine;

    public void Initialize(SceneSetup setup)
    {
        sceneSetup = setup;
    }

    public void GenerateMultipleTrajectories()
    {
        GenerateMultipleTrajectories(defaultTrajectoryCount);
    }

    public void GenerateMultipleTrajectories(int trajectoryCount)
    {
        if (sceneSetup == null)
        {
            Debug.LogWarning("Trajectory generation skipped: SceneSetup is not initialized.");
            return;
        }

        if (trajectoryCount <= 0)
        {
            Debug.LogWarning("Trajectory generation skipped: trajectoryCount must be greater than zero.");
            return;
        }

        if (trajectoryBatchRoutine != null)
        {
            Debug.LogWarning("Trajectory generation is already running.");
            return;
        }

        trajectoryBatchRoutine = StartCoroutine(GenerateMultipleTrajectoriesRoutine(trajectoryCount));
    }

    private IEnumerator GenerateMultipleTrajectoriesRoutine(int trajectoryCount)
    {
        EnsureTrajectoryRecordersAreAssigned();

        for (int i = 0; i < trajectoryCount; i++)
        {
            Debug.Log($"Starting trajectory {i + 1}/{trajectoryCount}.");

            sceneSetup.ResetSceneState();
            yield return new WaitForSeconds(settleAfterResetSeconds);

            StartTrajectoryRecording();
            yield return StartCoroutine(RunAgenticAIGearMoveSequence());
            StopSceneAfterTrajectory();
            StopTrajectoryRecording();

            Debug.Log($"Completed trajectory {i + 1}/{trajectoryCount}.");
            yield return new WaitForSeconds(betweenTrajectoryDelaySeconds);
        }

        trajectoryBatchRoutine = null;
        Debug.Log($"Trajectory batch complete. Generated {trajectoryCount} trajectories.");
    }

    private void EnsureTrajectoryRecordersAreAssigned()
    {
        if (trajectoryRecorders != null && trajectoryRecorders.Length > 0) return;

        trajectoryRecorders = FindObjectsByType<PoseAndImageRecorder>(FindObjectsSortMode.None);
        if (trajectoryRecorders == null || trajectoryRecorders.Length == 0)
        {
            Debug.LogWarning("No PoseAndImageRecorder components found. Trajectories will run without recording.");
        }
    }

    private void StartTrajectoryRecording()
    {
        if (trajectoryRecorders == null) return;

        foreach (PoseAndImageRecorder recorder in trajectoryRecorders)
        {
            if (recorder != null) recorder.StartRecording();
        }
    }

    private void StopTrajectoryRecording()
    {
        if (trajectoryRecorders == null) return;

        foreach (PoseAndImageRecorder recorder in trajectoryRecorders)
        {
            if (recorder != null) recorder.StopRecording();
        }
    }

    private IEnumerator RunAgenticAIGearMoveSequence()
    {
        // Wait for activity window, then require robot end-effectors to settle for
        // a short duration so recordings end after motion completes.
        float elapsed = 0f;
        float settledFor = 0f;
        bool movementObserved = false;
        Vector3[] previousEndEffectorPositions = GetEndEffectorPositions();

        while (elapsed < maxTrajectoryRunSeconds)
        {
            yield return null;
            elapsed += Time.deltaTime;

            Vector3[] currentEndEffectorPositions = GetEndEffectorPositions();
            float maxDelta = GetMaxEndEffectorDelta(previousEndEffectorPositions, currentEndEffectorPositions);
            previousEndEffectorPositions = currentEndEffectorPositions;
            if (maxDelta > settleDistanceThreshold) movementObserved = true;

            if (elapsed < minTrajectoryRunSeconds) continue;
            if (!movementObserved) continue;

            if (maxDelta <= settleDistanceThreshold)
            {
                settledFor += Time.deltaTime;
                if (settledFor >= requiredSettledSeconds) break;
            }
            else
            {
                settledFor = 0f;
            }
        }

        if (!movementObserved)
        {
            Debug.LogWarning("Trajectory run ended without observed robot motion.");
        }
    }

    private void StopSceneAfterTrajectory()
    {
        if (sceneSetup == null || sceneSetup.robots == null) return;

        if (sceneSetup.agentRobotController != null && sceneSetup.agentRobotController.IsCSVPlaying())
        {
            sceneSetup.agentRobotController.StopCSVPlayback();
        }

        foreach (GameObject robot in sceneSetup.robots)
        {
            if (robot == null) continue;

            SuctionController suctionController = robot.GetComponent<SuctionController>();
            if (suctionController != null)
            {
                suctionController.SetSuctionState(false);
            }

            UnifiedRobotController unifiedController = robot.GetComponent<UnifiedRobotController>();
            if (unifiedController != null)
            {
                unifiedController.currentMode = UnifiedRobotController.ControlMode.Start;
            }
        }
    }

    private Vector3[] GetEndEffectorPositions()
    {
        if (sceneSetup == null || sceneSetup.robots == null) return new Vector3[0];

        Vector3[] positions = new Vector3[sceneSetup.robots.Length];
        for (int i = 0; i < sceneSetup.robots.Length; i++)
        {
            GameObject robot = sceneSetup.robots[i];
            if (robot == null)
            {
                positions[i] = Vector3.zero;
                continue;
            }

            RobotArmSetup robotArmSetup = robot.GetComponent<RobotArmSetup>();
            if (robotArmSetup == null || robotArmSetup.robotJoints == null || robotArmSetup.robotJoints.Length == 0)
            {
                positions[i] = robot.transform.position;
                continue;
            }

            positions[i] = robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform.position;
        }

        return positions;
    }

    private float GetMaxEndEffectorDelta(Vector3[] previousPositions, Vector3[] currentPositions)
    {
        if (previousPositions == null || currentPositions == null) return float.MaxValue;

        int count = Mathf.Min(previousPositions.Length, currentPositions.Length);
        float maxDelta = 0f;

        for (int i = 0; i < count; i++)
        {
            float delta = Vector3.Distance(previousPositions[i], currentPositions[i]);
            if (delta > maxDelta) maxDelta = delta;
        }

        return maxDelta;
    }

    private void OnDisable()
    {
        if (trajectoryBatchRoutine != null)
        {
            StopCoroutine(trajectoryBatchRoutine);
            trajectoryBatchRoutine = null;
            StopTrajectoryRecording();
        }
    }
}
