using UnityEngine;
using System.Collections.Generic;

public class SceneResetController : MonoBehaviour
{
    private class SceneObjectInitialState
    {
        public Transform parent;
        public Vector3 position;
        public Quaternion rotation;
    }

    [SerializeField] private float positionRandomRadius = 0.05f;

    private SceneSetup sceneSetup;
    private readonly Dictionary<GameObject, SceneObjectInitialState> initialTargetStates
        = new();

    public void Initialize(SceneSetup setup)
    {
        sceneSetup = setup;

        initialTargetStates.Clear();

        if (sceneSetup == null || sceneSetup.targets == null) return;

        foreach (GameObject target in sceneSetup.targets)
        {
            if (target == null) continue;

            initialTargetStates[target] = new SceneObjectInitialState
            {
                parent = target.transform.parent,
                position = target.transform.position,
                rotation = target.transform.rotation
            };
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
            ResetSceneState();
    }

    public void ResetSceneState()
    {
        if (sceneSetup == null || sceneSetup.targets == null || sceneSetup.robots == null)
        {
            Debug.LogWarning("Scene reset skipped: scene references are not initialized.");
            return;
        }

        foreach (GameObject robot in sceneSetup.robots)
        {
            if (robot == null) continue;

            SuctionController robotSuction = robot.GetComponent<SuctionController>();
            if (robotSuction != null)
            {
                robotSuction.ResetSuctionState();
            }
        }

        // Collect valid targets and their initial states
        List<GameObject> validTargets = new List<GameObject>();
        List<SceneObjectInitialState> states = new List<SceneObjectInitialState>();
        foreach (GameObject target in sceneSetup.targets)
        {
            if (target == null || !initialTargetStates.TryGetValue(target, out SceneObjectInitialState state)) continue;
            validTargets.Add(target);
            states.Add(state);
        }

        // Fisher-Yates shuffle: permute which gear goes to which initial slot
        for (int i = states.Count - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            (states[i], states[j]) = (states[j], states[i]);
        }

        // Place each target at its assigned (shuffled) slot with a random XZ offset
        for (int i = 0; i < validTargets.Count; i++)
        {
            SceneObjectInitialState state = states[i];
            Vector2 offset = Random.insideUnitCircle * positionRandomRadius;

            validTargets[i].transform.SetParent(state.parent, true);
            validTargets[i].transform.position = state.position + new Vector3(offset.x, 0f, offset.y);
            validTargets[i].transform.rotation = state.rotation;

            Rigidbody rb = validTargets[i].GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.isKinematic = true;
            }
        }

        if (sceneSetup.agentRobotController != null)
        {
            foreach (GameObject robot in sceneSetup.robots)
            {
                if (robot != null)
                {
                    sceneSetup.agentRobotController.ResetToHomePosition(robot);
                }
            }
        }
        else
        {
            foreach (GameObject robot in sceneSetup.robots)
            {
                if (robot == null) continue;

                RobotArmSetup armSetup = robot.GetComponent<RobotArmSetup>();
                if (armSetup != null)
                {
                    armSetup.ResetArmPosition();
                }
            }
        }

        Debug.Log("Scene reset complete: gears restored and robot arms homed.");
    }
}
