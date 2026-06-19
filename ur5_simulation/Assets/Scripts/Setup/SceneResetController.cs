using System.Collections.Generic;
using UnityEngine;

public class SceneResetController : MonoBehaviour
{
    [SerializeField] private float positionRandomRadius = 0.05f;

    private readonly Dictionary<GameObject, SceneObjectInitialState> initialTargetStates = new();

    private SceneSetup sceneSetup;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
            ResetSceneState();
    }

    public void Initialize(SceneSetup setup)
    {
        sceneSetup = setup;

        initialTargetStates.Clear();

        if (sceneSetup == null || sceneSetup.targets == null)
            return;

        foreach (GameObject target in sceneSetup.targets)
        {
            if (target == null)
                continue;

            initialTargetStates[target] = new SceneObjectInitialState
            {
                parent = target.transform.parent,
                position = target.transform.position,
                rotation = target.transform.rotation
            };
        }
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
            if (!robot)
                continue;

            robot.GetComponent<SuctionController>().ResetSuctionState();
            robot.GetComponent<RobotArmSetup>().ResetArmPosition();
        }

        // Collect valid targets and their initial states
        List<GameObject> validTargets = new();
        List<SceneObjectInitialState> states = new();
        foreach (GameObject target in sceneSetup.targets)
        {
            if (
                target == null
                || !initialTargetStates.TryGetValue(target, out SceneObjectInitialState state)
            )
                continue;
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

            var rb = validTargets[i].GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
                rb.isKinematic = true; // freeze during teleport to prevent physics glitch
            }

            validTargets[i].transform.SetParent(state.parent, true);
            validTargets[i].transform.position =
                state.position + new Vector3(offset.x, 0f, offset.y);
            validTargets[i].transform.rotation = state.rotation;

            if (rb != null)
                rb.isKinematic = false; // release physics so attraction force and gravity work
        }

        Debug.Log("Scene reset complete: gears restored and robot arms homed.");
    }

    private class SceneObjectInitialState
    {
        public Transform parent;
        public Vector3 position;
        public Quaternion rotation;
    }
}