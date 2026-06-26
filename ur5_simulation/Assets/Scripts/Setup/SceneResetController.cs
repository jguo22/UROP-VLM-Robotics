using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SceneResetController : MonoBehaviour
{
    [SerializeField] private float positionRandomRadius = 0.03f;
    [SerializeField] private bool permutationRandomization = false;
    [SerializeField] private bool offsetRandomization = false;

    private readonly List<SceneObjectInitialState> initialStates = new();
    private readonly List<RobotArmSetup> robotArmSetups = new();
    private readonly List<SuctionController> suctionControllers = new();

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
            ResetSceneState();
    }

    public void Initialize(SceneSetup sceneSetup)
    {
        // initialize block locations
        foreach (GameObject target in sceneSetup.targets)
            initialStates.Add(new SceneObjectInitialState
            {
                transform = target.transform,
                rigidbody = target.GetComponent<Rigidbody>(),
                parent = target.transform.parent,
                position = target.transform.position,
                rotation = target.transform.rotation
            });

        // initialize robots
        foreach (GameObject robot in sceneSetup.robots)
        {
            suctionControllers.Add(robot.GetComponent<SuctionController>());
            robotArmSetups.Add(robot.GetComponent<RobotArmSetup>());
        }
    }

    public void ResetSceneState()
    {
        ResetRobots();
        ResetObjects();
    }

    private void ResetRobots()
    {
        foreach (SuctionController suctionController in suctionControllers) suctionController.ResetSuctionState();
        foreach (RobotArmSetup robotArmSetup in robotArmSetups) robotArmSetup.ResetArmPosition();
    }

    // Reset the objects that are being moved around
    private void ResetObjects()
    {
        // assignments for permutation randomization
        var assignments = Enumerable.Range(0, initialStates.Count).ToList();
        // Fisher-Yates shuffle: permute which gear goes to which initial slot
        if (permutationRandomization)
            for (int i = assignments.Count - 1; i > 0; i--)
            {
                int j = Random.Range(0, i + 1);
                (assignments[i], assignments[j]) = (assignments[j], assignments[i]);
            }

        // Place each target at its assigned (shuffled) slot with a random XZ offset
        for (int i = 0; i < assignments.Count; i++)
        {
            SceneObjectInitialState state = initialStates[i];

            // stop movement
            Rigidbody rb = state.rigidbody;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            // rb.isKinematic = true; // freeze during teleport to prevent physics glitch

            // reset parent and rotation
            Transform objectTransform = state.transform;
            objectTransform.SetParent(state.parent, true);
            objectTransform.rotation = state.rotation;

            // reset position to randomized position
            Vector3 position = initialStates[assignments[i]].position;
            if (offsetRandomization)
            {
                Vector2 offset = Random.insideUnitCircle * positionRandomRadius;
                position += new Vector3(offset.x, 0f, offset.y);
            }

            objectTransform.position = position;
        }
    }

    private class SceneObjectInitialState
    {
        public Transform parent;
        public Vector3 position;
        public Rigidbody rigidbody;
        public Quaternion rotation;
        public Transform transform;
    }
}