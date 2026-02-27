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

    private SceneSetup sceneSetup;
    private readonly Dictionary<GameObject, SceneObjectInitialState> initialTargetStates = new Dictionary<GameObject, SceneObjectInitialState>();

    public void Initialize(SceneSetup setup)
    {
        sceneSetup = setup;
        CaptureInitialSceneState();
    }

    private void CaptureInitialSceneState()
    {
        initialTargetStates.Clear();

        if (sceneSetup == null || sceneSetup.targets == null) return;

        foreach (GameObject target in sceneSetup.targets)
        {
            if (target == null) continue;

            Rigidbody targetRigidbody = target.GetComponent<Rigidbody>();
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
            if (robot == null) continue;

            SuctionController robotSuction = robot.GetComponent<SuctionController>();
            if (robotSuction != null)
            {
                robotSuction.ResetSuctionState();
            }
        }

        foreach (GameObject target in sceneSetup.targets)
        {
            if (target == null || !initialTargetStates.TryGetValue(target, out SceneObjectInitialState state)) continue;

            target.transform.SetParent(state.parent, true);
            target.transform.position = state.position;
            target.transform.rotation = state.rotation;

            Rigidbody targetRigidbody = target.GetComponent<Rigidbody>();
            if (targetRigidbody != null)
            {
                targetRigidbody.linearVelocity = Vector3.zero;
                targetRigidbody.angularVelocity = Vector3.zero;
                targetRigidbody.isKinematic = true;
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
