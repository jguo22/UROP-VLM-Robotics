using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using static ConstantsUR5;

[DefaultExecutionOrder(100)]
public abstract class AgenticSocketBase : MonoBehaviour
{
    protected const float IdleVelocityThreshold = 0.05f;
    private readonly object lockObject = new();

    private readonly Queue<Action> mainThreadActions = new();
    private readonly byte[] receiveBuffer = new byte[4096];
    private TcpClient client;

    private string hostAddress;

    private bool isSocketActive = false;
    private TcpListener listener;
    private int portNumber;

    protected EpisodeRecorder recorder;
    protected RobotArmSetup[] robotArmSetups;
    protected int robotCount;
    protected GameObject[] robots;
    protected SceneSetup sceneSetup;
    private NetworkStream stream;
    protected SuctionController[] suctionControllers;
    protected int targetCount;
    protected GameObject[] targets;

    protected UR5Controller[] ur5Controllers;

    private void Start()
    {
        InitializeScene();
        ReadEnv();
        StartSocket();
    }

    private void Update()
    {
        lock (lockObject)
        {
            while (mainThreadActions.Count > 0)
            {
                Action action = mainThreadActions.Dequeue();
                action.Invoke();
            }
        }
    }

    private void OnApplicationQuit()
    {
        closeSocket();
    }

    protected void InitializeScene()
    {
        sceneSetup = GetComponent<SceneSetup>();

        robots = sceneSetup.robots;
        targets = sceneSetup.targets;
        robotCount = robots.Length;
        targetCount = targets.Length;

        robotArmSetups = new RobotArmSetup[robotCount];
        suctionControllers = new SuctionController[robotCount];
        ur5Controllers = new UR5Controller[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            robotArmSetups[i] = robots[i].GetComponent<RobotArmSetup>();
            suctionControllers[i] = robots[i].GetComponent<SuctionController>();
            ur5Controllers[i] = robots[i].GetComponent<UR5Controller>();
        }

        recorder = GetComponent<EpisodeRecorder>();
        if (recorder == null) Debug.LogWarning("No EpisodeRecorder");
    }

    private void StartSocket()
    {
        try
        {
            listener = new TcpListener(IPAddress.Parse(hostAddress), portNumber);
            listener.Start();
            Debug.Log("Unity server started, waiting for Python connection...");

            client = listener.AcceptTcpClient();
            stream = client.GetStream();
            Debug.Log("Python connected!");

            isSocketActive = true;

            stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
        }
        catch (Exception e)
        {
            Debug.LogError("Socket error: " + e.Message);
        }
    }

    private void ReadEnv()
    {
        string envFile = ProjectPaths.Get("socket.env");
        if (!File.Exists(envFile))
        {
            Debug.LogError($"socket.env not found at {envFile}");
            return;
        }

        foreach (string line in File.ReadAllLines(envFile))
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("#")) continue;

            string[] parts = line.Split('=', 2);
            if (parts.Length == 2)
            {
                string key = parts[0].Trim();
                string value = parts[1].Trim().Trim('"');

                Environment.SetEnvironmentVariable(key, value);
            }
        }

        hostAddress = Environment.GetEnvironmentVariable("HOST");
        portNumber = int.Parse(Environment.GetEnvironmentVariable("PORT"));

        Debug.Log($"{hostAddress} | {portNumber}");
    }

    private void ReceiveCallback(IAsyncResult AR)
    {
        try
        {
            int bytesRead = stream.EndRead(AR);
            if (bytesRead > 0)
            {
                string receivedData = Encoding.UTF8.GetString(receiveBuffer, 0, bytesRead);
                Debug.Log("Received from Python: " + receivedData);

                lock (lockObject)
                {
                    mainThreadActions.Enqueue(() =>
                    {
                        string response = ProcessAICommand(receivedData);
                        byte[] responseData = Encoding.UTF8.GetBytes(response);
                        stream.Write(responseData, 0, responseData.Length);
                        Debug.Log("Sent to Python: " + response);
                    });
                }

                stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
            }
        }
        catch (Exception e)
        {
            if (isSocketActive)
                Debug.LogError("Receive callback error: " + e.Message);
        }
    }

    protected abstract string ProcessAICommand(string commandJson);

    protected string TryProcessCommonCommand(string type)
    {
        switch (type)
        {
            case "get_scene_state":
                return GetSceneStateJson();

            case "reset_scene":
                if (recorder != null)
                    recorder.StopRecording();
                sceneSetup.ResetSceneState();
                return JsonUtility.ToJson(
                    new ControlResponse
                    {
                        success = true,
                        message = "scene_reset",
                        timestamp = Time.time
                    }
                );

            case "start_recording":
                if (recorder != null)
                    recorder.StartRecording();
                return JsonUtility.ToJson(
                    new ControlResponse
                    {
                        success = true,
                        message = "recording_started",
                        timestamp = Time.time
                    }
                );

            case "stop_recording":
                if (recorder != null)
                    recorder.StopRecording();
                return JsonUtility.ToJson(
                    new ControlResponse
                    {
                        success = true,
                        message = "recording_stopped",
                        timestamp = Time.time
                    }
                );

            case "get_motion_status":
                bool idle = IsAllRobotsIdle();
                return JsonUtility.ToJson(
                    new MotionStatusResponse
                    {
                        success = true,
                        is_idle = idle,
                        timestamp = Time.time
                    }
                );

            default:
                return null;
        }
    }

    private string GetSceneStateJson()
    {
        try
        {
            var sceneState = new SceneStateData
            {
                robots = GetRobotStates(),
                timestamp = Time.time
            };

            return JsonUtility.ToJson(sceneState);
        }
        catch (Exception e)
        {
            Debug.LogError("Error getting scene state: " + e.Message);
            return JsonUtility.ToJson(
                new AIResponse
                {
                    success = false,
                    message = "Error getting scene state: " + e.Message
                }
            );
        }
    }

    protected RobotStateData[] GetRobotStates()
    {
        var robotStates = new RobotStateData[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            GameObject robot = robots[i];
            RobotArmSetup robotArmSetup = robotArmSetups[i];
            SuctionController suctionController = suctionControllers[i];

            var targetObjects = suctionController.targetBlocks;
            var robotTargets = new ObjectStateData[targetObjects.Length];
            for (int j = 0; j < targetObjects.Length; j++)
            {
                GameObject target = targetObjects[j];
                Vector3 targetPosition = target.GetComponent<Collider>().bounds.center;

                robotTargets[j] = new ObjectStateData
                {
                    name = target.name,
                    position = new[] { targetPosition.x, targetPosition.y, targetPosition.z },
                    is_attached = target.transform.parent != null
                };
            }

            Transform endEffector = robotArmSetup
                .robotJoints[^1]
                .transform;
            robotStates[i] = new RobotStateData
            {
                name = robot.name,
                end_effector_position = new[]
                {
                    endEffector.position.x,
                    endEffector.position.y,
                    endEffector.position.z
                },
                objects = robotTargets,
                suction_active = suctionController != null && suctionController.enableSuction
            };
        }

        return robotStates;
    }

    protected float[] GetJointAngles(ArticulationBody[] robotJoints)
    {
        float[] angles = new float[JointCount];

        for (int i = 0; i < angles.Length && i < robotJoints.Length; i++) angles[i] = robotJoints[i].jointPosition[0];
        return angles;
    }

    protected ObjectStateData[] GetObjectStates()
    {
        var objects = new ObjectStateData[targetCount];

        for (int i = 0; i < targetCount; i++)
        {
            GameObject target = targets[i];
            Vector3 targetPosition = target.GetComponent<Collider>().bounds.center;

            objects[i] = new ObjectStateData
            {
                name = target.name,
                position = new[] { targetPosition.x, targetPosition.y, targetPosition.z },
                is_attached = target.transform.parent != null
            };
        }

        return objects;
    }

    protected bool ExecuteActivateSuction(GameObject robot)
    {
        var suctionController = robot.GetComponent<SuctionController>();
        if (suctionController == null)
            return false;

        suctionController.ToggleSuction();
        return true;
    }

    protected bool ExecuteDeactivateSuction(GameObject robot)
    {
        var suctionController = robot.GetComponent<SuctionController>();
        if (suctionController == null)
            return false;

        if (suctionController.enableSuction) suctionController.ToggleSuction();
        return true;
    }

    protected bool IsAllRobotsIdle()
    {
        bool isIdle = true;
        foreach (GameObject robot in robots)
        {
            var setup = robot.GetComponent<RobotArmSetup>();
            if (setup == null)
                continue;

            float[] vels = new float[JointCount];
            bool robotBusy = false;
            for (int i = 0; i < JointCount; i++)
            {
                vels[i] = setup.robotJoints[i].jointVelocity[0];
                if (Mathf.Abs(vels[i]) > IdleVelocityThreshold)
                    robotBusy = true;
            }

            if (robotBusy)
            {
                string velStr = string.Join(", ", vels);
                print($"{robot.name} vels=[{velStr}]");
                isIdle = false;
            }
        }

        return isIdle;
    }

    public void closeSocket()
    {
        isSocketActive = false;
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
        if (listener != null)
            listener.Stop();
    }
}