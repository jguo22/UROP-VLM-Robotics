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
    protected TcpListener listener;
    protected TcpClient client;
    protected NetworkStream stream;
    protected byte[] receiveBuffer = new byte[4096];

    protected string hostAddress;
    protected int portNumber;

    protected EpisodeRecorder recorder;

    protected UR5Controller[] ur5Controllers;
    protected SceneSetup sceneSetup;
    protected GameObject[] robots;
    protected GameObject[] targets;
    protected RobotArmSetup[] robotArmSetups;
    protected SuctionController[] suctionControllers;
    protected int robotCount;
    protected int targetCount;

    private Queue<Action> mainThreadActions = new Queue<Action>();
    private object lockObject = new object();

    protected bool isSocketActive = false;

    protected const float IdleVelocityThreshold = 0.05f;

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
                var action = mainThreadActions.Dequeue();
                action.Invoke();
            }
        }
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
        if (recorder == null)
        {
            Debug.LogWarning("No EpisodeRecorder");
        }
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
        if (!System.IO.File.Exists(envFile))
        {
            Debug.LogError($"socket.env not found at {envFile}");
            return;
        }
        foreach (var line in File.ReadAllLines(envFile))
        {
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("#"))
            {
                continue;
            }

            var parts = line.Split('=', 2);
            if (parts.Length == 2)
            {
                var key = parts[0].Trim();
                var value = parts[1].Trim().Trim('"');

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
                        timestamp = Time.time,
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
                        timestamp = Time.time,
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
                        timestamp = Time.time,
                    }
                );

            case "get_motion_status":
                bool idle = IsAllRobotsIdle();
                return JsonUtility.ToJson(
                    new MotionStatusResponse
                    {
                        success = true,
                        is_idle = idle,
                        timestamp = Time.time,
                    }
                );

            default:
                return null;
        }
    }

    protected string GetSceneStateJson()
    {
        try
        {
            var sceneState = new SceneStateData
            {
                robots = GetRobotStates(),
                timestamp = Time.time,
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
                    message = "Error getting scene state: " + e.Message,
                }
            );
        }
    }

    protected RobotStateData[] GetRobotStates()
    {
        RobotStateData[] robotStates = new RobotStateData[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            var robot = robots[i];
            var robotArmSetup = robotArmSetups[i];
            var suctionController = suctionControllers[i];

            GameObject[] targetObjects = suctionController.targetBlocks;
            var robotTargets = new ObjectStateData[targetObjects.Length];
            for (int j = 0; j < targetObjects.Length; j++)
            {
                var target = targetObjects[j];
                Vector3 targetPosition = target.GetComponent<Collider>().bounds.center;

                robotTargets[j] = new ObjectStateData
                {
                    name = target.name,
                    position = new float[] { targetPosition.x, targetPosition.y, targetPosition.z },
                    is_attached = target.transform.parent != null,
                };
            }

            var endEffector = robotArmSetup
                .robotJoints[robotArmSetup.robotJoints.Length - 1]
                .transform;
            robotStates[i] = new RobotStateData
            {
                name = robot.name,
                end_effector_position = new float[]
                {
                    endEffector.position.x,
                    endEffector.position.y,
                    endEffector.position.z,
                },
                objects = robotTargets,
                suction_active = suctionController != null && suctionController.enableSuction,
            };
        }

        return robotStates;
    }

    protected float[] GetJointAngles(ArticulationBody[] robotJoints)
    {
        float[] angles = new float[JointCount];

        for (int i = 0; i < angles.Length && i < robotJoints.Length; i++)
        {
            angles[i] = robotJoints[i].jointPosition[0];
        }
        return angles;
    }

    protected ObjectStateData[] GetObjectStates()
    {
        ObjectStateData[] objects = new ObjectStateData[targetCount];

        for (int i = 0; i < targetCount; i++)
        {
            var target = targets[i];
            Vector3 targetPosition = target.GetComponent<Collider>().bounds.center;

            objects[i] = new ObjectStateData
            {
                name = target.name,
                position = new float[] { targetPosition.x, targetPosition.y, targetPosition.z },
                is_attached = target.transform.parent != null,
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

        if (suctionController.enableSuction)
        {
            suctionController.ToggleSuction();
        }
        return true;
    }

    protected bool IsAllRobotsIdle()
    {
        bool isIdle = true;
        foreach (GameObject robot in robots)
        {
            RobotArmSetup setup = robot.GetComponent<RobotArmSetup>();
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

    private void OnApplicationQuit()
    {
        closeSocket();
    }
}
