using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using static ConstantsUR5;

[DefaultExecutionOrder(100)]
public class SimulAgenticSocketSetup : MonoBehaviour
{
    private TcpListener listener;
    private TcpClient client;
    private NetworkStream stream;
    private byte[] receiveBuffer = new byte[4096]; // Increased buffer size

    //for socket connection
    private static string unityAssetsFilePath = "ur5_simulation/Assets";
    private string envFilePath = Application.dataPath.Replace(unityAssetsFilePath, "");
    private string hostAddress;
    private int portNumber;

    private EpisodeRecorder recorder;

    // AI Agent Integration
    private UR5Controller[] ur5Controllers;
    private SceneSetup sceneSetup;
    private GameObject[] robots;
    private GameObject[] targets;
    private RobotArmSetup[] robotArmSetups;
    private SuctionController[] suctionControllers;

    // private SimulAgentController[] simulAgentControllers;
    private int robotCount;
    private int targetCount;

    private Queue<Action> mainThreadActions = new Queue<Action>();
    private object lockObject = new object();

    //private string pendingResponse;
    //private bool isProcessingCommand;

    private bool isSocketActive = false;

    private bool toMoveArms = false;
    private bool toMoveLeftArm = false;
    private bool toMoveRightArm = false;

    [HideInInspector]
    public bool moveArmsNow = false;

    void Start()
    {
        // Initialize AI components
        sceneSetup = GetComponent<SceneSetup>();

        recorder = GetComponent<EpisodeRecorder>();
        if (recorder == null)
            recorder = gameObject.AddComponent<EpisodeRecorder>();

        //GET ALL REQUIRED SCENE DATA
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

        // Read environment variables for socket configuration
        ReadEnv();

        try
        {
            // Start listening for connections
            listener = new TcpListener(IPAddress.Parse(hostAddress), portNumber);
            listener.Start();
            Debug.Log("Unity server started, waiting for Python connection...");

            // Accept connection from Python
            client = listener.AcceptTcpClient();
            stream = client.GetStream();
            Debug.Log("Python connected!");

            isSocketActive = true;

            // Start receiving data
            stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
        }
        catch (Exception e)
        {
            Debug.LogError("Socket error: " + e.Message);
        }
    }

    void Update()
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

    private void ReadEnv()
    {
        Debug.Log(Application.dataPath);
        string envFile = Directory.GetFiles(envFilePath, "socket.env")[0];
        foreach (var line in File.ReadAllLines(envFile))
        {
            // Skip empty lines and comments
            if (string.IsNullOrWhiteSpace(line) || line.StartsWith("#"))
            {
                continue;
            }

            var parts = line.Split('=', 2); // Split only on the first '='
            if (parts.Length == 2)
            {
                var key = parts[0].Trim();
                var value = parts[1].Trim().Trim('"'); // Remove potential quotes

                // Set as environment variable
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

                // Queue the processing to main thread
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

                // // Process AI command
                // string response = ProcessAICommand(receivedData);

                // // Send response back to Python
                // byte[] responseData = Encoding.UTF8.GetBytes(response);
                // stream.Write(responseData, 0, responseData.Length);
                // Debug.Log("Sent to Python: " + response);

                // Continue listening for more data
                stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
            }
        }
        catch (Exception e)
        {
            if (isSocketActive)
                Debug.LogError("Receive callback error: " + e.Message);
        }
    }

    private string ProcessAICommand(string commandJson)
    {
        try
        {
            // Parse JSON command
            var command = JsonUtility.FromJson<SimulAICommand>(commandJson);

            switch (command.type)
            {
                case "get_scene_state":
                    return GetSceneStateJson();

                case "reset_scene":
                    sceneSetup.ResetSceneState();
                    return JsonUtility.ToJson(new ControlResponse {
                        success = true, message = "scene_reset", timestamp = Time.time });

                case "start_recording":
                    if (recorder != null) recorder.StartRecording();
                    return JsonUtility.ToJson(new ControlResponse {
                        success = true, message = "recording_started", timestamp = Time.time });

                case "stop_recording":
                    if (recorder != null) recorder.StopRecording();
                    return JsonUtility.ToJson(new ControlResponse {
                        success = true, message = "recording_stopped", timestamp = Time.time });

                case "get_motion_status":
                    bool idle = IsAllRobotsIdle();
                    return JsonUtility.ToJson(new MotionStatusResponse {
                        success = true, is_idle = idle, timestamp = Time.time });

                default: // simultaenous AI command
                    return ExecuteActionJson(command);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error processing AI command: " + e.Message);
            return JsonUtility.ToJson(
                new AIResponse
                {
                    success = false,
                    message = "Error processing command: " + e.Message,
                }
            );
        }
    }

    private string GetSceneStateJson()
    {
        try
        {
            var sceneState = new SceneStateData
            {
                robots = GetRobotStates(),
                // objects = GetObjectStates(),
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

    private RobotStateData[] GetRobotStates()
    {
        //if (robots == null) return new RobotStateData[0];

        RobotStateData[] robotStates = new RobotStateData[robotCount];

        //List<RobotStateData> robotStates = new List<RobotStateData>();

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
                    is_attached =
                        target.transform.parent
                        != null // && block.transform.parent.name.Contains("suction")
                    ,
                };
            }

            var endEffector = robotArmSetup
                .robotJoints[robotArmSetup.robotJoints.Length - 1]
                .transform;
            robotStates[i] = new RobotStateData
            {
                name = robot.name,
                // joint_angles = GetJointAngles(robotArmSetup.robotJoints),
                end_effector_position = new float[]
                {
                    endEffector.position.x,
                    endEffector.position.y,
                    endEffector.position.z,
                },
                objects = robotTargets,
                suction_active = suctionController != null && suctionController.enableSuction,
                //is_moving = unifiedController != null && unifiedController.currentMode != UnifiedRobotController.ControlMode.Start
            };

            //robotStates[i] = robotState;
        }

        return robotStates;
    }

    private float[] GetJointAngles(ArticulationBody[] robotJoints)
    {
        float[] angles = new float[JointCount]; //UR5 has 6 joints

        for (int i = 0; i < angles.Length && i < robotJoints.Length; i++)
        {
            angles[i] = robotJoints[i].jointPosition[0]; // radians
        }
        return angles;
    }

    private ObjectStateData[] GetObjectStates()
    {
        //List<ObjectStateData> objects = new List<ObjectStateData>();
        ObjectStateData[] objects = new ObjectStateData[targetCount];

        // Find all objects with specific tags or names
        //GameObject[] blocks = GameObject.FindGameObjectsWithTag("Block");

        for (int i = 0; i < targetCount; i++)
        {
            var target = targets[i];
            Vector3 targetPosition = target.GetComponent<Collider>().bounds.center;

            objects[i] = new ObjectStateData
            {
                name = target.name,
                position = new float[] { targetPosition.x, targetPosition.y, targetPosition.z },
                is_attached =
                    target.transform.parent
                    != null // && block.transform.parent.name.Contains("suction")
                ,
            };
        }

        return objects;
    }

    private string ExecuteActionJson(SimulAICommand command)
    {
        try
        {
            bool leftSuccess = true;
            bool rightSuccess = true;
            toMoveArms = false;
            toMoveLeftArm = false;
            toMoveRightArm = false;
            moveArmsNow = false;
            string message = "";

            GameObject ur5_left_robot;
            GameObject ur5_right_robot;
            if (robots[0].name == "ur5_left")
            {
                ur5_left_robot = robots[0];
                ur5_right_robot = robots[1];
            }
            else
            {
                ur5_left_robot = robots[1];
                ur5_right_robot = robots[0];
            }

            // execute action for ur5_left
            switch (command.ur5_left.action_type)
            {
                case "home_robot":
                    robotArmSetups[System.Array.IndexOf(robots, ur5_left_robot)].ResetArmPosition();
                    leftSuccess = true;
                    message = "Left Robot homed successfully";
                    break;
                case "solve_ik": // delay movement cos need to move both arms tgt
                    toMoveLeftArm = true;
                    break;
                case "move_robot": // delay movement cos need to move both arms tgt
                    toMoveLeftArm = true;
                    break;
                case "stationary":
                    break;

                case "activate_suction":
                    leftSuccess = ExecuteActivateSuction(ur5_left_robot);
                    message = leftSuccess
                        ? "Left Suction activated"
                        : "Left Suction activation failed";
                    break;

                case "deactivate_suction":
                    leftSuccess = ExecuteDeactivateSuction(ur5_left_robot);
                    message = leftSuccess
                        ? "Left Suction deactivated"
                        : "Left Suction deactivation failed";
                    break;

                default:
                    message = "Unknown action type: " + command.ur5_left.action_type;
                    break;
            }

            message += " | ";

            // execute action for ur5 right
            switch (command.ur5_right.action_type)
            {
                case "home_robot":
                    robotArmSetups[System.Array.IndexOf(robots, ur5_right_robot)].ResetArmPosition();
                    rightSuccess = true;
                    message += "Right Robot homed successfully";
                    break;
                case "solve_ik": // delay movement cos need to move both arms tgt
                    toMoveRightArm = true;
                    break;
                case "move_robot": // delay movement cos need to move both arms tgt
                    toMoveRightArm = true;
                    break;
                case "stationary":
                    break;

                case "activate_suction":
                    rightSuccess = ExecuteActivateSuction(ur5_right_robot);
                    message += rightSuccess
                        ? "Right Suction activated"
                        : "Right Suction activation failed";
                    break;

                case "deactivate_suction":
                    rightSuccess = ExecuteDeactivateSuction(ur5_right_robot);
                    message += rightSuccess
                        ? "Right Suction deactivated"
                        : "Right Suction deactivation failed";
                    break;

                default:
                    message += "Unknown action type: " + command.ur5_right.action_type;
                    break;
            }

            // coordinate and execute movements tgt
            bool success = leftSuccess && rightSuccess;
            toMoveArms = toMoveLeftArm || toMoveRightArm;

            if (toMoveArms)
            {
                Debug.Log("Time to move arms");
                GameObject[] ikRobots;
                ActionParameters[] parameterArray;
                if (toMoveLeftArm && toMoveRightArm)
                {
                    ikRobots = new GameObject[robots.Length];
                    ikRobots[0] = ur5_left_robot;
                    ikRobots[1] = ur5_right_robot;
                    parameterArray = new ActionParameters[robots.Length];
                    parameterArray[0] = command.ur5_left.parameters;
                    parameterArray[1] = command.ur5_right.parameters;
                }
                else if (toMoveLeftArm)
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_left_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_left.parameters;
                }
                else
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_right_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_right.parameters;
                }
                return ExecuteMoveRobot(ikRobots, parameterArray);
            }

            toMoveArms = false;
            toMoveLeftArm = false;
            toMoveRightArm = false;
            moveArmsNow = false;
            return JsonUtility.ToJson(new AIResponse { success = success, message = message });
        }
        catch (Exception e)
        {
            Debug.LogError("Error executing action: " + e.Message);
            return JsonUtility.ToJson(
                new AIResponse { success = false, message = "Error executing action: " + e.Message }
            );
        }
    }

    private bool ExecuteSolveIK(GameObject[] robots, ActionParameters[] parameters)
    {
        try
        {
            for (int i = 0; i < robots.Length; i++)
            {
                int idx = System.Array.IndexOf(this.robots, robots[i]);
                ur5Controllers[idx].SetJointAngles(parameters[i].joint_angles);
            }
            moveArmsNow = true;
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError("Error in ExecuteSolveIK: " + e.Message);
            return false;
        }
    }

    private string ExecuteMoveRobot(GameObject[] ikRobots, ActionParameters[] parameters)
    {
        bool allSuccess = true;
        try
        {
            for (int i = 0; i < ikRobots.Length; i++)
            {
                if (parameters[i] == null || parameters[i].target_position == null)
                {
                    Debug.LogError($"ExecuteMoveRobot: null parameters or target_position for robot {ikRobots[i]?.name}");
                    return JsonUtility.ToJson(new AIResponse { success = false, message = "null target_position" });
                }
                int idx = System.Array.IndexOf(robots, ikRobots[i]);
                if (idx < 0 || ur5Controllers[idx] == null)
                {
                    Debug.LogError($"ExecuteMoveRobot: UR5Controller not found for robot {ikRobots[i]?.name} (idx={idx})");
                    return JsonUtility.ToJson(new AIResponse { success = false, message = "UR5Controller not found" });
                }
                Vector3 targetPos = new Vector3(
                    parameters[i].target_position[0],
                    parameters[i].target_position[1] + SuctionIKOffsetY,
                    parameters[i].target_position[2]);
                if (!ur5Controllers[idx].MoveToWorldPosition(targetPos))
                    allSuccess = false;
            }
        }
        catch (Exception e)
        {
            Debug.LogError("ExecuteMoveRobot error: " + e.Message);
            return JsonUtility.ToJson(new AIResponse { success = false, message = e.Message });
        }
        return JsonUtility.ToJson(new AIResponse {
            success = allSuccess,
            message = allSuccess ? "move_executed" : "ik_failed"
        });
    }

    private bool ExecuteActivateSuction(GameObject robot)
    {
        var suctionController = robot.GetComponent<SuctionController>();
        if (suctionController == null)
            return false;

        suctionController.ToggleSuction();
        return true;
    }

    private bool ExecuteDeactivateSuction(GameObject robot)
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

    private const float IdleVelocityThreshold = 0.05f;

    private bool IsAllRobotsIdle()
    {
        bool isIdle = true;
        foreach (GameObject robot in robots)
        {
            RobotArmSetup setup = robot.GetComponent<RobotArmSetup>();
            if (setup == null) continue;
            foreach (ArticulationBody joint in setup.robotJoints)
            {
                if (joint == null || joint.dofCount == 0) continue;
                if (Mathf.Abs(joint.jointVelocity[0]) > IdleVelocityThreshold)
                {
                    int jointIndex = System.Array.IndexOf(setup.robotJoints, joint);
                    print($"{robot.name} joint[{jointIndex}] vel={joint.jointVelocity[0]:F4}");
                    isIdle = false;
                }
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

    void OnApplicationQuit()
    {
        closeSocket();
    }
}
