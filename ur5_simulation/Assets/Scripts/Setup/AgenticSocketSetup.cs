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
public class AgenticSocketSetup : MonoBehaviour
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

    private PoseAndImageRecorder recorder;

    // AI Agent Integration
    private AgentRobotController agentController;
    private IKResponse ik_response;
    private SceneSetup sceneSetup;
    private GameObject[] robots;
    private GameObject[] targets;
    private RobotArmSetup[] robotArmSetups;
    private SuctionController[] suctionControllers;
    private int robotCount;
    private int targetCount;

    private Queue<Action> mainThreadActions = new Queue<Action>();
    private object lockObject = new object();

    //private string pendingResponse;
    //private bool isProcessingCommand;

    private bool isSocketActive = false;

    void Start()
    {
        // Initialize AI components
        sceneSetup = GetComponent<SceneSetup>();

        if (sceneSetup != null)
            agentController = sceneSetup.agentRobotController;

        recorder = GetComponent<PoseAndImageRecorder>();
        if (recorder == null)
            recorder = gameObject.AddComponent<PoseAndImageRecorder>();

        if (agentController == null)
        {
            Debug.LogWarning(
                "PythonSocketSetup: AgentRobotController not found. AI features may be limited."
            );
        }

        //GET ALL REQUIRED SCENE DATA
        robots = sceneSetup.robots;
        targets = sceneSetup.targets;
        robotCount = robots.Length;
        targetCount = targets.Length;

        robotArmSetups = new RobotArmSetup[robotCount];
        suctionControllers = new SuctionController[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            RobotArmSetup robotArmSetup = robots[i].GetComponent<RobotArmSetup>();
            robotArmSetups[i] = robotArmSetup;

            SuctionController suctionController = robots[i].GetComponent<SuctionController>();
            suctionControllers[i] = suctionController;
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
            var command = JsonUtility.FromJson<AICommand>(commandJson);

            switch (command.type)
            {
                case "get_scene_state":
                    return GetSceneStateJson();

                case "execute_action":
                    return ExecuteActionJson(command);

                case "reset_scene":
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
                    return JsonUtility.ToJson(
                        new AIResponse
                        {
                            success = false,
                            message = "Unknown command type: " + command.type,
                        }
                    );
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

        // foreach (var robot in robots)
        // {
        //     var robotArmSetup = robot.GetComponent<RobotArmSetup>();
        //     var suctionController = robot.GetComponent<SuctionController>();
        //     //var unifiedController = robot.GetComponent<UnifiedRobotController>();

        //     if (robotArmSetup != null)
        //     {
        //         var endEffector = robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform;

        //         var robotState = new RobotStateData
        //         {
        //             name = robot.name,
        //             joint_angles = GetJointAngles(robotArmSetup.robotJoints),
        //             end_effector_position = new float[]
        //             {
        //                 endEffector.position.x,
        //                 endEffector.position.y,
        //                 endEffector.position.z
        //             },
        //             suction_active = suctionController != null && suctionController.enableSuction,
        //             //is_moving = unifiedController != null && unifiedController.currentMode != UnifiedRobotController.ControlMode.Start
        //         };

        //         robotStates.Add(robotState);
        //     }
        // }

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

        // foreach (var block in targets)
        // {
        //     objects.Add(new ObjectStateData
        //     {
        //         name = block.name,
        //         position = new float[] { block.transform.position.x, block.transform.position.y, block.transform.position.z },
        //         rotation = new float[] { block.transform.rotation.x, block.transform.rotation.y, block.transform.rotation.z, block.transform.rotation.w },
        //         is_attached = block.transform.parent != null // && block.transform.parent.name.Contains("suction")
        //     });
        // }

        // return objects.ToArray();
    }

    private string ExecuteActionJson(AICommand command)
    {
        try
        {
            bool success = false;
            string message = "";

            // Find the target robot
            GameObject targetRobot = null;
            if (sceneSetup != null && sceneSetup.robots != null)
            {
                targetRobot = sceneSetup.robots.FirstOrDefault(r => r.name == command.robot_name);
            }

            if (targetRobot == null)
            {
                return JsonUtility.ToJson(
                    new AIResponse
                    {
                        success = false,
                        message = "Robot not found: " + command.robot_name,
                    }
                );
            }

            // Execute the action based on type
            switch (command.action_type)
            {
                case "home_robot":
                    success = agentController.ResetToHomePosition(targetRobot);
                    message = success ? "Robot homed successfully" : "Robot homing failed";
                    break;
                case "solve_ik":
                    success = ExecuteSolveIK(targetRobot, command.parameters);
                    message = success ? "IK command executed" : "IK command failed";
                    break;
                case "move_robot":
                    ExecuteMoveRobot(targetRobot, command.parameters);
                    return JsonUtility.ToJson(ik_response);

                case "activate_suction":
                    success = ExecuteActivateSuction(targetRobot);
                    message = success ? "Suction activated" : "Suction activation failed";
                    break;

                case "deactivate_suction":
                    success = ExecuteDeactivateSuction(targetRobot);
                    message = success ? "Suction deactivated" : "Suction deactivation failed";
                    break;

                default:
                    message = "Unknown action type: " + command.action_type;
                    break;
            }

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

    private bool ExecuteSolveIK(GameObject robot, ActionParameters parameters)
    {
        // set joint angles for specified robot
        return agentController.SetJointAngles(robot, parameters.joint_angles);
    }

    private void ExecuteMoveRobot(GameObject robot, ActionParameters parameters)
    {
        try
        {
            Vector3 targetInputPosition = new Vector3(
                parameters.target_position[0],
                parameters.target_position[1],
                parameters.target_position[2]
            );

            targetInputPosition.y += SuctionIKOffsetY; // slight offset to account for suction cup height
            Vector3 endEffectorTargetPosition = robot.transform.InverseTransformPoint(
                targetInputPosition
            );

            var robot_state_data = new IKRobotStateData
            {
                robot_name = robot.name,
                current_joint_angles = agentController.GetJointAngles(robot),
                end_effector_position = new float[]
                {
                    endEffectorTargetPosition.x,
                    endEffectorTargetPosition.y,
                    endEffectorTargetPosition.z,
                },
            };

            ik_response = new IKResponse
            {
                success = true,
                message = "run_ik",
                robot_state = robot_state_data,
            };
        }
        catch (Exception e)
        {
            Debug.LogError("Error in ExecuteMoveRobot: " + e.Message);
            ik_response = new IKResponse
            {
                success = false,
                message = "Error in ExecuteMoveRobot: " + e.Message,
                robot_state = null,
            };
        }
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
            if (setup == null)
                continue;
            foreach (ArticulationBody joint in setup.robotJoints)
            {
                if (joint == null || joint.dofCount == 0)
                    continue;

                int jointIndex = System.Array.IndexOf(setup.robotJoints, joint);
                print($"{robot.name} joint[{jointIndex}] vel={joint.jointVelocity[0]:F4}");

                if (Mathf.Abs(joint.jointVelocity[0]) > IdleVelocityThreshold)
                {
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
