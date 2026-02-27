using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Net;
using System.IO;
using System.Collections.Generic;
using System.Linq;

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
    
    // AI Agent Integration
    private AgentRobotController agentController;
    private SimulIKResponse ik_response;
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

    private bool toSendIKCommand = false;
    private bool toSendIKLeft = false;
    private bool toSendIKRight = false;
    private bool toMoveArms = false;
    private bool toMoveLeftArm = false;
    private bool toMoveRightArm = false;

    [HideInInspector]
    public bool moveArmsNow = false;

    void Start()
    {
        // Initialize AI components
        sceneSetup = GetComponent<SceneSetup>();

        if (sceneSetup != null) agentController = sceneSetup.agentRobotController;

        if (agentController == null)
        {
            Debug.LogWarning("PythonSocketSetup: AgentRobotController not found. AI features may be limited.");
        }

        //GET ALL REQUIRED SCENE DATA
        robots = sceneSetup.robots;
        targets = sceneSetup.targets;
        robotCount = robots.Length;
        targetCount = targets.Length;

        robotArmSetups = new RobotArmSetup[robotCount];
        suctionControllers = new SuctionController[robotCount];
        // simulAgentControllers = new SimulAgentController[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            RobotArmSetup robotArmSetup = robots[i].GetComponent<RobotArmSetup>();
            robotArmSetups[i] = robotArmSetup;

            SuctionController suctionController = robots[i].GetComponent<SuctionController>();
            suctionControllers[i] = suctionController;

            // SimulAgentController simulAgentController = robots[i].GetComponent<SimulAgentController>();
            // simulAgentControllers[i] = simulAgentController;
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
            if (isSocketActive) Debug.LogError("Receive callback error: " + e.Message);
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
                    
                default: // simultaenous AI command
                    return ExecuteActionJson(command);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error processing AI command: " + e.Message);
            return JsonUtility.ToJson(new AIResponse 
            { 
                success = false, 
                message = "Error processing command: " + e.Message 
            });
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
                timestamp = Time.time
            };
            
            return JsonUtility.ToJson(sceneState);
        }
        catch (Exception e)
        {
            Debug.LogError("Error getting scene state: " + e.Message);
            return JsonUtility.ToJson(new AIResponse 
            { 
                success = false, 
                message = "Error getting scene state: " + e.Message 
            });
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
                    position = new float[] { 
                        targetPosition.x,
                        targetPosition.y,
                        targetPosition.z
                    },
                    is_attached = target.transform.parent != null // && block.transform.parent.name.Contains("suction")
                };
            }

            var endEffector = robotArmSetup.robotJoints[robotArmSetup.robotJoints.Length - 1].transform;
            robotStates[i] = new RobotStateData
            {
                name = robot.name,
                // joint_angles = GetJointAngles(robotArmSetup.robotJoints),
                end_effector_position = new float[]
                {
                    endEffector.position.x,
                    endEffector.position.y,
                    endEffector.position.z
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
                is_attached = target.transform.parent != null // && block.transform.parent.name.Contains("suction")
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
            toSendIKCommand = false;
            toSendIKLeft = false;
            toSendIKRight = false;
            toMoveArms = false;
            moveArmsNow = false;
            string message = "";

            GameObject ur5_left_robot;
            GameObject ur5_right_robot;
            if (robots[0].name == "ur5_left")
            {
                ur5_left_robot = robots[0];
                ur5_right_robot = robots[1];
            } else
            {
                ur5_left_robot = robots[1];
                ur5_right_robot = robots[0];
            }

            // execute action for ur5_left
            switch (command.ur5_left.action_type)
            {
                case "home_robot":
                    // toMoveLeftArm = true;
                    leftSuccess = agentController.ResetToHomePosition(ur5_left_robot);
                    message = leftSuccess ? "Left Robot homed successfully" : "Left Robot homing failed";
                    break;
                case "solve_ik": // delay movement cos need to move both arms tgt
                    toMoveLeftArm = true;
                    break;
                case "move_robot": // delay movement cos need to move both arms tgt
                    toSendIKLeft = true;
                    break;
                case "stationary":
                    break;
                    
                case "activate_suction":
                    leftSuccess = ExecuteActivateSuction(ur5_left_robot);
                    message = leftSuccess ? "Left Suction activated" : "Left Suction activation failed";
                    break;
                    
                case "deactivate_suction":
                    leftSuccess = ExecuteDeactivateSuction(ur5_left_robot);
                    message = leftSuccess ? "Left Suction deactivated" : "Left Suction deactivation failed";
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
                    // toMoveRightArm = true;
                    rightSuccess = agentController.ResetToHomePosition(ur5_right_robot);
                    message += rightSuccess ? "Right Robot homed successfully" : "Right Robot homing failed";
                    break;
                case "solve_ik": // delay movement cos need to move both arms tgt
                    toMoveRightArm = true;
                    break;
                case "move_robot": // delay movement cos need to move both arms tgt
                    toSendIKRight = true;
                    break;
                case "stationary":
                    break;
                    
                case "activate_suction":
                    rightSuccess = ExecuteActivateSuction(ur5_right_robot);
                    message += rightSuccess ? "Right Suction activated" : "Right Suction activation failed";
                    break;
                    
                case "deactivate_suction":
                    rightSuccess = ExecuteDeactivateSuction(ur5_right_robot);
                    message += rightSuccess ? "Right Suction deactivated" : "Right Suction deactivation failed";
                    break;
                    
                default:
                    message += "Unknown action type: " + command.ur5_right.action_type;
                    break;
            }

            // coordinate and execute movements tgt
            bool success = leftSuccess && rightSuccess;
            toSendIKCommand = toSendIKLeft || toSendIKRight;

            if (toSendIKCommand)
            {
                GameObject[] ikRobots;
                ActionParameters[] parameterArray;
                if (toSendIKLeft && toSendIKRight)
                {
                    ikRobots = new GameObject[robots.Length];
                    ikRobots[0] = ur5_left_robot;
                    ikRobots[1] = ur5_right_robot;
                    parameterArray = new ActionParameters[robots.Length];
                    parameterArray[0] = command.ur5_left.parameters;
                    parameterArray[1] = command.ur5_right.parameters;
                } else if (toSendIKLeft)
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_left_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_left.parameters;
                } else
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_right_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_right.parameters;
                }
                ExecuteMoveRobot(ikRobots, parameterArray);
                return JsonUtility.ToJson(ik_response);
            }

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
                } else if (toMoveLeftArm)
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_left_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_left.parameters;
                } else
                {
                    ikRobots = new GameObject[1];
                    ikRobots[0] = ur5_right_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_right.parameters;
                }
                bool solveIKSuccess = ExecuteSolveIK(ikRobots, parameterArray);
                message = solveIKSuccess ? "IK command executed" : "IK command failed";
            }

            toMoveArms = false;
            toMoveLeftArm = false;
            toMoveRightArm = false;
            toSendIKLeft = false;
            toSendIKRight = false;
            toSendIKCommand = false;
            moveArmsNow = false;
            return JsonUtility.ToJson(new AIResponse { success = success, message = message });
        }
        catch (Exception e)
        {
            Debug.LogError("Error executing action: " + e.Message);
            return JsonUtility.ToJson(new AIResponse 
            { 
                success = false, 
                message = "Error executing action: " + e.Message 
            });
        }
    }

    private bool ExecuteSolveIK(GameObject[] robots, ActionParameters[] parameters)
    {
        try
        {
            for (int i = 0; i < robots.Length; i++)
            {
                GameObject robot = robots[i];
                // SimulAgentController simulAgentController = simulAgentControllers.FirstOrDefault(r => r.robot_name == robot.name);
                // simulAgentController.ikSolution = parameters[i].joint_angles;
                agentController.SetJointAngles(robot, parameters[i].joint_angles);
            }
            moveArmsNow = true;
            return true;
        } catch (Exception e)
        {
            Debug.LogError("Error in ExecuteSolveIK: " + e.Message);
            return false;
        }
    }
    
    private void ExecuteMoveRobot(GameObject[] robots, ActionParameters[] parameters)
    {
        IKRobotStateData[] robot_states = new IKRobotStateData[robots.Length];
        try
        {
            for (int i = 0; i < robots.Length; i++)
            {
                GameObject robot = robots[i];
                Vector3 targetInputPosition = new Vector3(
                    parameters[i].target_position[0],
                    parameters[i].target_position[1],
                    parameters[i].target_position[2]
                );

                targetInputPosition.y += SuctionIKOffsetY; // slight offset to account for suction cup height
                Vector3 endEffectorTargetPosition = robot.transform.InverseTransformPoint(targetInputPosition);

                robot_states[i] = new IKRobotStateData
                {
                    robot_name = robot.name,
                    current_joint_angles = agentController.GetJointAngles(robot),
                    end_effector_position = new float[]
                    {
                        endEffectorTargetPosition.x,
                        endEffectorTargetPosition.y,
                        endEffectorTargetPosition.z
                    }
                };
            }
            
            ik_response = new SimulIKResponse
            {
                success = true,
                robot_states = robot_states
            };
        } 
        catch (Exception e)
        {
            Debug.LogError("Error in ExecuteMoveRobot: " + e.Message);
            ik_response = new SimulIKResponse
            {
                success = false,
                message = "Error in ExecuteMoveRobot: " + e.Message,
                robot_states = null
            };
        }
        
    }
    
    private bool ExecuteActivateSuction(GameObject robot)
    {
        var suctionController = robot.GetComponent<SuctionController>();
        if (suctionController == null) return false;
        
        suctionController.ToggleSuction();
        return true;
    }

    private bool ExecuteDeactivateSuction(GameObject robot)
    {
        var suctionController = robot.GetComponent<SuctionController>();
        if (suctionController == null) return false;

        if (suctionController.enableSuction)
        {
            suctionController.ToggleSuction();
        }
        return true;
    }
    
    public void closeSocket()
    {
        isSocketActive = false;
        if (stream != null) stream.Close();
        if (client != null) client.Close();
        if (listener != null) listener.Stop();
    }

    void OnApplicationQuit()
    {
        closeSocket();
    }
}
