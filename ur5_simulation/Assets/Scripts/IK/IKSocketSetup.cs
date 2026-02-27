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
public class IKSocketSetup : MonoBehaviour
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
    private SceneSetup sceneSetup;
    private UR5IKController[] ur5IKControllers;
    private GameObject[] robots;
    private GameObject[] targets;
    private RobotArmSetup[] robotArmSetups;
    private int robotCount;
    private int targetCount;
    private int robotIdx;

    private Queue<Action> mainThreadActions = new Queue<Action>();
    private object lockObject = new object();
    //private string pendingResponse;
    //private bool isProcessingCommand;

    private bool isSocketActive = false;

    void Start()
    {

        sceneSetup = GetComponent<SceneSetup>();

        //GET ALL REQUIRED SCENE DATA
        robots = sceneSetup.robots;
        robotCount = robots.Length;

        robotArmSetups = new RobotArmSetup[robotCount];
        ur5IKControllers = new UR5IKController[robotCount];

        for (int i = 0; i < robotCount; i++)
        {
            RobotArmSetup robotArmSetup = robots[i].GetComponent<RobotArmSetup>();
            robotArmSetups[i] = robotArmSetup;

            UR5IKController ur5IKController = robots[i].GetComponent<UR5IKController>();
            ur5IKControllers[i] = ur5IKController;
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
                // Debug.Log("Received from Python: " + receivedData);

                // Queue the processing to main thread
                lock (lockObject)
                {
                    mainThreadActions.Enqueue(() =>
                    {
                        string response = ProcessIKCommand(receivedData);
                        byte[] responseData = Encoding.UTF8.GetBytes(response);
                        stream.Write(responseData, 0, responseData.Length);
                        // Debug.Log("Sent to Python: " + response);
                    });
                }

                // Continue listening for more data
                stream.BeginRead(receiveBuffer, 0, receiveBuffer.Length, ReceiveCallback, null);
            }
        }
        catch (Exception e)
        {
            if (isSocketActive) Debug.LogError("Receive callback error: " + e.Message);
        }
    }

    private string ProcessIKCommand(string commandJson)
    {
        // Implement IK command processing here
        try
        {
            var command = JsonUtility.FromJson<IKCommand>(commandJson);
            switch (command.type)
            {
                case "move_to_target":
                    return ExecuteMoveToTargetJson(command);
                case "waiting":
                    return IKWaitJson();
                default:
                    return JsonUtility.ToJson(new IKResponse 
                    { 
                        success = false, 
                        message = "Unknown command type: " + command.type 
                    });
            }
        } catch (Exception e)
        {
            Debug.LogError("Error processing IK command: " + e.Message);
            return JsonUtility.ToJson(new IKResponse 
            { 
                success = false, 
                message = "Error processing command: " + e.Message 
            });
        }
        
    }

    private string ExecuteMoveToTargetJson(IKCommand command)
    {
        try
        {                    
            // Set IK solution ready flag
            // ur5IKControllers[robotIdx].hasIKSolution = true; // IK solution is ready
            // ur5IKControllers[robotIdx].ikSolution = command.joint_angles.Select(a => (double)a).ToArray();

            // calculate same effective joint angles with least movement
            float[] currentJointAngles = ur5IKControllers[robotIdx].GetJointAngles();
            for (int i = 0; i < JointCount; i++)
            {
                command.joint_angles[i] = GetBestIKAngle(currentJointAngles[i], command.joint_angles[i]);
                Debug.Log($"Joint {i}: Setting target to {command.joint_angles[i] * Mathf.Rad2Deg} degrees ({command.joint_angles[i]} radians)");
            }
            
            // directly set joint angles here
            ur5IKControllers[robotIdx].SetJointAngles(command.joint_angles);
            Debug.Log("Set joint angles in IKSOCKET");
            
            return JsonUtility.ToJson(new IKResponse 
            { 
                success = true, 
                message = "Move to target command executed"
            });
        }
        catch (Exception e)
        {
            Debug.LogError("Error executing move to target: " + e.Message);
            return JsonUtility.ToJson(new IKResponse 
            { 
                success = false, 
                message = "Error executing move to target: " + e.Message 
            });
        }
    }

    private string IKWaitJson() // ik python client waiting for Unity to send IK stuff
    {
        // check if any of the robots are ready for IK commands
        for (int i = 0; i < robotCount; i++)
        {
            var ur5IKController = ur5IKControllers[i];
            if (ur5IKController.ikActive)
            {
                ur5IKController.ikActive = false; // reset flag
                robotIdx = i;
                var robot_state_data = new IKRobotStateData
                {
                    robot_name = robots[i].name,
                    current_joint_angles = ur5IKController.GetJointAngles(),
                    end_effector_position = new float[]
                    {
                        ur5IKController.endEffectorTargetPosition.x,
                        ur5IKController.endEffectorTargetPosition.y,
                        ur5IKController.endEffectorTargetPosition.z
                    }
                };
                return JsonUtility.ToJson(new IKResponse 
                { 
                    success = true, 
                    message = "Ik active",
                    robot_state = robot_state_data
                });
            }
        }

        return JsonUtility.ToJson(new IKResponse 
        { 
            success = true, 
            message = "Waiting for user IK input"
        });
    }

    private float GetBestIKAngle(float currentAngle, float targetAngle)
    {
        float twoPi = 2 * Mathf.PI;
        float currentAngleRadian = currentAngle * Mathf.Deg2Rad;
        float angleDifference = targetAngle - currentAngleRadian;

        // Normalize the angle difference to the range [-π, π]
        while (angleDifference > Mathf.PI) angleDifference -= twoPi;
        while (angleDifference < -Mathf.PI) angleDifference += twoPi;

        // Calculate the best target angle
        float bestTargetAngle = currentAngleRadian + angleDifference;
        return bestTargetAngle;
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
