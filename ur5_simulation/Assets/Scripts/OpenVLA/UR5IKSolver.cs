using UnityEngine;
using System;
using System.Net.Sockets;
using System.IO;

/// <summary>
/// IK Solver for UR5 robot using Python roboticstoolbox via TCP connection
/// Connects to a Python server that performs IK calculations using rtb library
/// </summary>
public class UR5IKSolver : MonoBehaviour
{
    [Header("Python IK Server Settings")]
    [Tooltip("Python IK server host address")]
    public string serverHost = "127.0.0.1";

    [Tooltip("Python IK server port")]
    public int serverPort = 5010;

    [Tooltip("Enable auto-reconnect on connection failure")]
    public bool autoReconnect = true;

    [Tooltip("Connection timeout in milliseconds")]
    public int connectionTimeout = 5000;

    [Header("Debug")]
    [Tooltip("Enable detailed logging")]
    public bool debugMode = false;

    // TCP client for communication with Python server
    private TcpClient client;
    private NetworkStream stream;
    private BinaryWriter writer;
    private BinaryReader reader;
    private bool isConnected = false;

    // Command types
    private const byte CMD_SOLVE_IK = 1;
    private const byte CMD_SOLVE_IK_DELTA = 2;

    void Start()
    {
        ConnectToServer();
    }

    void OnDestroy()
    {
        DisconnectFromServer();
    }

    /// <summary>
    /// Connect to the Python IK server
    /// </summary>
    private bool ConnectToServer()
    {
        try
        {
            if (isConnected)
            {
                Debug.LogWarning("UR5IKSolver: Already connected to server");
                return true;
            }

            Debug.Log($"UR5IKSolver: Connecting to Python IK server at {serverHost}:{serverPort}...");

            client = new TcpClient();
            client.ReceiveTimeout = connectionTimeout;
            client.SendTimeout = connectionTimeout;

            IAsyncResult result = client.BeginConnect(serverHost, serverPort, null, null);
            bool success = result.AsyncWaitHandle.WaitOne(TimeSpan.FromMilliseconds(connectionTimeout));

            if (!success)
            {
                Debug.LogError("UR5IKSolver: Connection timeout");
                client.Close();
                return false;
            }

            client.EndConnect(result);

            stream = client.GetStream();
            writer = new BinaryWriter(stream);
            reader = new BinaryReader(stream);

            isConnected = true;
            Debug.Log("UR5IKSolver: Connected to Python IK server successfully");
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"UR5IKSolver: Failed to connect to server: {e.Message}");
            isConnected = false;
            return false;
        }
    }

    /// <summary>
    /// Disconnect from the Python IK server
    /// </summary>
    private void DisconnectFromServer()
    {
        try
        {
            if (writer != null) writer.Close();
            if (reader != null) reader.Close();
            if (stream != null) stream.Close();
            if (client != null) client.Close();

            isConnected = false;
            Debug.Log("UR5IKSolver: Disconnected from Python IK server");
        }
        catch (Exception e)
        {
            Debug.LogError($"UR5IKSolver: Error during disconnect: {e.Message}");
        }
    }

    /// <summary>
    /// Ensure connection to server, reconnect if necessary
    /// </summary>
    private bool EnsureConnection()
    {
        if (isConnected && client != null && client.Connected)
        {
            return true;
        }

        if (autoReconnect)
        {
            Debug.LogWarning("UR5IKSolver: Connection lost, attempting to reconnect...");
            DisconnectFromServer();
            return ConnectToServer();
        }

        return false;
    }

    /// <summary>
    /// Solve inverse kinematics for target position and rotation
    /// </summary>
    /// <param name="targetPosition">Target end-effector position in Unity coordinates</param>
    /// <param name="targetRotation">Target end-effector rotation in Unity coordinates</param>
    /// <param name="currentAngles">Current joint angles in radians</param>
    /// <returns>Target joint angles in radians, or null if no solution found</returns>
    public float[] SolveIK(Vector3 targetPosition, Quaternion targetRotation, float[] currentAngles)
    {
        print("solving ik for target position");
        print($"Target: ({targetPosition.x:F6}, {targetPosition.y:F6}, {targetPosition.z:F6})");
        if (currentAngles == null || currentAngles.Length != 6)
        {
            Debug.LogError("UR5IKSolver.SolveIK: currentAngles must be an array of 6 floats");
            return null;
        }

        if (!EnsureConnection())
        {
            Debug.LogError("UR5IKSolver.SolveIK: Not connected to Python server");
            return currentAngles; // Return current angles as fallback
        }

        try
        {
            if (debugMode)
            {
                Debug.Log($"UR5IKSolver.SolveIK: Sending request - pos={targetPosition}, rot={targetRotation}");
            }

            // Send command type
            writer.Write(CMD_SOLVE_IK);

            // Send target position (3 doubles)
            writer.Write((double)targetPosition.x);
            writer.Write((double)targetPosition.y);
            writer.Write((double)targetPosition.z);

            // Send target rotation (4 doubles - quaternion xyzw)
            writer.Write((double)targetRotation.x);
            writer.Write((double)targetRotation.y);
            writer.Write((double)targetRotation.z);
            writer.Write((double)targetRotation.w);

            // Send current angles (6 doubles)
            for (int i = 0; i < 6; i++)
            {
                writer.Write((double)currentAngles[i]);
            }

            writer.Flush();

            // Wait for data to be available with timeout
            int timeout = connectionTimeout;
            int elapsed = 0;
            int sleepInterval = 10; // ms

            while (!stream.DataAvailable && elapsed < timeout)
            {
                System.Threading.Thread.Sleep(sleepInterval);
                elapsed += sleepInterval;
            }

            if (!stream.DataAvailable)
            {
                Debug.LogError("UR5IKSolver.SolveIK: Timeout waiting for server response");
                isConnected = false;
                return null;
            }

            // Read response: 1 byte success flag
            byte success = reader.ReadByte();

            if (success == 1)
            {
                // Wait for full response data
                elapsed = 0;
                while (stream.DataAvailable && stream.Length < 48 && elapsed < timeout)
                {
                    System.Threading.Thread.Sleep(sleepInterval);
                    elapsed += sleepInterval;
                }

                // Read 6 joint angles (6 doubles = 48 bytes)
                float[] solution = new float[6];
                for (int i = 0; i < 6; i++)
                {
                    solution[i] = (float)reader.ReadDouble();
                }

                if (debugMode)
                {
                    Debug.Log($"UR5IKSolver.SolveIK: Solution found - joints={string.Join(", ", solution)}");
                }

                return solution;
            }
            else
            {
                Debug.LogWarning("UR5IKSolver.SolveIK: Python server could not find IK solution");
                return null;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"UR5IKSolver.SolveIK: Communication error: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");
            isConnected = false;
            return null;
        }
    }
}
