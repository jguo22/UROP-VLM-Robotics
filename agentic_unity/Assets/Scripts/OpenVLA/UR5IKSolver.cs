using UnityEngine;
using System;
using System.Net.Sockets;
using System.IO;
using System.Threading.Tasks;

/// <summary>
/// IK Solver for UR5 robot using Python roboticstoolbox via TCP connection.
/// Connects to a Python server that performs IK calculations using rtb library.
///
/// Protocol:
///   Client sends: 104 bytes [target_pos(3d) + target_rot(4d) + current_angles(6d)]
///   Server responds: 1 byte success flag + 48 bytes joint angles (if success)
/// </summary>
public class UR5IKSolver : MonoBehaviour
{
    [Header("Python IK Server Settings")]
    [SerializeField]
    [Tooltip("Python IK server host address")]
    private string serverHost = "127.0.0.1";

    [SerializeField]
    [Tooltip("Python IK server port")]
    private int serverPort = 5010;

    [SerializeField]
    [Tooltip("Enable auto-reconnect on connection failure")]
    private bool autoReconnect = true;

    [SerializeField]
    [Tooltip("Connection timeout in milliseconds")]
    private int connectionTimeout = 5000; // Initial connection timeout (5 seconds)

    // [Header("Debug")]
    // [SerializeField]
    // [Tooltip("Enable detailed logging")]
    private bool debugMode = true;

    // TCP client for communication with Python server
    private TcpClient client;
    private NetworkStream stream;
    private BinaryWriter writer;
    private BinaryReader reader;
    private bool isConnected = false;
    private bool isConnecting = false;
    private Task connectionTask = null;

    void Start()
    {
        EnsureConnection();
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

            // Enable TCP keepalive to prevent idle disconnects
            client.Client.SetSocketOption(System.Net.Sockets.SocketOptionLevel.Socket,
                                         System.Net.Sockets.SocketOptionName.KeepAlive, true);

            // Set 10-minute timeout to keep connection alive
            client.ReceiveTimeout = 600000; // 10 minutes
            client.SendTimeout = 600000; // 10 minutes

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
            Debug.Log($"UR5IKSolver: Connected to Python IK server at {serverHost}:{serverPort} successfully!");
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
    /// Ensure connection to server, start connection if needed
    /// Returns true if connected, false if connecting or failed
    /// </summary>
    private bool EnsureConnection()
    {
        // Already connected
        if (isConnected && client != null && client.Connected)
        {
            return true;
        }

        // Currently connecting - don't block, just return false
        if (isConnecting)
        {
            return false;
        }

        // Not connected and not connecting - start async connection
        if (autoReconnect || connectionTask == null)
        {
            isConnecting = true;
            connectionTask = Task.Run(() =>
            {
                try
                {
                    bool success = ConnectToServer();
                    isConnecting = false;
                    return success;
                }
                catch (Exception e)
                {
                    Debug.LogError($"UR5IKSolver: Async connection failed: {e.Message}");
                    isConnecting = false;
                    return false;
                }
            });
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
        if (currentAngles == null || currentAngles.Length != 6)
            throw new Exception("UR5IKSolver.SolveIK: currentAngles must be an array of 6 floats");

        if (!EnsureConnection())
        {
            // Not connected yet - return current angles and connection will retry
            if (debugMode && !isConnecting)
            {
                Debug.LogWarning("UR5IKSolver.SolveIK: Not connected to Python server");
            }
            return currentAngles;
        }

        try
        {
            if (debugMode)
            {
                Debug.Log($"UR5IKSolver.SolveIK: Sending request - pos={targetPosition}, rot={targetRotation}");
            }

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

            // Wait for data to be available with 10-minute timeout
            int timeout = 600000; // 10 minutes in milliseconds
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
                // Read 6 joint angles (6 doubles = 48 bytes)
                // BinaryReader will block until data is available
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
