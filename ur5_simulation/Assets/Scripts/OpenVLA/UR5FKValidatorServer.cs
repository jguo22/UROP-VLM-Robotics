using System;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// TCP server that broadcasts UR5 robot state (end effector position and joint angles)
/// to connected clients in binary format for forward kinematics validation.
///
/// Binary protocol: 9 doubles (72 bytes) - [ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6]
/// Uses async I/O on main thread - no state caching or background threads needed.
/// </summary>
public class UR5FKValidatorServer : MonoBehaviour
{
    [Header("Server Settings")]
    [Tooltip("TCP port for FK validation server")]
    public int port = 5005;

    [Tooltip("Update rate in Hz (how often to send robot state)")]
    public float updateRate = 30f;

    [Header("Debug")]
    public bool showDebugInfo = true;

    private TcpListener server;
    private bool isRunning = true;
    private UR5Controller ur5controller;

    async void Start()
    {
        // Get UR5Controller component
        ur5controller = GetComponent<UR5Controller>();
        if (ur5controller == null)
        {
            Debug.LogError("UR5FKValidatorServer: UR5Controller component not found!");
            enabled = false;
            return;
        }

        Debug.Log($"UR5 FK Validator Server starting on port {port} (update rate: {updateRate} Hz)");

        // Start async server
        await HandleClientAsync();
    }

    /// <summary>
    /// Accept client connections asynchronously
    /// </summary>
    async Task HandleClientAsync()
    {
        try
        {
            server = new TcpListener(IPAddress.Any, port);
            server.Start();
            Debug.Log($"FK Validator Server listening on port {port}");

            while (isRunning)
            {
                try
                {
                    // Wait for client connection (async, non-blocking)
                    TcpClient client = await server.AcceptTcpClientAsync();
                    Debug.Log($"FK Validator Client connected from {client.Client.RemoteEndPoint}");

                    // Stream data to client in separate task (allows multiple clients)
                    _ = StreamToClientAsync(client);
                }
                catch (ObjectDisposedException)
                {
                    // Server was stopped
                    break;
                }
                catch (Exception e)
                {
                    if (isRunning)
                    {
                        Debug.LogError($"Error accepting client: {e.Message}");
                    }
                }
            }
        }
        catch (Exception e)
        {
            if (isRunning)
            {
                Debug.LogError($"FK Validator Server error: {e}");
            }
        }
        finally
        {
            server?.Stop();
            Debug.Log("FK Validator Server stopped");
        }
    }

    /// <summary>
    /// Stream robot state to individual client
    /// </summary>
    async Task StreamToClientAsync(TcpClient client)
    {
        try
        {
            using (client)
            using (NetworkStream stream = client.GetStream())
            {
                float updateInterval = 1f / updateRate;
                float lastUpdateTime = Time.time;

                // Stream robot state to client
                while (client.Connected && isRunning)
                {
                    // Rate limiting
                    if (Time.time - lastUpdateTime >= updateInterval)
                    {
                        try
                        {
                            // Get fresh robot state (on main thread - safe!)
                            double[] state = GetRobotState();

                            // Send state asynchronously
                            await SendBinaryStateAsync(stream, state);

                            lastUpdateTime = Time.time;

                            if (showDebugInfo)
                            {
                                Debug.Log($"Sent state - EE: ({state[0]:F4}, {state[1]:F4}, {state[2]:F4})");
                            }
                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning($"Error sending to client: {e.Message}");
                            break;
                        }
                    }

                    // Small delay to prevent tight loop (async, non-blocking)
                    await Task.Delay(1);
                }
            }

            Debug.Log("FK Validator Client disconnected");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Client handler error: {e.Message}");
        }
    }

    /// <summary>
    /// Get current robot state from Unity components (main thread only)
    /// </summary>
    double[] GetRobotState()
    {
        double[] state = new double[9];

        // Get end effector pose
        var (position, rotation) = ur5controller.GetEndEffectorPose();

        // Get joint angles (in radians)
        float[] jointAngles = ur5controller.GetJointAngles();

        if (jointAngles == null || jointAngles.Length != 6)
        {
            Debug.LogWarning("Invalid joint angles received");
            return state;
        }

        // this is Unity to ROS conversion 
        // // End effector position
        // // Unity uses y as up and down, but convention is z as up and down
        // double[] conventionalPosition = { position.x, position.z, position.y };
        // // rtb coordinates are rotated by 90deg?
        // state[0] = conventionalPosition[1];
        // state[1] = -conventionalPosition[0];
        // state[2] = conventionalPosition[2];

        state[0] = position.x;
        state[1] = position.y;
        state[2] = position.z;

        // Joint angles (radians)
        for (int i = 0; i < 6; i++)
        {
            state[3 + i] = jointAngles[i];
        }

        return state;
    }

    /// <summary>
    /// Send robot state in binary format (72 bytes) asynchronously
    /// Format: 9 doubles (little-endian) - [ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6]
    /// </summary>
    async Task SendBinaryStateAsync(NetworkStream stream, double[] state)
    {
        // Pack 9 doubles into 72 bytes (little-endian)
        byte[] buffer = new byte[72];

        for (int i = 0; i < 9; i++)
        {
            byte[] doubleBytes = BitConverter.GetBytes(state[i]);
            Array.Copy(doubleBytes, 0, buffer, i * 8, 8);
        }

        // Send binary data asynchronously
        await stream.WriteAsync(buffer, 0, 72);
        await stream.FlushAsync();
    }

    void OnDestroy()
    {
        Cleanup();
    }

    void OnApplicationQuit()
    {
        Cleanup();
    }

    void Cleanup()
    {
        // Stop server
        isRunning = false;

        try
        {
            server?.Stop();
        }
        catch (Exception e)
        {
            Debug.LogError($"Error stopping server: {e.Message}");
        }

        Debug.Log("UR5 FK Validator Server cleaned up");
    }
}
