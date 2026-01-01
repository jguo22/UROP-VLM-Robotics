using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class UR5TCPServer : MonoBehaviour
{
    [Header("Settings")]
    public int port = 5000;
    public Camera robotCamera;
    public int imageWidth = 224;
    public int imageHeight = 224;

    [Header("Action Scaling")]
    [Tooltip("Control frequency in Hz (BridgeData V2 training freq = 5 Hz)")]
    private float controlFrequency = 5.0f;
    [Tooltip("BridgeData V2 was collected at 5 Hz")]
    private float trainingFrequency = 5.0f;
    [Tooltip("Additional workspace scale factor (adjust empirically, 1.0 = no scaling)")]
    private float workspaceScale = 2.0f;

    private TcpListener server;
    private Thread serverThread;
    private bool isRunning = true;
    private RenderTexture renderTexture;
    private Texture2D cameraImage;

    // Thread-safe queue for main thread actions
    private static readonly Queue<Action> _executionQueue = new Queue<Action>();
    private static readonly object _lock = new object();

    private UR5Controller robotController;

    void Start()
    {
        // Get UR5Controller component
        robotController = GetComponent<UR5Controller>();
        if (robotController == null)
        {
            Debug.LogError("VLAServer: UR5Controller component not found!");
            enabled = false;
            return;
        }

        // Initialize camera and render texture on the main thread
        if (robotCamera == null)
            robotCamera = Camera.main;

        if (robotCamera == null)
        {
            Debug.LogError("No camera found! Please assign a camera to the robotCamera field.");
            return;
        }

        // Set up render texture
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32)
        {
            name = "NetworkStreamRT",
            antiAliasing = 1,
            filterMode = FilterMode.Bilinear
        };

        // Create camera image
        cameraImage = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        // Create a second camera for network streaming
        CreateNetworkCamera();

        // Start server thread
        serverThread = new Thread(ServerLoop)
        {
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.BelowNormal
        };
        serverThread.Start();

        Debug.Log($"UR5 TCP Server started on port {port}");
    }

    void CreateNetworkCamera()
    {
        // This runs on main thread during Start()
        var networkCamObj = new GameObject("NetworkStreamCamera");
        networkCamObj.transform.SetParent(robotCamera.transform);
        networkCamObj.transform.localPosition = Vector3.zero;
        networkCamObj.transform.localRotation = Quaternion.identity;

        var networkCam = networkCamObj.AddComponent<Camera>();
        networkCam.CopyFrom(robotCamera);
        networkCam.depth = robotCamera.depth - 1;
        networkCam.targetTexture = renderTexture;
    }

    void Update()
    {
        // Process all pending actions on the main thread
        while (_executionQueue.Count > 0)
        {
            Action action = null;
            lock (_lock)
            {
                if (_executionQueue.Count > 0)
                    action = _executionQueue.Dequeue();
            }
            action?.Invoke();
        }
    }

    public static void RunOnMainThread(Action action)
    {
        if (action == null) return;

        lock (_lock)
        {
            _executionQueue.Enqueue(action);
        }
    }

    void ServerLoop()
    {
        try
        {
            server = new TcpListener(IPAddress.Any, port);
            server.Start();

            while (isRunning)
            {
                if (server.Pending())
                {
                    using (TcpClient client = server.AcceptTcpClient())
                    using (NetworkStream stream = client.GetStream())
                    {
                        Debug.Log("Client connected");

                        while (client.Connected && isRunning)
                        {
                            try
                            {
                                int messageType = stream.ReadByte();
                                if (messageType == -1) break;

                                switch (messageType)
                                {
                                    case 0: // Image request
                                        SendImage(stream);
                                        break;
                                    case 1: // Action
                                        ProcessAction(stream);
                                        break;
                                    default:
                                        Debug.LogWarning($"Unknown message type: {messageType}");
                                        break;
                                }
                            }
                            catch (Exception e)
                            {
                                Debug.LogError($"Error in client loop: {e}");
                                break;
                            }
                        }
                    }
                }
                Thread.Sleep(10);
            }
        }
        catch (Exception e)
        {
            if (isRunning)
                Debug.LogError($"Server error: {e}");
        }
    }

    void SendImage(NetworkStream stream)
    {
        try
        {
            var tcs = new TaskCompletionSource<bool>();

            RunOnMainThread(() =>
            {
                try
                {
                    if (renderTexture == null || cameraImage == null)
                    {
                        Debug.LogError("Render texture or camera image is null!");
                        tcs.SetResult(false);
                        return;
                    }

                    // Read from render texture
                    RenderTexture.active = renderTexture;
                    cameraImage.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
                    cameraImage.Apply();
                    RenderTexture.active = null;

                    // Encode to JPG
                    byte[] imageBytes = cameraImage.EncodeToJPG();

                    if (imageBytes == null || imageBytes.Length == 0)
                    {
                        Debug.LogError("Failed to encode image to JPG");
                        tcs.SetResult(false);
                        return;
                    }

                    Debug.Log($"Sending {imageBytes.Length} bytes of image data");

                    // Send image size (4 bytes, big-endian)
                    byte[] sizeBytes = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(imageBytes.Length));
                    stream.Write(sizeBytes, 0, 4);

                    // Send image data
                    stream.Write(imageBytes, 0, imageBytes.Length);
                    stream.Flush();

                    tcs.SetResult(true);
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error in SendImage: {e}");
                    tcs.SetResult(false);
                }
            });

            // Wait for the main thread to complete
            if (!tcs.Task.Wait(TimeSpan.FromSeconds(5))) // 5 second timeout
            {
                Debug.LogError("Timed out waiting for image capture");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error in SendImage (outer): {e}");
        }
    }

    void ProcessAction(NetworkStream stream)
    {
        try
        {
            // Read action size (4 bytes, big-endian)
            byte[] sizeBytes = new byte[4];
            if (stream.Read(sizeBytes, 0, 4) != 4)
                throw new Exception("Failed to read action size");

            int actionSize = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(sizeBytes, 0));

            // Read action data
            byte[] buffer = new byte[actionSize];
            int bytesRead = 0;
            while (bytesRead < actionSize)
            {
                int read = stream.Read(buffer, bytesRead, actionSize - bytesRead);
                if (read == 0)
                    throw new Exception("Connection closed while reading action");
                bytesRead += read;
            }

            // Convert to float array
            if (bytesRead == actionSize && actionSize % 4 == 0)
            {
                float[] action = new float[actionSize / 4];
                Buffer.BlockCopy(buffer, 0, action, 0, actionSize);

                // Process on main thread
                RunOnMainThread(() =>
                {
                    // Actions received from OpenVLA's predict_action() are already unnormalized
                    // (in physical units: meters for position, radians for rotation)
                    Debug.Log($"Received action (raw): {string.Join(", ", action)}");

                    // Scale actions for control frequency and workspace differences
                    float[] scaledAction = new float[action.Length];

                    // Frequency scaling: adjust for dt difference
                    // If running faster than training freq, take smaller steps
                    float frequencyScale = trainingFrequency / controlFrequency;

                    // Apply scaling to position and rotation deltas (first 6 dimensions)
                    for (int i = 0; i < 6; i++)
                    {
                        scaledAction[i] = action[i] * frequencyScale * workspaceScale;
                    }

                    // Gripper action (last dimension) - no scaling needed
                    scaledAction[6] = action[6];

                    Debug.Log($"Scaled action (freq={controlFrequency}Hz, workspace={workspaceScale}x): {string.Join(", ", scaledAction)}");
                    Vector3 deltaPosition = new Vector3(scaledAction[0], scaledAction[1], scaledAction[2]);
                    Quaternion deltaRotation = Quaternion.Euler(scaledAction[3], scaledAction[4], scaledAction[5]);
                    robotController.ApplyDeltaAction(deltaPosition, deltaRotation);
                });

                // Send success ACK
                stream.WriteByte(1);
                return;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"Error processing action: {e}");
        }

        // Send failure ACK
        try { stream.WriteByte(0); } catch { }
    }

    void OnDestroy()
    {
        isRunning = false;
        server?.Stop();
        serverThread?.Join(1000);

        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
        if (cameraImage != null)
            Destroy(cameraImage);
    }
}
