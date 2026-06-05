using System;
using System.Collections.Generic;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

// HTTP client that talks to an OpenVLA-OFT `deploy.py` server (FastAPI, POST /act).
// Captures a camera frame, packages it with the instruction as a json_numpy
// payload, posts to the server, and applies the returned action chunk to the
// UR5 each control tick.
public class OpenVLAController : MonoBehaviour
{
    private const int ActionDim = 7;

    [Header("OpenVLA Server")] 
    private readonly string serverUrl = "http://localhost:8778/act";

    [SerializeField] private string instruction = "put blue, red, and green gears into planetary gearbox";

    [Header("Capture")] [SerializeField] private ObservationCapture observationCapture;

    [Header("Control")] [Tooltip("How often to query the VLA (Hz).")] [SerializeField]
    private float controlFrequency = 10.0f;

    [Tooltip("Automatically query the server every 1/controlFrequency seconds")]
    public bool autoQuery = true;

    [Header("Action Scaling")] [Tooltip("Training control frequency")] [SerializeField]
    private float trainingFrequency = 10.0f;

    [Header("UI")] [SerializeField] private Rect startButtonRect = new(20f, 20f, 160f, 40f);

    private readonly Queue<float[]> pendingActions = new();

    private HttpClient httpClient;
    private float lastStepTime = -1f;
    private bool requestInFlight;

    private UR5Controller robotController;
    private bool started;

    private void Start()
    {
        robotController = GetComponent<UR5Controller>();
        if (robotController == null)
        {
            Debug.LogError("OpenVLAServer: UR5Controller component not found!");
            enabled = false;
            return;
        }

        if (observationCapture == null)
        {
            Debug.LogError("OpenVLAController: observation not assigned.");
            enabled = false;
            return;
        }

        observationCapture.Initialize();

        httpClient = new HttpClient { Timeout = TimeSpan.FromSeconds(30) };

        Debug.Log($"OpenVLAServer client targeting {serverUrl}");
    }

    private void Update()
    {
        if (!started)
            return;

        float interval = 1f / controlFrequency;
        if (Time.time - lastStepTime < interval)
            return;

        if (pendingActions.Count > 0)
        {
            lastStepTime = Time.time;
            ApplyAction(pendingActions.Dequeue());
            return;
        }

        if (autoQuery && !requestInFlight)
            _ = QueryAsync();
    }

    private void OnDestroy()
    {
        httpClient?.Dispose();
    }

    private void OnGUI()
    {
        string label = started ? "Stop OpenVLA" : "Start OpenVLA";
        if (GUI.Button(startButtonRect, label))
            started = !started;
    }

    private async Task QueryAsync()
    {
        requestInFlight = true;
        try
        {
            string payload = GetJsonObservation();

            using var content = new StringContent(payload, Encoding.UTF8, "application/json");
            HttpResponseMessage response = await httpClient.PostAsync(serverUrl, content);
            response.EnsureSuccessStatusCode();
            string body = await response.Content.ReadAsStringAsync();

            float[][] chunk = ParseActionChunk(body);
            if (chunk == null || chunk.Length == 0)
            {
                // deploy.py's catch-all returns the bare string "error" on any exception;
                // the real traceback is in the server's stdout on the compute node
                // (usually a bad unnorm_key or an unexpected image shape).
                string hint =
                    body.IndexOf("error", StringComparison.OrdinalIgnoreCase) >= 0
                        ? " — server raised an exception; check the deploy.py log on the compute node"
                        : "";
                Debug.LogWarning($"OpenVLAServer: bad action response: {body}{hint}");
                return;
            }

            foreach (float[] a in chunk)
                pendingActions.Enqueue(a);
        }
        catch (Exception e)
        {
            Debug.LogError($"OpenVLAServer query failed: {e.Message}");
        }
        finally
        {
            requestInFlight = false;
        }
    }

    private string GetJsonObservation()
    {
        // Image capture must happen on the main thread
        byte[] imageBytes = CaptureImageRGB();

        ObservationCapture.Observation observation = observationCapture.CaptureState();
        float[] anglesDeg = observation.jointAnglesDeg;
        float[] state = new float[anglesDeg.Length + 2];
        // OpenVLA-OFT proprio is trained in radians.
        for (int i = 0; i < anglesDeg.Length; i++)
            state[i] = anglesDeg[i] * Mathf.Deg2Rad;
        state[^2] = 0;
        state[^1] = observation.suctionOn ? 1 : -1;

        var payloadDict = new
        {
            full_image = new
            {
                __numpy__ = Convert.ToBase64String(imageBytes),
                dtype = "uint8",
                shape = new[] { ObservationCapture.ImageWidth, ObservationCapture.ImageHeight, 3 }
            },
            state,
            instruction
        };

        return JsonConvert.SerializeObject(payloadDict);
    }

    private byte[] CaptureImageRGB()
    {
        // Unity's ReadPixels gives bottom-up rows; OpenVLA/PIL expects top-down.
        byte[] raw = observationCapture.CaptureScreenshotRgb();
        return FlipImageVertically(
            raw,
            ObservationCapture.ImageWidth,
            ObservationCapture.ImageHeight,
            3
        );
    }

    private static byte[] FlipImageVertically(byte[] src, int width, int height, int channels)
    {
        byte[] dst = new byte[src.Length];
        int rowSize = width * channels;
        for (int y = 0; y < height; y++)
            Buffer.BlockCopy(src, y * rowSize, dst, (height - 1 - y) * rowSize, rowSize);
        return dst;
    }

    // deploy.py returns a json_numpy array of shape (chunk_size, ActionDim).
    // Slice the flat float buffer into one float[ActionDim] per step.
    private static float[][] ParseActionChunk(string body)
    {
        JToken obj = JArray.Parse(body)[0];

        string b64 = (string)obj["__numpy__"];
        string dtype = (string)obj["dtype"];

        if (b64 == null || dtype == null)
            return null;

        float[] flat = DecodeFloatArray(Convert.FromBase64String(b64), dtype);
        if (flat == null || flat.Length < ActionDim)
            return null;

        int chunkSize = flat.Length / ActionDim;
        float[][] chunk = new float[chunkSize][];
        for (int i = 0; i < chunkSize; i++)
        {
            chunk[i] = new float[ActionDim];
            Array.Copy(flat, i * ActionDim, chunk[i], 0, ActionDim);
        }

        return chunk;
    }

    // Reinterprets json_numpy's raw little-endian bytes as a float[], widening float64 to float32.
    private static float[] DecodeFloatArray(byte[] bytes, string dtype)
    {
        if (dtype.Contains("f4") || dtype.Contains("float32"))
        {
            float[] arr = new float[bytes.Length / 4];
            Buffer.BlockCopy(bytes, 0, arr, 0, arr.Length * 4);
            return arr;
        }

        if (dtype.Contains("f8") || dtype.Contains("float64"))
        {
            float[] arr = new float[bytes.Length / 8];
            for (int i = 0; i < arr.Length; i++)
                arr[i] = (float)BitConverter.ToDouble(bytes, i * 8);
            return arr;
        }

        Debug.LogWarning($"OpenVLAServer: unsupported action dtype '{dtype}'");
        return null;
    }

    private void ApplyAction(float[] action)
    {
        // for (int i = 0; i < action.Length-1; i++)
        // {
        //     action[i] *= 0.7f;
        // }
        Debug.Log($"OpenVLA action (raw): {string.Join(", ", action)}");

        var deltaPosition = new Vector3(action[0], action[1], action[2]);

        // action[3..5] is an axis-angle rotation vector in RADIANS (scipy as_rotvec),
        // matching ur5_unity_dataset_builder.py. Decode as axis-angle, not Euler.
        var rotVec = new Vector3(action[3], action[4], action[5]);
        float angleRad = rotVec.magnitude;
        Quaternion deltaRotation = angleRad < 1e-8f
            ? Quaternion.identity
            : Quaternion.AngleAxis(angleRad * Mathf.Rad2Deg, rotVec / angleRad);

        robotController.MoveDelta(deltaPosition, deltaRotation);

        robotController.SetSuction(action[6] < 0.5f);
    }
}