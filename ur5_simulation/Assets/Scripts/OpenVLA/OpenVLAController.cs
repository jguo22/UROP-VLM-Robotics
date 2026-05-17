using System;
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
//
// Body shape: { "observation": { "full_image": <uint8 HxWx3 RGB ndarray>, ... },
//               "instruction": <str> }
// Add "*wrist*" image keys inside "observation" if the server was launched with
// num_images_in_input>1, and "state" if it was launched with --use_proprio.
// The action un-normalization key is configured server-side.
public class OpenVLAController : MonoBehaviour
{
    [Header("OpenVLA Server")]
    public string serverUrl = "http://localhost:8777/act";
    public string instruction = "put blue, red, and green gears into planetary gearbox";

    [Header("Capture")]
    [SerializeField]
    private ObservationCapture observation;

    [Header("Control")]
    [Tooltip("How often to query the VLA (Hz). BridgeData V2 trained at 5 Hz.")]
    public float controlFrequency = 5.0f;

    [Tooltip("Automatically query the server every 1/controlFrequency seconds")]
    public bool autoQuery = true;

    [Header("Action Scaling")]
    [Tooltip("Training control frequency (BridgeData V2 = 5 Hz)")]
    private float trainingFrequency = 5.0f;

    [Tooltip("Additional workspace scale factor (tune empirically)")]
    private float workspaceScale = 2.0f;

    private UR5Controller robotController;
    private HttpClient httpClient;
    private bool requestInFlight = false;
    private float lastQueryTime = -1f;

    void Start()
    {
        robotController = GetComponent<UR5Controller>();
        if (robotController == null)
        {
            Debug.LogError("OpenVLAServer: UR5Controller component not found!");
            enabled = false;
            return;
        }

        if (observation == null)
        {
            Debug.LogError("OpenVLAController: observation not assigned.");
            enabled = false;
            return;
        }
        observation.Initialize();

        httpClient = new HttpClient { Timeout = TimeSpan.FromSeconds(30) };

        Debug.Log($"OpenVLAServer client targeting {serverUrl}");
    }

    void Update()
    {
        if (!autoQuery || requestInFlight)
            return;

        float interval = 1f / controlFrequency;
        if (Time.time - lastQueryTime < interval)
            return;

        lastQueryTime = Time.time;
        _ = QueryAndApplyAsync();
    }

    public void QueryOnce()
    {
        if (requestInFlight)
            return;
        _ = QueryAndApplyAsync();
    }

    async Task QueryAndApplyAsync()
    {
        requestInFlight = true;
        try
        {
            // Image capture must happen on the main thread
            byte[] rgbBytes = CaptureImageRGB();
            string payload = BuildPayload(
                rgbBytes,
                ObservationCapture.ImageWidth,
                ObservationCapture.ImageHeight,
                this.observation.CaptureState().jointAnglesDeg
            );

            using var content = new StringContent(payload, Encoding.UTF8, "application/json");
            var response = await httpClient.PostAsync(serverUrl, content);
            response.EnsureSuccessStatusCode();
            string body = await response.Content.ReadAsStringAsync();

            float[] action = ParseActionResponse(body);
            if (action == null || action.Length < 7)
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

            ApplyAction(action);
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

    byte[] CaptureImageRGB()
    {
        // Unity's ReadPixels gives bottom-up rows; OpenVLA/PIL expects top-down.
        byte[] raw = observation.CaptureScreenshotRgb();
        return FlipImageVertically(
            raw,
            ObservationCapture.ImageWidth,
            ObservationCapture.ImageHeight,
            3
        );
    }

    static byte[] FlipImageVertically(byte[] src, int width, int height, int channels)
    {
        byte[] dst = new byte[src.Length];
        int rowSize = width * channels;
        for (int y = 0; y < height; y++)
            Buffer.BlockCopy(src, y * rowSize, dst, (height - 1 - y) * rowSize, rowSize);
        return dst;
    }

    string BuildPayload(byte[] imageBytes, int width, int height, float[] state)
    {
        // json_numpy encodes an ndarray as {"__ndarray__": <base64 raw bytes>, "dtype": "...", "shape": [...]}.
        // deploy.py expects {"observation": {...}, "instruction": str}.
        var payload = new
        {
            full_image = new
            {
                __numpy__ = Convert.ToBase64String(imageBytes),
                dtype = "uint8",
                shape = new[] { height, width, 3 },
            },
            state = state,
            instruction,
        };
        string result = JsonConvert.SerializeObject(payload);
        print(result);
        return result;
    }

    static float[] ParseActionResponse(string body)
    {
        // json_numpy ndarray: {"__ndarray__": "<base64 raw bytes>", "dtype": "<f4", "shape": [...]}
        // (some json_numpy versions use the key "__numpy__" — accept either).
        JObject obj;
        try
        {
            obj = JObject.Parse(body);
        }
        catch (JsonException)
        {
            return null;
        }

        var b64 = (string)(obj["__ndarray__"] ?? obj["__numpy__"]);
        var dtype = (string)obj["dtype"];
        if (b64 == null || dtype == null)
            return null;

        return DecodeFloatArray(Convert.FromBase64String(b64), dtype);
    }

    // Reinterprets json_numpy's raw little-endian bytes as a float[], widening float64 to float32.
    static float[] DecodeFloatArray(byte[] bytes, string dtype)
    {
        if (dtype.Contains("f4") || dtype.Contains("float32"))
        {
            var arr = new float[bytes.Length / 4];
            Buffer.BlockCopy(bytes, 0, arr, 0, arr.Length * 4);
            return arr;
        }
        if (dtype.Contains("f8") || dtype.Contains("float64"))
        {
            var arr = new float[bytes.Length / 8];
            for (int i = 0; i < arr.Length; i++)
                arr[i] = (float)BitConverter.ToDouble(bytes, i * 8);
            return arr;
        }

        Debug.LogWarning($"OpenVLAServer: unsupported action dtype '{dtype}'");
        return null;
    }

    void ApplyAction(float[] action)
    {
        Debug.Log($"OpenVLA action (raw): {string.Join(", ", action)}");

        float frequencyScale = trainingFrequency / Mathf.Max(controlFrequency, 0.01f);
        float[] scaled = new float[7];
        for (int i = 0; i < 6; i++)
            scaled[i] = action[i] * frequencyScale * workspaceScale;
        scaled[6] = action[6];

        Vector3 deltaPosition = new Vector3(scaled[0], scaled[1], scaled[2]);
        Quaternion deltaRotation = Quaternion.Euler(scaled[3], scaled[4], scaled[5]);
        robotController.MoveDelta(deltaPosition, deltaRotation);

        // OpenVLA gripper convention: 1.0 = open, 0.0 = closed (BridgeData V2)
        robotController.SetSuction(scaled[6] < 0.5f);
    }

    void OnDestroy()
    {
        httpClient?.Dispose();
    }
}
