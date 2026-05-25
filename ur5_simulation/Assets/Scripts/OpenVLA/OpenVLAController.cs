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
public class OpenVLAController : MonoBehaviour
{
    [Header("OpenVLA Server")] [SerializeField]
    private string serverUrl = "http://localhost:8777/act";

    [SerializeField] private string instruction = "put blue, red, and green gears into planetary gearbox";

    [Header("Capture")] [SerializeField] private ObservationCapture observationCapture;

    [Header("Control")] [Tooltip("How often to query the VLA (Hz).")] [SerializeField]
    private float controlFrequency = 10.0f;

    [Tooltip("Automatically query the server every 1/controlFrequency seconds")]
    public bool autoQuery = true;

    [Header("Action Scaling")] [Tooltip("Training control frequency")] [SerializeField]
    private float trainingFrequency = 10.0f;

    private HttpClient httpClient;
    private float lastQueryTime = -1f;
    private bool requestInFlight;

    private UR5Controller robotController;

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
        if (!autoQuery || requestInFlight)
            return;

        float interval = 1f / controlFrequency;
        if (Time.time - lastQueryTime < interval)
            return;

        lastQueryTime = Time.time;
        _ = QueryAndApplyAsync();
    }

    private void OnDestroy()
    {
        httpClient?.Dispose();
    }

    private async Task QueryAndApplyAsync()
    {
        requestInFlight = true;
        try
        {
            string payload = GetJsonObservation();

            using var content = new StringContent(payload, Encoding.UTF8, "application/json");
            HttpResponseMessage response = await httpClient.PostAsync(serverUrl, content);
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

    private string GetJsonObservation()
    {
        // Image capture must happen on the main thread
        byte[] imageBytes = CaptureImageRGB();

        ObservationCapture.Observation observation = observationCapture.CaptureState();
        float[] state = new float[observation.jointAnglesDeg.Length + 2];
        observation.jointAnglesDeg.CopyTo(state, 0);
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

    private static float[] ParseActionResponse(string body)
    {
        JToken obj = JArray.Parse(body)[0];

        string b64 = (string)obj["__numpy__"];
        string dtype = (string)obj["dtype"];

        if (b64 == null || dtype == null)
            return null;
        return DecodeFloatArray(Convert.FromBase64String(b64), dtype);
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
        Debug.Log($"OpenVLA action (raw): {string.Join(", ", action)}");

        float frequencyScale = trainingFrequency / Mathf.Max(controlFrequency, 0.01f);
        float[] scaled = new float[7];
        for (int i = 0; i < 6; i++)
            scaled[i] = action[i] * frequencyScale;
        scaled[6] = action[6];

        var deltaPosition = new Vector3(scaled[0], scaled[1], scaled[2]);
        Quaternion deltaRotation = Quaternion.Euler(scaled[3], scaled[4], scaled[5]);
        robotController.MoveDelta(deltaPosition, deltaRotation);

        robotController.SetSuction(scaled[6] < 0.5f);
    }
}