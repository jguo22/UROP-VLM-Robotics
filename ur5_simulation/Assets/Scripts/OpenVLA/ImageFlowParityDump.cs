using System.IO;
using UnityEngine;
using static ConstantsUR5;

// Debug helper for verifying the two image pipelines agree on pixels.
//
//   File flow (EpisodeRecorder -> rlds_dataset_builder):
//       ObservationCapture.CaptureScreenshotPng() -> EncodeToPNG -> images/NNNNNN.png
//       Python reads it with PIL.Image.open(...).convert("RGB").
//
//   Wire flow (OpenVLAController -> deploy.py):
//       ObservationCapture.CaptureScreenshotRgb() (bottom-up) -> FlipImageVertically
//       -> json_numpy {__numpy__ base64, dtype, shape}; the server decodes with
//       np.frombuffer(...).reshape(shape).
//
// Both start from the same rendered buffer, so for a single render this dumps
// both artifacts side by side (file_flow.png + wire_flow.json) — the wire artifact
// is the actual json_numpy envelope, so its self-declared dtype/shape are tested
// too. python/tests/test_image_flow_parity.py loads each through its real decode
// path and asserts the arrays are pixel-identical.
//
// Drop this on any object that has an ObservationCapture wired up, enter play
// mode, and press the on-screen "Dump image parity" button (or call DumpOnce()).
public class ImageFlowParityDump : MonoBehaviour
{
    [SerializeField]
    private ObservationCapture observationCapture;

    [SerializeField]
    private Rect dumpButtonRect = new(20f, 70f, 200f, 40f);

    [SerializeField]
    private Rect syntheticButtonRect = new(20f, 120f, 200f, 40f);

    private void OnGUI()
    {
        if (GUI.Button(dumpButtonRect, "Dump image parity"))
            DumpOnce();
        if (GUI.Button(syntheticButtonRect, "Dump synthetic parity"))
            DumpSyntheticParity();
    }

    // Definitive format/orientation/channel test. Comparing two real renders only
    // catches a mismatch if the scene is asymmetric, and at 224x224 a height/width
    // transpose doesn't even change the array shape. So instead we run a *known*
    // positional pattern through the same two code paths and let Python check each
    // decoded array against the absolute expected pattern (not just against each
    // other). Every axis and channel is independently identifiable:
    //
    //   intended top-down image P:  R = column, G = row (0..223 gradients),
    //                               B = 255 (constant channel sentinel).
    //
    // A vertical flip reverses G, a horizontal flip reverses R, an H/W transpose
    // swaps the R and G gradients, and any channel swap moves the 255 sentinel —
    // each lands at a known location, so the failure mode is unambiguous.
    //
    // Writes file_flow.png + wire_flow.json + meta.json into Exports/parity/
    // synthetic_<timestamp>/. Checked by python/tests/test_image_format_parity.py.
    public string DumpSyntheticParity()
    {
        int w = ObservationCapture.ImageWidth;
        int h = ObservationCapture.ImageHeight;

        // Match the real capture buffer's format exactly (RGB24, no mipmaps).
        var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
        var pixels = new Color32[w * h];
        for (int y = 0; y < h; y++)
        {
            // Unity textures are bottom-up (y=0 is the bottom row), and EncodeToPNG
            // flips to top-down. Store P so the encoded PNG's top row is rowTop==0,
            // exactly mirroring how ReadPixels(bottom-up) + EncodeToPNG behaves.
            int rowTop = h - 1 - y;
            for (int x = 0; x < w; x++)
            {
                pixels[y * w + x] = new Color32(
                    (byte)(x % 256), // R = column
                    (byte)(rowTop % 256), // G = row (from top)
                    255, // B = constant sentinel
                    255
                );
            }
        }
        tex.SetPixels32(pixels);
        tex.Apply(false);

        // Same two downstream paths the real flows use, on this known buffer.
        byte[] pngBytes = tex.EncodeToPNG();
        byte[] rawRgb = tex.GetRawTextureData();
        byte[] wireBytes = FlipImageVertically(rawRgb, w, h, 3);

        Destroy(tex);

        string wireJson = WirePayloadJson(wireBytes, w, h);

        string folder = ProjectPaths.Get(
            Path.Combine(
                "ur5_simulation",
                "Exports",
                "parity",
                "synthetic_" + System.DateTime.Now.ToString("yyyyMMdd_HHmmss")
            )
        );
        Directory.CreateDirectory(folder);

        File.WriteAllBytes(Path.Combine(folder, "file_flow.png"), pngBytes);
        File.WriteAllText(Path.Combine(folder, "wire_flow.json"), wireJson);
        File.WriteAllText(
            Path.Combine(folder, "meta.json"),
            $"{{\"type\":\"synthetic_positional\",\"width\":{w},\"height\":{h},"
            + "\"channels\":3,\"pattern\":\"R=col,G=rowTop,B=255\"}}"
        );

        Debug.Log(
            $"ImageFlowParityDump: wrote synthetic parity ({w}x{h}) → {folder}"
        );
        return folder;
    }

    // Renders one frame and writes file_flow.png + wire_flow.bin into a fresh
    // Exports/parity/<timestamp>/ folder. Returns that folder path.
    public string DumpOnce()
    {
        if (observationCapture == null)
        {
            Debug.LogError("ImageFlowParityDump: observationCapture not assigned.");
            return null;
        }

        observationCapture.Initialize();

        // File flow: exactly what EpisodeRecorder writes to images/NNNNNN.png.
        byte[] pngBytes = observationCapture.CaptureScreenshotPng();

        // Wire flow: exactly what OpenVLAController sends after the vertical flip.
        // GetRawTextureData() is bottom-up; OpenVLA/PIL expect top-down.
        byte[] rawRgb = observationCapture.CaptureScreenshotRgb();
        byte[] wireBytes = FlipImageVertically(
            rawRgb,
            ObservationCapture.ImageWidth,
            ObservationCapture.ImageHeight,
            3
        );

        string folder = ProjectPaths.Get(
            Path.Combine(
                "ur5_simulation",
                "Exports",
                "parity",
                System.DateTime.Now.ToString("yyyyMMdd_HHmmss")
            )
        );
        Directory.CreateDirectory(folder);

        File.WriteAllBytes(Path.Combine(folder, "file_flow.png"), pngBytes);
        File.WriteAllText(
            Path.Combine(folder, "wire_flow.json"),
            WirePayloadJson(wireBytes, ObservationCapture.ImageWidth, ObservationCapture.ImageHeight)
        );

        Debug.Log(
            $"ImageFlowParityDump: wrote file_flow.png ({pngBytes.Length} B) and "
            + $"wire_flow.json → {folder}"
        );
        return folder;
    }

    // The exact json_numpy envelope OpenVLAController.GetJsonObservation sends for
    // full_image, so the dumped wire artifact carries the real dtype + shape
    // declaration (shape is [H, W, 3], matching the controller) and the base64
    // round-trip — not just raw bytes. Python decodes it via the payload's own
    // shape/dtype, exactly like offline_replay.decode_numpy.
    private static string WirePayloadJson(byte[] wireBytes, int width, int height)
    {
        var payload = new
        {
            __numpy__ = System.Convert.ToBase64String(wireBytes),
            dtype = "uint8",
            shape = new[] { height, width, 3 },
        };
        return Newtonsoft.Json.JsonConvert.SerializeObject(payload);
    }

    // Mirrors OpenVLAController.FlipImageVertically; kept in sync intentionally so
    // the dumped wire bytes match the deployed path byte-for-byte.
    private static byte[] FlipImageVertically(byte[] src, int width, int height, int channels)
    {
        byte[] dst = new byte[src.Length];
        int rowSize = width * channels;
        for (int y = 0; y < height; y++)
            System.Buffer.BlockCopy(src, y * rowSize, dst, (height - 1 - y) * rowSize, rowSize);
        return dst;
    }
}
