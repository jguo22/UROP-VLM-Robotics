using UnityEngine;
using static ConstantsUR5;

/// <summary>
/// Reads world state (end-effector pose, joint angles, suction, attached block)
/// and renders the recording camera into a fixed-size buffer for screenshots.
/// Owns render resources; safe to reuse across episodes.
/// </summary>
public class ObservationCapture : MonoBehaviour
{
    public struct Observation
    {
        public Vector3 endEffectorPosition;
        public Quaternion endEffectorRotation;
        public bool suctionOn;
        public float[] jointAnglesDeg;
        public bool blockAttracted;
    }

    public const int ImageWidth = 224;
    public const int ImageHeight = 224;

    [SerializeField]
    private Camera recordingCamera;

    [SerializeField]
    private SuctionController suctionController;

    [SerializeField]
    private RobotArmSetup robotArmSetup;

    private Transform endEffector;
    private RenderTexture renderTexture;
    private Texture2D screenshotBuffer;
    private bool resourcesAllocated = false;

    /// <summary>
    /// Allocates render resources on first call, caches the end-effector transform,
    /// and binds the camera to the render texture. Idempotent across episodes.
    /// </summary>
    public void Initialize()
    {
        if (!resourcesAllocated)
        {
            endEffector = robotArmSetup.robotJoints[^1].transform;
            renderTexture = new RenderTexture(ImageWidth, ImageHeight, 24);
            screenshotBuffer = new Texture2D(ImageWidth, ImageHeight, TextureFormat.RGB24, false);
            resourcesAllocated = true;
        }
        recordingCamera.targetTexture = renderTexture;
    }

    /// <summary>Restores the camera's display output between episodes.</summary>
    public void ReleaseCameraTarget()
    {
        if (recordingCamera != null)
            recordingCamera.targetTexture = null;
    }

    public Observation CaptureState()
    {
        ArticulationBody[] joints = robotArmSetup.robotJoints;
        float[] anglesDeg = new float[JointCount];
        for (int i = 0; i < JointCount; i++)
            anglesDeg[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;

        return new Observation
        {
            endEffectorPosition = endEffector.position,
            endEffectorRotation = endEffector.rotation,
            suctionOn = suctionController.enableSuction,
            jointAnglesDeg = anglesDeg,
            blockAttracted = suctionController.isBlockAttached,
        };
    }

    public byte[] CaptureScreenshotPng()
    {
        RenderToBuffer();
        return screenshotBuffer.EncodeToPNG();
    }

    /// <summary>Returns raw uint8 RGB bytes (bottom-up rows, as Unity's ReadPixels provides).</summary>
    public byte[] CaptureScreenshotRgb()
    {
        RenderToBuffer();
        return screenshotBuffer.GetRawTextureData();
    }

    private void RenderToBuffer()
    {
        recordingCamera.Render();
        RenderTexture.active = renderTexture;
        screenshotBuffer.ReadPixels(
            new Rect(0, 0, renderTexture.width, renderTexture.height),
            0,
            0
        );
        screenshotBuffer.Apply();
        RenderTexture.active = null;
    }

    private void OnDestroy()
    {
        ReleaseCameraTarget();
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
        if (screenshotBuffer != null)
            Destroy(screenshotBuffer);
    }
}
