using UnityEngine;

/// <summary>
/// Keyboard-driven IK control for UR5 using UR5Controller.MoveDelta().
/// WASDQE for translation, arrow keys + PgUp/PgDn for rotation, R to reset.
/// Also provides an OnGUI panel to move to a specific world position/rotation.
/// </summary>
[RequireComponent(typeof(UR5Controller))]
public class InputController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField]
    private float ikMoveSpeed = 1f; // meters per second

    private UR5Controller ur5Controller;

    // Internal pose state
    private Vector3 targetPosition = Vector3.zero;
    private Vector3 targetEuler = Vector3.zero;

    // UI editing buffers (kept in sync with internal state)
    private string posX = "0",
        posY = "0",
        posZ = "0";
    private string rotX = "0",
        rotY = "0",
        rotZ = "0";
    private string statusMessage = "";

    void Start()
    {
        ur5Controller = GetComponent<UR5Controller>();
    }

    void Update()
    {
        // Translation (WASDQE)
        Vector3 deltaPosition = Vector3.zero;
        float moveStep = ikMoveSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.W))
            deltaPosition.z += moveStep;
        if (Input.GetKey(KeyCode.S))
            deltaPosition.z -= moveStep;
        if (Input.GetKey(KeyCode.A))
            deltaPosition.x -= moveStep;
        if (Input.GetKey(KeyCode.D))
            deltaPosition.x += moveStep;
        if (Input.GetKey(KeyCode.Q))
            deltaPosition.y -= moveStep;
        if (Input.GetKey(KeyCode.E))
            deltaPosition.y += moveStep;

        Quaternion deltaRotation =
            Quaternion.Euler(180, 0, 0)
            * Quaternion.Inverse(ur5Controller.GetEndEffectorPoseLocal().rotation);
        if (deltaPosition != Vector3.zero)
        {
            ur5Controller.MoveDelta(deltaPosition, deltaRotation);
        }
    }

    void OnGUI()
    {
        float panelWidth = 320f;
        float panelHeight = 210f;
        float x = 10f;
        float y = Screen.height - panelHeight - 10f;

        Rect panelRect = new Rect(x, y, panelWidth, panelHeight);

        // Background
        Texture2D bg = new Texture2D(1, 1);
        bg.SetPixel(0, 0, new Color(0, 0, 0, 0.75f));
        bg.Apply();
        GUI.DrawTexture(panelRect, bg);

        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.normal.textColor = Color.white;
        labelStyle.fontSize = 13;

        GUIStyle fieldStyle = new GUIStyle(GUI.skin.textField);
        fieldStyle.fontSize = 13;

        GUIStyle statusStyle = new GUIStyle(GUI.skin.label);
        statusStyle.fontSize = 12;
        statusStyle.normal.textColor = statusMessage.StartsWith("OK") ? Color.green : Color.red;

        float pad = 10f;
        float labelW = 60f;
        float fieldW = 70f;
        float rowH = 22f;
        float col1 = x + pad;
        float col2 = col1 + labelW;
        float col3 = col2 + fieldW + 4f;
        float col4 = col3 + labelW;
        float col5 = col4 + fieldW + 4f;
        float col6 = col5 + labelW;

        float row = y + pad;

        // Header
        GUI.Label(
            new Rect(col1, row, panelWidth - pad * 2, rowH),
            "Move to World Pose",
            labelStyle
        );
        row += rowH + 4f;

        // Position row
        GUI.Label(new Rect(col1, row, labelW, rowH), "Pos X", labelStyle);
        string newPosX = GUI.TextField(new Rect(col2, row, fieldW, rowH), posX, fieldStyle);
        GUI.Label(new Rect(col3, row, labelW, rowH), "Y", labelStyle);
        string newPosY = GUI.TextField(new Rect(col4, row, fieldW, rowH), posY, fieldStyle);
        GUI.Label(new Rect(col5, row, labelW, rowH), "Z", labelStyle);
        string newPosZ = GUI.TextField(new Rect(col6, row, fieldW, rowH), posZ, fieldStyle);
        row += rowH + 6f;

        if (newPosX != posX)
        {
            posX = newPosX;
            if (float.TryParse(posX, out float px))
                targetPosition.x = px;
        }
        if (newPosY != posY)
        {
            posY = newPosY;
            if (float.TryParse(posY, out float py))
                targetPosition.y = py;
        }
        if (newPosZ != posZ)
        {
            posZ = newPosZ;
            if (float.TryParse(posZ, out float pz))
                targetPosition.z = pz;
        }

        // Rotation row
        GUI.Label(new Rect(col1, row, labelW, rowH), "Rot X", labelStyle);
        string newRotX = GUI.TextField(new Rect(col2, row, fieldW, rowH), rotX, fieldStyle);
        GUI.Label(new Rect(col3, row, labelW, rowH), "Y", labelStyle);
        string newRotY = GUI.TextField(new Rect(col4, row, fieldW, rowH), rotY, fieldStyle);
        GUI.Label(new Rect(col5, row, labelW, rowH), "Z", labelStyle);
        string newRotZ = GUI.TextField(new Rect(col6, row, fieldW, rowH), rotZ, fieldStyle);
        row += rowH + 6f;

        if (newRotX != rotX)
        {
            rotX = newRotX;
            if (float.TryParse(rotX, out float rx))
                targetEuler.x = rx;
        }
        if (newRotY != rotY)
        {
            rotY = newRotY;
            if (float.TryParse(rotY, out float ry))
                targetEuler.y = ry;
        }
        if (newRotZ != rotZ)
        {
            rotZ = newRotZ;
            if (float.TryParse(rotZ, out float rz))
                targetEuler.z = rz;
        }

        // Fill-from-current button
        if (GUI.Button(new Rect(col1, row, 140f, rowH), "Fill Current Pose"))
        {
            (Vector3 pos, Quaternion rot) = ur5Controller.GetEndEffectorPoseWorld();
            targetPosition = pos;
            targetEuler = rot.eulerAngles;
            posX = targetPosition.x.ToString("F4");
            posY = targetPosition.y.ToString("F4");
            posZ = targetPosition.z.ToString("F4");
            rotX = targetEuler.x.ToString("F2");
            rotY = targetEuler.y.ToString("F2");
            rotZ = targetEuler.z.ToString("F2");
            statusMessage = "";
        }
        row += rowH + 6f;

        // Move button
        if (GUI.Button(new Rect(col1, row, 80f, rowH), "Move"))
        {
            bool success = ur5Controller.MoveToTarget(
                targetPosition,
                Quaternion.Euler(targetEuler)
            );
            statusMessage = success ? "OK - IK solved" : "ERR - no IK solution";
        }

        // Status
        if (!string.IsNullOrEmpty(statusMessage))
        {
            GUI.Label(new Rect(col1 + 88f, row, 200f, rowH), statusMessage, statusStyle);
        }
    }
}
