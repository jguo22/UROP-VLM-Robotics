using UnityEngine;

/// <summary>
/// Keyboard-driven IK control for UR5 using UR5Controller.MoveDelta().
/// WASDQE for translation, arrow keys + PgUp/PgDn for rotation, R to reset.
/// </summary>
[RequireComponent(typeof(UR5Controller))]
public class UR5KeyboardIKController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField]
    private float ikMoveSpeed = 1f; // meters per second

    [SerializeField]
    private float ikRotateSpeed = 30.0f; // degrees per second

    private UR5Controller ur5Controller;

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

        // Rotation (arrow keys + PgUp/PgDn)
        // Vector3 deltaRotation = Vector3.zero;
        // float rotateStep = ikRotateSpeed * Time.deltaTime;
        // if (Input.GetKey(KeyCode.UpArrow)) deltaRotation.x += rotateStep;
        // if (Input.GetKey(KeyCode.DownArrow)) deltaRotation.x -= rotateStep;
        // if (Input.GetKey(KeyCode.LeftArrow)) deltaRotation.y -= rotateStep;
        // if (Input.GetKey(KeyCode.RightArrow)) deltaRotation.y += rotateStep;
        // if (Input.GetKey(KeyCode.PageUp)) deltaRotation.z += rotateStep;
        // if (Input.GetKey(KeyCode.PageDown)) deltaRotation.z -= rotateStep;
        // if (deltaPosition != Vector3.zero || deltaRotation != Vector3.zero)
        // {
        //     ur5Controller.MoveDelta(deltaPosition, Quaternion.Euler(deltaRotation));
        // }

        Quaternion deltaRotation =
            Quaternion.Euler(180, 0, 0)
            * Quaternion.Inverse(ur5Controller.GetEndEffectorPose().rotation);
        if (deltaPosition != Vector3.zero)
        {
            ur5Controller.MoveDelta(deltaPosition, deltaRotation);
        }
    }
}
