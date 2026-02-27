using UnityEngine;

namespace Unity.Robotics.UrdfImporter.Control
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

    public class JointControl : MonoBehaviour
    {
        public ArticulationBody joint;
        public RotationDirection direction = RotationDirection.None;
        public ControlType controltype;

        private float speed = 5f; // degrees per second
        private float torque = 100f; // Nm

        void Start()
        {
            joint = GetComponent<ArticulationBody>();
            if (joint == null)
            {
                Debug.LogError("JointControl requires an ArticulationBody component!");
                enabled = false;
                return;
            }
        }

        void FixedUpdate()
        {
            if (joint == null || joint.jointType == ArticulationJointType.FixedJoint)
                return;

            if (direction != RotationDirection.None)
            {
                float velocity = (float)direction * speed * Mathf.Deg2Rad;

                // Apply velocity control
                ArticulationDrive drive = joint.xDrive;
                drive.targetVelocity = velocity;
                joint.xDrive = drive;

                // Apply torque limits
                joint.maxAngularVelocity = Mathf.Abs(velocity);
                joint.maxDepenetrationVelocity = 0.1f;
            }
            else
            {
                // Stop movement
                ArticulationDrive drive = joint.xDrive;
                drive.targetVelocity = 0f;
                joint.xDrive = drive;
            }
        }

        public void SetDirection(RotationDirection newDirection)
        {
            direction = newDirection;
        }

        public void SetSpeed(float newSpeed)
        {
            speed = newSpeed;
        }

        public void SetTorque(float newTorque)
        {
            torque = newTorque;
        }
    }
}
