using System;
using UnityEngine;

using System.Text.RegularExpressions;

using static ConstantsUR5;

namespace Unity.Robotics.UrdfImporter.Control
{
    //public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };

    [DefaultExecutionOrder(-1)]  // Ensures this runs after RobotArmSetup
    [RequireComponent(typeof(SuctionController))]
    public class ManualController : MonoBehaviour
    {
        // Stores original colors of the part being highlighted
        private Color[] prevColor;
        private int previousIndex;

        [InspectorReadOnly(hideInEditMode: true)]
        public string selectedJoint;
        [HideInInspector]
        public int selectedIndex;

        private ArticulationBody[] articulationChain;
        private ArticulationBody[] robotJoints;

        private SuctionController suctionController;

        public ControlType control = ControlType.PositionControl;

        public float speed = JointSpeed; // Units: degree/s
        public float torque = 100f; // Units: Nm or N
        public float acceleration = JointAcceleration;// Units: m/s^2 / degree/s^2

        private float originalAngleTolerance = 0.01f;
        private float angleTolerance = 0.01f;
        private float slowAngleTolerance = 15f;
        private float superSlowAngleTolerance = 0.5f;
        private int angleConvergenceMaxCount = 5;
        private int totalConvergenceMaxCount = 15;
        private int angleConvergenceCount = 0;
        private int totalConvergenceCount = 0;

        private int maxIterations = 1000;
        private int iterations = 0;
        private float previousIterateJointAngle = 1000f;
        private float previousDifference = 1000f;
        private float originalDifference = 1000f;
        private bool obtainedOriginalDifference = false;

        [Tooltip("Color to highlight the currently selected join")]
        public Color highLightColor = new Color(1.0f, 0, 0, 1.0f);

        private bool isWaitingForAngleInput = false;
        private bool isChangingAngle = false;
        private string targetAngleInputString = "";
        private float targetAngleInputValue = 0f;

        void Start()
        {
            previousIndex = selectedIndex = 0;
            this.gameObject.AddComponent<FKRobot>();
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();

            foreach (ArticulationBody joint in articulationChain)
            {
                if (JointNames.ContainsKey(joint.name))
                {
                    joint.gameObject.AddComponent<ManualJointControl>();
                    joint.jointFriction = JointFriction;
                    joint.angularDamping = AngularDamping;
                    ArticulationDrive currentDrive = joint.xDrive;
                    currentDrive.forceLimit = ForceLimit;
                    joint.xDrive = currentDrive;
                }
            }

            robotJoints = new ArticulationBody[JointCount + 1];
            Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);

            suctionController = GetComponent<SuctionController>();

            DisplaySelectedJoint(selectedIndex);
            StoreJointColors(selectedIndex);
        }

        void SetSelectedJointIndex(int index)
        {
            if (robotJoints.Length > 0)
            {
                selectedIndex = (index + JointCount) % JointCount;
            }
        }

        void Update()
        {
            if (!isChangingAngle)
            {
                bool SelectionInput1 = Input.GetKeyDown("right");
                bool SelectionInput2 = Input.GetKeyDown("left");
                bool targetAngleInput = Input.GetKeyDown(KeyCode.T);

                SetSelectedJointIndex(selectedIndex); // to make sure it is in the valid range
                UpdateDirection(selectedIndex);

                if (SelectionInput2)
                {
                    SetSelectedJointIndex(selectedIndex - 1);
                    Highlight(selectedIndex);
                }
                else if (SelectionInput1)
                {
                    SetSelectedJointIndex(selectedIndex + 1);
                    Highlight(selectedIndex);
                }
                else if (targetAngleInput)
                {
                    PromptForTargetAngle();
                }

                if (isWaitingForAngleInput) HandleAngleInput();

            }

            if (isChangingAngle) UpdateDirectionWithInput(robotJoints, targetAngleInputValue, selectedIndex);
            else UpdateDirection(selectedIndex);
        }

        public float[] GetCurrentJointAngles(ArticulationBody[] robotJoints)
        {
            if (robotJoints == null) return null;

            float[] angles = new float[JointCount]; // UR5 has 6 joints
            for (int i = 0; i < angles.Length && i < robotJoints.Length; i++)
            {
                angles[i] = robotJoints[i].jointPosition[0] * Mathf.Rad2Deg;
            }
            return angles;
        }

        private void HandleAngleInput()
        {
            if (!isWaitingForAngleInput) return;

            //Handle character input
            if (Input.inputString.Length > 0)
            {
                //Debug.Log("Input.inputString: " + Input.inputString);
                foreach (char c in Input.inputString)
                {
                    if (c == '\b') // Backspace
                    {
                        if (targetAngleInputString.Length > 0)
                        {
                            targetAngleInputString = targetAngleInputString.Substring(0, targetAngleInputString.Length);
                        }
                    }
                    else if (c == '\n' || c == '\r') // Enter
                    {
                        ProcessAngleInput();
                        return;
                    }
                    else if (c == 'e' || c == 'E')
                    {
                        isWaitingForAngleInput = false;
                        //return;
                    }
                }
            }
        }

        void ProcessAngleInput()
        {
            if (float.TryParse(targetAngleInputString, out targetAngleInputValue))
            {
                Debug.Log($"Setting target angle for {selectedJoint} to {targetAngleInputValue}°");

                UpdateDirectionWithInput(robotJoints, targetAngleInputValue, selectedIndex);
                isWaitingForAngleInput = false; // Reset the waiting state after processing
                isChangingAngle = true;
            }
            else
            {
                Debug.Log("Invalid target angle input: '" + targetAngleInputString + "'");
                isWaitingForAngleInput = false; // Reset even on error to prevent getting stuck
            }
        }

        void PromptForTargetAngle()
        {
            isWaitingForAngleInput = true;
            targetAngleInputString = "";
            targetAngleInputValue = 0f;

            Debug.Log("Enter target angle for " + selectedJoint + " (use GUI input field)");
        }

        // void SetSelectedJointTargetAngle(float targetAngle)
        // {
        //     robotJoints[selectedIndex].xDrive.target = targetAngle;
        //     Debug.Log("Set target angle for " + selectedJoint + " to " + targetAngle + "°");
        // }

        private void ResetConvergenceParameters()
        {
            isChangingAngle = false;
            angleConvergenceCount = 0;
            totalConvergenceCount = 0;
            originalAngleTolerance = 0.01f;
            angleTolerance = 0.01f;
            iterations = 0;
            previousIterateJointAngle = 1000f;
            previousDifference = 1000f;
            originalDifference = 1000f;
            obtainedOriginalDifference = false;
            maxIterations = 1000;
        }

        public void UpdateDirectionWithInput(ArticulationBody[] robotJoints, float targetAngle, int jointIndex)
        {
            if (jointIndex < 0 || jointIndex >= JointCount)
            {
                return;
            }

            float currentAngle = GetCurrentJointAngles(robotJoints)[jointIndex];

            if (targetAngle < SmallestJointAngle) targetAngle = SmallestJointAngle;
            else if (targetAngle > LargestJointAngle) targetAngle = LargestJointAngle;
            //float difference = targetAngle - currentAngle;

            ArticulationDrive drive = robotJoints[jointIndex].xDrive;
            drive.target = targetAngle;

            drive.stiffness = 10000f;
            drive.damping = 100f;
            drive.forceLimit = 1000f;
            //drive.targetVelocity = 15f;

            robotJoints[jointIndex].xDrive = drive;
            ResetConvergenceParameters();

            // if (!obtainedOriginalDifference)
            // {
            //     originalDifference = difference;
            //     obtainedOriginalDifference = true;
            // }
            // //maxIterations = (int)(Mathf.Abs(originalDifference) * 20f) + 100; //dynamic max iterations based on difference
            // maxIterations = (int)(Mathf.Abs(originalDifference) / 0.2f) + 200;

            // switch (jointIndex)
            // {
            //     case 0: //base
            //         originalAngleTolerance = 0.01f;
            //         break;
            //     case 1: //shoulder
            //         originalAngleTolerance = 0.05f;
            //         break;
            //     case 2: //elbow
            //         originalAngleTolerance = 0.1f;
            //         break;
            //     default: //higher tolerance for smaller links
            //         originalAngleTolerance = 0.1f;
            //         break;
            // }
            // ;

            // if (originalAngleTolerance >= angleTolerance) angleTolerance = originalAngleTolerance;

            // ManualJointControl current = robotJoints[jointIndex].GetComponent<ManualJointControl>();
            // if (current == null)
            // {
            //     current = robotJoints[jointIndex].gameObject.AddComponent<ManualJointControl>();
            //     //Debug.Log("Current not found");
            // }
            // if (previousIndex != jointIndex)
            // {
            //     ManualJointControl previous = robotJoints[previousIndex].GetComponent<ManualJointControl>();
            //     if (previous == null)
            //     {
            //         previous = robotJoints[previousIndex].gameObject.AddComponent<ManualJointControl>();
            //         Debug.Log("Previous not found");
            //     }
            //     previous.direction = RotationDirection.None;
            //     previousIndex = jointIndex;
            // }

            // if (current.controltype != control) UpdateControlType(current);

            // //Debug.Log(targetAngle + " " + currentAngle + " " + difference);

            // if (Mathf.Abs(difference) <= superSlowAngleTolerance) {
            //     current.speed = JointSuperSlowSpeed;
            //     //current.acceleration = JointSuperSlowAcceleration;
            // }  else if (Mathf.Abs(difference) <= slowAngleTolerance) {
            //     current.speed = JointSlowSpeed;
            //     //current.acceleration = JointSlowAcceleration;
            // } else {
            //     current.speed = JointSpeed;
            //     //current.acceleration = JointAcceleration;
            // }

            // if (difference >= angleTolerance) {
            //     current.direction = RotationDirection.Positive;
            //     angleConvergenceCount = 0;
            // } else if (difference <= -angleTolerance) {
            //     current.direction = RotationDirection.Negative;
            //     angleConvergenceCount = 0;
            // } else {
            //     current.direction = RotationDirection.None;
            //     angleConvergenceCount++;
            //     totalConvergenceCount++;
            //     if (angleConvergenceCount >= angleConvergenceMaxCount)
            //     {
            //         //current.direction = RotationDirection.None;
            //         ResetConvergenceParameters();
            //         Debug.Log("Angle converged to target, stopping movement");
            //     }
            //     else if (totalConvergenceCount >= totalConvergenceMaxCount && (angleTolerance < 0.1f || jointIndex >= 3))
            //     {
            //         //isChangingAngle = false;
            //         totalConvergenceCount = 0;
            //         angleTolerance += 0.01f;
            //         Debug.Log("Adjusted angle tolerance to " + angleTolerance);
            //     }
            // }

            // if (Mathf.Abs(currentAngle - previousIterateJointAngle) < angleTolerance) {
            //     iterations++;
            //     if (iterations >= maxIterations) {
            //         Debug.Log("Max iterations reached, stopping movement");
            //         current.direction = RotationDirection.None;
            //         ResetConvergenceParameters();
            //     }
            // }

            // if (Mathf.Abs(difference) < slowAngleTolerance) {
            //     iterations++;
            //     if (iterations >= maxIterations) {
            //         //isChangingAngle = false;
            //         iterations = 0;
            //         angleTolerance = 0.1f;
            //         Debug.Log("Max iterations reached, adjusted angle tolerance to " + angleTolerance);
            //     }
            // } else if (Mathf.Abs(currentAngle - previousIterateJointAngle) < angleTolerance) {
            //     iterations++;
            //     if (iterations >= maxIterations) {
            //         current.direction = RotationDirection.None;
            //         ResetConvergenceParameters();
            //     }
            // }
            // else if (Mathf.Abs(difference - previousDifference) < angleTolerance) {
            //     iterations++;
            //     Debug.Log("Iterations (based on difference): " + iterations);
            //     if (iterations >= maxIterations) {
            //         current.direction = RotationDirection.None;
            //         ResetConvergenceParameters();
            //     }
            // }

            //previousIterateJointAngle = currentAngle;
            //previousDifference = difference;

        }

        /// <summary>
        /// Highlights the color of the robot by changing the color of the part to a color set by the user in the inspector window
        /// </summary>
        /// <param name="selectedIndex">Index of the link selected in the Articulation Chain</param>
        private void Highlight(int selectedIndex)
        {
            if (selectedIndex == previousIndex || selectedIndex < 0 || selectedIndex >= JointCount)
            {
                return;
            }

            // reset colors for the previously selected joint
            ResetJointColors(previousIndex);

            // store colors for the current selected joint
            StoreJointColors(selectedIndex);

            DisplaySelectedJoint(selectedIndex);
            Renderer[] rendererList = robotJoints[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

            // set the color of the selected join meshes to the highlight color
            foreach (var mesh in rendererList)
            {
                MaterialExtensions.SetMaterialColor(mesh.material, highLightColor);
            }
        }

        void DisplaySelectedJoint(int selectedIndex)
        {
            if (selectedIndex < 0 || selectedIndex > JointCount)
            {
                return;
            }
            string originalJointName = robotJoints[selectedIndex].name;
            selectedJoint = (JointNames.ContainsKey(originalJointName) ? JointNames[originalJointName] : originalJointName) + " (" + selectedIndex + ")";
        }

        /// <summary>
        /// Sets the direction of movement of the joint on every update
        /// </summary>
        /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
        private void UpdateDirection(int jointIndex)
        {
            if (jointIndex < 0 || jointIndex >= JointCount)
            {
                return;
            }
            float moveDirection = Input.GetAxis("Vertical"); //up > 0, down < 0
            ManualJointControl current = robotJoints[jointIndex].GetComponent<ManualJointControl>();
            if (current == null)
            {
                current = robotJoints[jointIndex].gameObject.AddComponent<ManualJointControl>();
                //Debug.Log("Current not found");
            }
            if (previousIndex != jointIndex)
            {
                ManualJointControl previous = robotJoints[previousIndex].GetComponent<ManualJointControl>();
                if (previous == null)
                {
                    previous = robotJoints[previousIndex].gameObject.AddComponent<ManualJointControl>();
                    Debug.Log("Previous not found");
                }
                previous.direction = RotationDirection.None;
                previousIndex = jointIndex;
            }

            if (current.controltype != control) UpdateControlType(current);

            if (moveDirection > 0) current.direction = RotationDirection.Positive;
            else if (moveDirection < 0) current.direction = RotationDirection.Negative;
            else current.direction = RotationDirection.None;
        }

        /// <summary>
        /// Stores original color of the part being highlighted
        /// </summary>
        /// <param name="index">Index of the part in the Articulation chain</param>
        private void StoreJointColors(int index)
        {
            Renderer[] materialLists = robotJoints[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
            prevColor = new Color[materialLists.Length];
            for (int counter = 0; counter < materialLists.Length; counter++)
            {
                prevColor[counter] = MaterialExtensions.GetMaterialColor(materialLists[counter]);
            }
        }

        /// <summary>
        /// Resets original color of the part being highlighted
        /// </summary>
        /// <param name="index">Index of the part in the Articulation chain</param>
        private void ResetJointColors(int index)
        {
            Renderer[] previousRendererList = robotJoints[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
            for (int counter = 0; counter < previousRendererList.Length; counter++)
            {
                MaterialExtensions.SetMaterialColor(previousRendererList[counter].material, prevColor[counter]);
            }
        }

        public void UpdateControlType(ManualJointControl joint)
        {
            joint.controltype = control;
            if (control == ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.joint.xDrive;
                drive.stiffness = JointStiffness;
                drive.damping = JointDamping;
                joint.joint.xDrive = drive;
            }
        }

        public void OnGUI()
        {
            GUIStyle centeredStyle = GUI.skin.GetStyle("Label");
            centeredStyle.alignment = TextAnchor.UpperCenter;
            GUI.Label(new Rect(Screen.width / 2 - 200, 10, 400, 20), "Press left/right arrow keys to select a robot joint.", centeredStyle);
            GUI.Label(new Rect(Screen.width / 2 - 200, 30, 400, 20), "Press up/down keys to move " + selectedJoint + ".", centeredStyle);

            // Display instructions for angle input
            GUI.Label(new Rect(Screen.width / 2 - 200, 50, 400, 20), "Press 'T' to set target angle for selected joint.", centeredStyle);

            if (isWaitingForAngleInput)
            {
                GUI.Label(new Rect(Screen.width / 2 - 200, 70, 400, 20), "Enter target angle (degrees):", centeredStyle);

                // Create input field
                GUI.SetNextControlName("AngleInputField");
                //GUI.TextField(new Rect(Screen.width / 2 - 100, 90, 200, 20), targetAngleInputString);
                targetAngleInputString = GUI.TextField(new Rect(Screen.width / 2 - 100, 90, 200, 20), targetAngleInputString);
                targetAngleInputString = Regex.Replace(targetAngleInputString, "[^0-9.-]", "");

                // Instructions
                GUI.Label(new Rect(Screen.width / 2 - 200, 110, 400, 20), "Press Enter to confirm, 'E' to exit", centeredStyle);

                // Show current input
                if (!string.IsNullOrEmpty(targetAngleInputString))
                {
                    GUI.Label(new Rect(Screen.width / 2 - 200, 130, 400, 20), $"Input: {targetAngleInputString}°", centeredStyle);
                }

                // Focus the input field
                GUI.FocusControl("AngleInputField");
            }
        }
    }
}
