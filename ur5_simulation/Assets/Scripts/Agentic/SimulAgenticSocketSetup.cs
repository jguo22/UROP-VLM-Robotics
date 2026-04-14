using System;
using UnityEngine;

public class SimulAgenticSocketSetup : AgenticSocketBase
{
    private bool toMoveArms = false;
    private bool toMoveLeftArm = false;
    private bool toMoveRightArm = false;

    [HideInInspector]
    public bool moveArmsNow = false;

    protected override void InitializeRecorder()
    {
        // Simul mode does not wire recorder to a specific arm
    }

    protected override string ProcessAICommand(string commandJson)
    {
        try
        {
            var command = JsonUtility.FromJson<SimulAICommand>(commandJson);

            string commonResult = TryProcessCommonCommand(command.type);
            if (commonResult != null)
                return commonResult;

            // Default: simultaneous AI command
            return ExecuteActionJson(command);
        }
        catch (Exception e)
        {
            Debug.LogError("Error processing AI command: " + e.Message);
            return JsonUtility.ToJson(
                new AIResponse
                {
                    success = false,
                    message = "Error processing command: " + e.Message,
                }
            );
        }
    }

    private string ExecuteActionJson(SimulAICommand command)
    {
        try
        {
            bool leftSuccess = true;
            bool rightSuccess = true;
            toMoveArms = false;
            toMoveLeftArm = false;
            toMoveRightArm = false;
            moveArmsNow = false;
            string message = "";

            GameObject ur5_left_robot;
            GameObject ur5_right_robot;
            if (robots[0].name == "ur5_left")
            {
                ur5_left_robot = robots[0];
                ur5_right_robot = robots[1];
            }
            else
            {
                ur5_left_robot = robots[1];
                ur5_right_robot = robots[0];
            }

            switch (command.ur5_left.action_type)
            {
                case "home_robot":
                    robotArmSetups[System.Array.IndexOf(robots, ur5_left_robot)].ResetArmPosition();
                    leftSuccess = true;
                    message = "Left Robot homed successfully";
                    break;
                case "move_robot":
                    toMoveLeftArm = true;
                    break;
                case "stationary":
                    break;

                case "activate_suction":
                    leftSuccess = ExecuteActivateSuction(ur5_left_robot);
                    message = leftSuccess
                        ? "Left Suction activated"
                        : "Left Suction activation failed";
                    break;

                case "deactivate_suction":
                    leftSuccess = ExecuteDeactivateSuction(ur5_left_robot);
                    message = leftSuccess
                        ? "Left Suction deactivated"
                        : "Left Suction deactivation failed";
                    break;

                default:
                    message = "Unknown action type: " + command.ur5_left.action_type;
                    break;
            }

            message += " | ";

            switch (command.ur5_right.action_type)
            {
                case "home_robot":
                    robotArmSetups[System.Array.IndexOf(robots, ur5_right_robot)]
                        .ResetArmPosition();
                    rightSuccess = true;
                    message += "Right Robot homed successfully";
                    break;
                case "move_robot":
                    toMoveRightArm = true;
                    break;
                case "stationary":
                    break;

                case "activate_suction":
                    rightSuccess = ExecuteActivateSuction(ur5_right_robot);
                    message += rightSuccess
                        ? "Right Suction activated"
                        : "Right Suction activation failed";
                    break;

                case "deactivate_suction":
                    rightSuccess = ExecuteDeactivateSuction(ur5_right_robot);
                    message += rightSuccess
                        ? "Right Suction deactivated"
                        : "Right Suction deactivation failed";
                    break;

                default:
                    message += "Unknown action type: " + command.ur5_right.action_type;
                    break;
            }

            bool success = leftSuccess && rightSuccess;
            toMoveArms = toMoveLeftArm || toMoveRightArm;

            if (toMoveArms)
            {
                Debug.Log("Time to move arms");
                GameObject[] robots;
                ActionParameters[] parameterArray;
                if (toMoveLeftArm && toMoveRightArm)
                {
                    robots = new GameObject[base.robots.Length];
                    robots[0] = ur5_left_robot;
                    robots[1] = ur5_right_robot;
                    parameterArray = new ActionParameters[base.robots.Length];
                    parameterArray[0] = command.ur5_left.parameters;
                    parameterArray[1] = command.ur5_right.parameters;
                }
                else if (toMoveLeftArm)
                {
                    robots = new GameObject[1];
                    robots[0] = ur5_left_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_left.parameters;
                }
                else
                {
                    robots = new GameObject[1];
                    robots[0] = ur5_right_robot;
                    parameterArray = new ActionParameters[1];
                    parameterArray[0] = command.ur5_right.parameters;
                }
                return ExecuteMoveRobot(robots, parameterArray);
            }

            toMoveArms = false;
            toMoveLeftArm = false;
            toMoveRightArm = false;
            moveArmsNow = false;
            return JsonUtility.ToJson(new AIResponse { success = success, message = message });
        }
        catch (Exception e)
        {
            Debug.LogError("Error executing action: " + e.Message);
            return JsonUtility.ToJson(
                new AIResponse { success = false, message = "Error executing action: " + e.Message }
            );
        }
    }

    private string ExecuteMoveRobot(GameObject[] robots, ActionParameters[] parameters)
    {
        bool allSuccess = true;
        try
        {
            for (int i = 0; i < robots.Length; i++)
            {
                if (parameters[i] == null || parameters[i].target_position == null)
                {
                    Debug.LogError(
                        $"ExecuteMoveRobot: null parameters or target_position for robot {robots[i]?.name}"
                    );
                    return JsonUtility.ToJson(
                        new AIResponse { success = false, message = "null target_position" }
                    );
                }
                int idx = System.Array.IndexOf(base.robots, robots[i]);
                if (idx < 0 || ur5Controllers[idx] == null)
                {
                    Debug.LogError(
                        $"ExecuteMoveRobot: UR5Controller not found for robot {robots[i]?.name} (idx={idx})"
                    );
                    return JsonUtility.ToJson(
                        new AIResponse { success = false, message = "UR5Controller not found" }
                    );
                }
                Vector3 targetPos = new Vector3(
                    parameters[i].target_position[0],
                    parameters[i].target_position[1],
                    parameters[i].target_position[2]
                );
                if (!ur5Controllers[idx].MoveToWorldPosition(targetPos))
                    allSuccess = false;
            }
        }
        catch (Exception e)
        {
            Debug.LogError("ExecuteMoveRobot error: " + e.Message);
            return JsonUtility.ToJson(new AIResponse { success = false, message = e.Message });
        }
        return JsonUtility.ToJson(
            new AIResponse
            {
                success = allSuccess,
                message = allSuccess ? "move_executed" : "move_failed",
            }
        );
    }
}
