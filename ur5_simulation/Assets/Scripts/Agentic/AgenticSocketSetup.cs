using System;
using System.Linq;
using UnityEngine;
using static ConstantsUR5;

public class AgenticSocketSetup : AgenticSocketBase
{
    protected override void InitializeRecorder()
    {
        recorder.robotArmSetup = robotArmSetups[0];
        recorder.suctionController = suctionControllers[0];
        recorder.recordingCamera = Camera.main;
    }

    protected override string ProcessAICommand(string commandJson)
    {
        try
        {
            var command = JsonUtility.FromJson<AICommand>(commandJson);

            string commonResult = TryProcessCommonCommand(command.type);
            if (commonResult != null)
                return commonResult;

            switch (command.type)
            {
                case "execute_action":
                    return ExecuteActionJson(command);

                case "pause_recording":
                    if (recorder != null)
                        recorder.PauseRecording();
                    return JsonUtility.ToJson(
                        new ControlResponse
                        {
                            success = true,
                            message = "recording_paused",
                            timestamp = Time.time,
                        }
                    );

                case "resume_recording":
                    if (recorder != null)
                        recorder.ResumeRecording();
                    return JsonUtility.ToJson(
                        new ControlResponse
                        {
                            success = true,
                            message = "recording_resumed",
                            timestamp = Time.time,
                        }
                    );

                default:
                    return JsonUtility.ToJson(
                        new AIResponse
                        {
                            success = false,
                            message = "Unknown command type: " + command.type,
                        }
                    );
            }
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

    private string ExecuteActionJson(AICommand command)
    {
        try
        {
            bool success = false;
            string message = "";

            GameObject targetRobot = null;
            if (sceneSetup != null && sceneSetup.robots != null)
            {
                targetRobot = sceneSetup.robots.FirstOrDefault(r => r.name == command.robot_name);
            }

            if (targetRobot == null)
            {
                return JsonUtility.ToJson(
                    new AIResponse
                    {
                        success = false,
                        message = "Robot not found: " + command.robot_name,
                    }
                );
            }

            switch (command.action_type)
            {
                case "home_robot":
                    int homeIdx = System.Array.IndexOf(robots, targetRobot);
                    robotArmSetups[homeIdx].ResetArmPosition();
                    success = true;
                    message = "Robot homed successfully";
                    break;
                case "solve_ik":
                    success = ExecuteSolveIK(targetRobot, command.parameters);
                    message = success ? "IK command executed" : "IK command failed";
                    break;
                case "move_robot":
                    return ExecuteMoveRobot(targetRobot, command.parameters);

                case "activate_suction":
                    success = ExecuteActivateSuction(targetRobot);
                    message = success ? "Suction activated" : "Suction activation failed";
                    break;

                case "deactivate_suction":
                    success = ExecuteDeactivateSuction(targetRobot);
                    message = success ? "Suction deactivated" : "Suction deactivation failed";
                    break;

                default:
                    message = "Unknown action type: " + command.action_type;
                    break;
            }

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

    private bool ExecuteSolveIK(GameObject robot, ActionParameters parameters)
    {
        int i = System.Array.IndexOf(robots, robot);
        ur5Controllers[i].SetJointAngles(parameters.joint_angles);
        return true;
    }

    private string ExecuteMoveRobot(GameObject robot, ActionParameters parameters)
    {
        if (recorder != null)
            recorder.StartRecording();

        try
        {
            Vector3 targetPos = new Vector3(
                parameters.target_position[0],
                parameters.target_position[1] + SuctionIKOffsetY,
                parameters.target_position[2]
            );
            int i = System.Array.IndexOf(robots, robot);
            bool success = ur5Controllers[i].MoveToTarget(targetPos, Quaternion.identity);
            return JsonUtility.ToJson(
                new AIResponse
                {
                    success = success,
                    message = success ? "move_executed" : "ik_failed",
                }
            );
        }
        catch (Exception e)
        {
            Debug.LogError("ExecuteMoveRobot error: " + e.Message);
            return JsonUtility.ToJson(new AIResponse { success = false, message = e.Message });
        }
    }
}
