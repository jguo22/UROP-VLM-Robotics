using UnityEditor;

public class BuildScript
{
    public static void Build()
    {
        BuildPlayerOptions options = new BuildPlayerOptions
        {
            scenes = new[] { "Assets/Scenes/Single Arm.unity" },
            locationPathName = "Builds/UR5Sim.app",
            target = BuildTarget.StandaloneOSX,
        };
        BuildPipeline.BuildPlayer(options);
    }
}
