using System.IO;
using UnityEngine;

public static class ProjectPaths
{
    private static string _root;

    public static string Root => _root ??= FindRoot();

    public static string Get(string relativePath) => Path.Combine(Root, relativePath);

    private static string FindRoot()
    {
        string dir = Application.dataPath;
        for (int i = 0; i < 8; i++)
        {
            if (Path.GetFileName(dir) == "ur5_simulation")
                return Directory.GetParent(dir).FullName;
            dir = Directory.GetParent(dir).FullName;
        }
        Debug.LogError("ProjectPaths: ur5_simulation folder not found");
        return Application.dataPath;
    }
}
