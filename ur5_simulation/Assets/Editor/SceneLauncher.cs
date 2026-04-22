using UnityEditor;
using UnityEngine.SceneManagement;

public class SceneLauncher
{
    public static void Launch()
    {
        SceneManager.LoadScene("Assets/Scenes/Single Arm.unity");
        EditorApplication.EnterPlaymode();
    }
}
