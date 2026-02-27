using UnityEngine;
using System.Collections;
using System.Collections.Generic;

/// <summary>
/// Programmatic Robot Control Examples
/// Demonstrates various motion patterns and automation
/// </summary>
public class ProgrammaticRobotControl : MonoBehaviour
{
    public UnifiedRobotController robotController;
    public bool autoStart = false;

    [Header("Motion Patterns")]
    public bool enableCircularMotion = false;
    public bool enableSquarePattern = false;
    public bool enablePickAndPlaceDemo = false;
    public bool enableJointOscillation = false;

    [Header("Pattern Settings")]
    public float patternRadius = 0.3f;
    public float patternHeight = 0.5f;
    public float patternSpeed = 1f;
    public int patternPoints = 8;

    private Coroutine currentPattern;
    private Vector3 startPosition;

    void Start()
    {
        if (robotController == null)
            robotController = FindObjectOfType<UnifiedRobotController>();

        if (robotController != null && autoStart)
        {
            StartCoroutine(RunDemoSequence());
        }

        // Safely get start position
        if (robotController != null)
        {
            startPosition = robotController.GetEndEffectorPosition();
        }
        else
        {
            startPosition = Vector3.zero;
            Debug.LogWarning("UnifiedRobotController not found. Some features may not work.");
        }
    }

    void Update()
    {
        HandlePatternControls();

        // Demo controls
        if (Input.GetKeyDown(KeyCode.C) && !enableCircularMotion)
            StartCircularMotion();
        if (Input.GetKeyDown(KeyCode.V) && !enableSquarePattern)
            StartSquarePattern();
        if (Input.GetKeyDown(KeyCode.B) && !enablePickAndPlaceDemo)
            StartPickAndPlaceDemo();
        if (Input.GetKeyDown(KeyCode.N) && !enableJointOscillation)
            StartJointOscillation();
        if (Input.GetKeyDown(KeyCode.X))
            StopAllPatterns();
    }

    void HandlePatternControls()
    {
        if (enableCircularMotion && currentPattern == null)
            currentPattern = StartCoroutine(CircularMotionPattern());

        if (enableSquarePattern && currentPattern == null)
            currentPattern = StartCoroutine(SquareMotionPattern());

        if (enablePickAndPlaceDemo && currentPattern == null)
            currentPattern = StartCoroutine(PickAndPlaceDemoPattern());

        if (enableJointOscillation && currentPattern == null)
            currentPattern = StartCoroutine(JointOscillationPattern());
    }

    #region Motion Patterns

    IEnumerator CircularMotionPattern()
    {
        Debug.Log("Starting circular motion pattern");

        while (enableCircularMotion)
        {
            for (int i = 0; i < patternPoints; i++)
            {
                float angle = (i * 360f / patternPoints) * Mathf.Deg2Rad;
                Vector3 offset = new Vector3(
                    Mathf.Cos(angle) * patternRadius,
                    0f,
                    Mathf.Sin(angle) * patternRadius
                );

                Vector3 targetPos = startPosition + offset;
                targetPos.y = patternHeight;

                robotController.MoveEndEffectorTo(targetPos);

                yield return new WaitForSeconds(1f / patternSpeed);
            }
        }

        currentPattern = null;
    }

    IEnumerator SquareMotionPattern()
    {
        Debug.Log("Starting square motion pattern");

        Vector3[] squarePoints = new Vector3[]
        {
            new Vector3(patternRadius, patternHeight, patternRadius),
            new Vector3(patternRadius, patternHeight, -patternRadius),
            new Vector3(-patternRadius, patternHeight, -patternRadius),
            new Vector3(-patternRadius, patternHeight, patternRadius)
        };

        while (enableSquarePattern)
        {
            foreach (Vector3 offset in squarePoints)
            {
                Vector3 targetPos = startPosition + offset;
                robotController.MoveEndEffectorTo(targetPos);

                yield return new WaitForSeconds(2f / patternSpeed);
            }
        }

        currentPattern = null;
    }

    IEnumerator PickAndPlaceDemoPattern()
    {
        Debug.Log("Starting pick and place demo");

        // Define pick and place positions
        Vector3 pickPos = new Vector3(0.5f, 0.1f, 0f);
        Vector3 placePos = new Vector3(-0.5f, 0.1f, 0f);
        Vector3 abovePickPos = pickPos + Vector3.up * 0.2f;
        Vector3 abovePlacePos = placePos + Vector3.up * 0.2f;

        while (enablePickAndPlaceDemo)
        {
            // Move above pick position
            robotController.MoveEndEffectorTo(abovePickPos);
            yield return new WaitForSeconds(2f);

            // Move down to pick
            robotController.MoveEndEffectorTo(pickPos);
            yield return new WaitForSeconds(1f);

            // Simulate picking (in real scenario, activate suction)
            Debug.Log("Picking object...");
            yield return new WaitForSeconds(1f);

            // Move up
            robotController.MoveEndEffectorTo(abovePickPos);
            yield return new WaitForSeconds(2f);

            // Move above place position
            robotController.MoveEndEffectorTo(abovePlacePos);
            yield return new WaitForSeconds(2f);

            // Move down to place
            robotController.MoveEndEffectorTo(placePos);
            yield return new WaitForSeconds(1f);

            // Simulate placing
            Debug.Log("Placing object...");
            yield return new WaitForSeconds(1f);

            // Move up
            robotController.MoveEndEffectorTo(abovePlacePos);
            yield return new WaitForSeconds(2f);
        }

        currentPattern = null;
    }

    IEnumerator JointOscillationPattern()
    {
        Debug.Log("Starting joint oscillation pattern");

        float[] baseAngles = robotController.GetCurrentJointAngles();
        float amplitude = 30f; // degrees
        float frequency = 0.5f; // Hz

        while (enableJointOscillation)
        {
            float time = Time.time;
            float[] oscillationAngles = new float[baseAngles.Length];

            for (int i = 0; i < baseAngles.Length; i++)
            {
                // Different frequencies for different joints
                float jointFreq = frequency * (i + 1);
                oscillationAngles[i] = baseAngles[i] + amplitude * Mathf.Sin(2f * Mathf.PI * jointFreq * time);
            }

            robotController.SetJointAngles(oscillationAngles);

            yield return null;
        }

        // Reset to base angles
        robotController.SetJointAngles(baseAngles);
        currentPattern = null;
    }

    #endregion

    #region Control Methods

    public void StartCircularMotion()
    {
        StopAllPatterns();
        enableCircularMotion = true;
        Debug.Log("Circular motion enabled");
    }

    public void StartSquarePattern()
    {
        StopAllPatterns();
        enableSquarePattern = true;
        Debug.Log("Square pattern enabled");
    }

    public void StartPickAndPlaceDemo()
    {
        StopAllPatterns();
        enablePickAndPlaceDemo = true;
        Debug.Log("Pick and place demo enabled");
    }

    public void StartJointOscillation()
    {
        StopAllPatterns();
        enableJointOscillation = true;
        Debug.Log("Joint oscillation enabled");
    }

    public void StopAllPatterns()
    {
        enableCircularMotion = false;
        enableSquarePattern = false;
        enablePickAndPlaceDemo = false;
        enableJointOscillation = false;

        if (currentPattern != null)
        {
            StopCoroutine(currentPattern);
            currentPattern = null;
        }

        Debug.Log("All patterns stopped");
    }

    #endregion

    #region Advanced Control Methods

    /// <summary>
    /// Move robot along a custom path defined by waypoints
    /// </summary>
    public void FollowPath(Vector3[] waypoints, float speed = 1f)
    {
        StartCoroutine(FollowPathCoroutine(waypoints, speed));
    }

    private IEnumerator FollowPathCoroutine(Vector3[] waypoints, float speed)
    {
        foreach (Vector3 waypoint in waypoints)
        {
            robotController.MoveEndEffectorTo(waypoint);
            yield return new WaitForSeconds(speed);
        }
    }

    /// <summary>
    /// Execute a smooth joint space trajectory
    /// </summary>
    public void ExecuteJointTrajectory(float[][] trajectory, float timeStep = 0.1f)
    {
        StartCoroutine(ExecuteJointTrajectoryCoroutine(trajectory, timeStep));
    }

    private IEnumerator ExecuteJointTrajectoryCoroutine(float[][] trajectory, float timeStep)
    {
        foreach (float[] jointAngles in trajectory)
        {
            robotController.SetJointAngles(jointAngles);
            yield return new WaitForSeconds(timeStep);
        }
    }

    /// <summary>
    /// Create a circular path for the end effector
    /// </summary>
    public Vector3[] CreateCircularPath(Vector3 center, float radius, int points = 16)
    {
        Vector3[] path = new Vector3[points];
        for (int i = 0; i < points; i++)
        {
            float angle = (i * 360f / points) * Mathf.Deg2Rad;
            path[i] = center + new Vector3(
                Mathf.Cos(angle) * radius,
                0f,
                Mathf.Sin(angle) * radius
            );
        }
        return path;
    }

    /// <summary>
    /// Create a square path for the end effector
    /// </summary>
    public Vector3[] CreateSquarePath(Vector3 center, float sideLength)
    {
        float halfSide = sideLength / 2f;
        return new Vector3[]
        {
            center + new Vector3(-halfSide, 0, -halfSide),
            center + new Vector3(halfSide, 0, -halfSide),
            center + new Vector3(halfSide, 0, halfSide),
            center + new Vector3(-halfSide, 0, halfSide)
        };
    }

    #endregion

    #region Demo Sequence

    IEnumerator RunDemoSequence()
    {
        Debug.Log("Starting automated demo sequence");

        yield return new WaitForSeconds(2f);

        // Demo 1: Joint oscillation
        Debug.Log("Demo 1: Joint oscillation");
        StartJointOscillation();
        yield return new WaitForSeconds(5f);

        // Demo 2: Circular motion
        Debug.Log("Demo 2: Circular motion");
        StartCircularMotion();
        yield return new WaitForSeconds(8f);

        // Demo 3: Square pattern
        Debug.Log("Demo 3: Square pattern");
        StartSquarePattern();
        yield return new WaitForSeconds(8f);

        // Demo 4: Pick and place
        Debug.Log("Demo 4: Pick and place demo");
        StartPickAndPlaceDemo();
        yield return new WaitForSeconds(10f);

        StopAllPatterns();
        robotController.ResetToHomePosition();

        Debug.Log("Demo sequence completed");
    }

    #endregion

    #region GUI

    void OnGUI()
    {
        GUIStyle style = new GUIStyle(GUI.skin.button);
        style.fontSize = 12;

        int buttonWidth = 120;
        int buttonHeight = 30;
        int spacing = 5;
        int startX = Screen.width - buttonWidth - 10;
        int startY = 10;

        if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "Circular"))
            StartCircularMotion();

        if (GUI.Button(new Rect(startX, startY + (buttonHeight + spacing), buttonWidth, buttonHeight), "Square"))
            StartSquarePattern();

        if (GUI.Button(new Rect(startX, startY + 2 * (buttonHeight + spacing), buttonWidth, buttonHeight), "Pick&Place"))
            StartPickAndPlaceDemo();

        if (GUI.Button(new Rect(startX, startY + 3 * (buttonHeight + spacing), buttonWidth, buttonHeight), "Oscillation"))
            StartJointOscillation();

        if (GUI.Button(new Rect(startX, startY + 4 * (buttonHeight + spacing), buttonWidth, buttonHeight), "Stop All"))
            StopAllPatterns();

        if (GUI.Button(new Rect(startX, startY + 5 * (buttonHeight + spacing), buttonWidth, buttonHeight), "Reset"))
            robotController.ResetToHomePosition();
    }

    #endregion
}
