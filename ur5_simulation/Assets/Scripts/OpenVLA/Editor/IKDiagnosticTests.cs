using NUnit.Framework;
using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Diagnostic tests to understand IK solver failures with real OpenVLA actions.
/// These tests help debug "No IK solution found" errors.
/// </summary>
public class IKDiagnosticTests
{
    private GameObject testObject;
    private UR5IKSolver solver;

    [SetUp]
    public void SetUp()
    {
        testObject = new GameObject("TestIKSolver");
        solver = testObject.AddComponent<UR5IKSolver>();
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(testObject);
    }

    #region Frame Transformation Verification

    [Test]
    public void DiagnosticFrameTransform_IdentityRotation_DeltaInWorldFrame()
    {
        // Test: With identity rotation, EE frame = world frame
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.2f);
        Quaternion currentRot = Quaternion.identity;

        Vector3 deltaEE = new Vector3(0.01f, 0.0f, 0.0f);  // 1cm forward in EE frame
        Vector3 expectedWorld = currentRot * deltaEE;       // Should equal deltaEE

        Debug.Log($"[DIAGNOSTIC] Identity rotation test:");
        Debug.Log($"  Delta EE: {deltaEE}");
        Debug.Log($"  Expected World: {expectedWorld}");
        Debug.Log($"  Are equal: {Vector3.Distance(deltaEE, expectedWorld) < 0.001f}");

        Assert.AreEqual(deltaEE.x, expectedWorld.x, 0.001f);
        Assert.AreEqual(deltaEE.y, expectedWorld.y, 0.001f);
        Assert.AreEqual(deltaEE.z, expectedWorld.z, 0.001f);
    }

    [Test]
    public void DiagnosticFrameTransform_90DegreeRotation_DeltaRotates()
    {
        // Test: With 90° Y rotation, +X in EE → -Z in world
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.2f);
        Quaternion currentRot = Quaternion.Euler(0, 90, 0);

        Vector3 deltaEE = new Vector3(0.01f, 0.0f, 0.0f);  // 1cm forward in EE frame
        Vector3 expectedWorld = currentRot * deltaEE;

        Debug.Log($"[DIAGNOSTIC] 90° rotation test:");
        Debug.Log($"  Delta EE: {deltaEE}");
        Debug.Log($"  Expected World: {expectedWorld}");
        Debug.Log($"  X component: {expectedWorld.x} (should be ~0)");
        Debug.Log($"  Z component: {expectedWorld.z} (should be ~-0.01)");

        Assert.AreEqual(0.0f, expectedWorld.x, 0.001f, "X should be ~0 after 90° rotation");
        Assert.AreEqual(0.0f, expectedWorld.y, 0.001f, "Y should be unchanged");
        Assert.AreEqual(-0.01f, expectedWorld.z, 0.001f, "Z should be -0.01 after rotation");
    }

    #endregion

    #region Actual OpenVLA Action Tests

    [Test]
    public void DiagnosticRealAction_FromLogs_ScaledWith2xWorkspace()
    {
        // From actual log: "Scaled action (freq=5Hz, workspace=2x): -0.008917196, -0.004085843, -0.01035585, 0.02782038, -0.0337147, 0.02243175, 0.9960784"
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.2f);  // Arbitrary starting pose
        Quaternion currentRot = Quaternion.Euler(0, 0, 0);
        float[] currentAngles = new float[6] { 0.0f, -Mathf.PI/4, Mathf.PI/4, 0.0f, Mathf.PI/4, 0.0f };

        // Actual scaled action from logs
        float[] deltaAction = new float[] {
            -0.008917196f,
            -0.004085843f,
            -0.01035585f,
            0.02782038f,
            -0.0337147f,
            0.02243175f
        };

        Debug.Log($"[DIAGNOSTIC] Real action test:");
        Debug.Log($"  Current pos: {currentPos}");
        Debug.Log($"  Current rot: {currentRot.eulerAngles}");
        Debug.Log($"  Delta action: [{string.Join(", ", deltaAction)}]");

        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        if (result == null)
        {
            Debug.LogError("[DIAGNOSTIC] IK FAILED - This is the bug we saw!");
            Debug.LogError($"  Target would have been unreachable from current pose");

            // Calculate what the target would have been
            Vector3 deltaPosEE = new Vector3(deltaAction[0], deltaAction[1], deltaAction[2]);
            Vector3 deltaPosWorld = currentRot * deltaPosEE;
            Vector3 targetPos = currentPos + deltaPosWorld;

            Debug.LogError($"  Target position: {targetPos}");
            Debug.LogError($"  Distance from origin: {targetPos.magnitude}m");
            Debug.LogError($"  UR5 max reach: ~0.85m");
        }
        else
        {
            Debug.Log($"[DIAGNOSTIC] IK SUCCEEDED");
            Debug.Log($"  Solution: [{string.Join(", ", result)}]");
        }

        Assert.IsNotNull(result, "Real OpenVLA action should have IK solution from reasonable starting pose");
    }

    [Test]
    public void DiagnosticWorkspaceReach_VariousPositions()
    {
        // Test IK solver at various positions in UR5 workspace
        float[] testDistances = { 0.2f, 0.4f, 0.6f, 0.8f };  // Distances from origin
        float[] currentAngles = new float[6] { 0.0f, -Mathf.PI/4, Mathf.PI/4, 0.0f, Mathf.PI/4, 0.0f };

        Debug.Log($"[DIAGNOSTIC] Workspace reach test:");

        foreach (float distance in testDistances)
        {
            Vector3 testPos = new Vector3(distance, 0.3f, 0.0f);
            Quaternion testRot = Quaternion.identity;

            // Small delta action
            float[] deltaAction = new float[] { 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

            float[] result = solver.SolveIKDelta(testPos, testRot, deltaAction, currentAngles);

            Debug.Log($"  Distance {distance}m: {(result != null ? "✓ Reachable" : "✗ Unreachable")}");

            if (distance <= 0.7f)  // Well within UR5 workspace
            {
                Assert.IsNotNull(result, $"Position at {distance}m should be reachable");
            }
        }
    }

    #endregion

    #region Solution Selection Diagnostics

    [Test]
    public void DiagnosticSolutionSelection_MultipleValid_PicksClosest()
    {
        // Test that when multiple solutions exist, the closest to current is chosen
        Vector3 targetPos = new Vector3(0.4f, 0.2f, 0.3f);
        Quaternion targetRot = Quaternion.identity;

        // Two different starting configurations
        float[] config1 = new float[6] { 0.1f, -0.5f, 0.3f, 0.0f, 0.2f, 0.0f };
        float[] config2 = new float[6] { 1.5f, -1.0f, 0.8f, 0.0f, 0.5f, 0.0f };

        Debug.Log($"[DIAGNOSTIC] Solution selection test:");

        float[] result1 = solver.SolveIK(targetPos, targetRot, config1);
        float[] result2 = solver.SolveIK(targetPos, targetRot, config2);

        if (result1 != null && result2 != null)
        {
            // Calculate distance from each result to its starting config
            float dist1 = 0f, dist2 = 0f;
            for (int i = 0; i < 6; i++)
            {
                float diff1 = Mathf.DeltaAngle(config1[i] * Mathf.Rad2Deg, result1[i] * Mathf.Rad2Deg);
                float diff2 = Mathf.DeltaAngle(config2[i] * Mathf.Rad2Deg, result2[i] * Mathf.Rad2Deg);
                dist1 += diff1 * diff1;
                dist2 += diff2 * diff2;
            }

            Debug.Log($"  Config 1 → Result distance: {Mathf.Sqrt(dist1)}°");
            Debug.Log($"  Config 2 → Result distance: {Mathf.Sqrt(dist2)}°");
            Debug.Log($"  Results differ: {!AreAnglesEqual(result1, result2)}");

            // Results should prefer staying close to starting config
            Assert.Less(dist1, 100f, "Solution should be reasonably close to starting config");
            Assert.Less(dist2, 100f, "Solution should be reasonably close to starting config");
        }
        else
        {
            Debug.LogWarning("[DIAGNOSTIC] One or both IK solutions failed");
        }
    }

    [Test]
    public void DiagnosticZeroDelta_AnalyzeBehavior()
    {
        // Deep dive into zero-delta behavior
        Vector3 currentPos = new Vector3(0.4f, 0.2f, 0.3f);
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6] { 0.1f, -0.5f, 0.3f, 0.0f, 0.2f, 0.0f };

        float[] zeroDelta = new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        Debug.Log($"[DIAGNOSTIC] Zero delta test:");
        Debug.Log($"  Current angles: [{string.Join(", ", currentAngles)}]");

        float[] result = solver.SolveIKDelta(currentPos, currentRot, zeroDelta, currentAngles);

        if (result != null)
        {
            Debug.Log($"  Result angles:  [{string.Join(", ", result)}]");

            // Calculate joint movement
            for (int i = 0; i < 6; i++)
            {
                float movement = Mathf.Abs(result[i] - currentAngles[i]) * Mathf.Rad2Deg;
                Debug.Log($"  Joint {i} movement: {movement:F2}°");

                if (movement > 1.0f)
                {
                    Debug.LogWarning($"  ⚠ Joint {i} moved {movement:F2}° with zero delta!");
                }
            }

            // The test expects minimal movement
            for (int i = 0; i < 6; i++)
            {
                float movement = Mathf.Abs(result[i] - currentAngles[i]) * Mathf.Rad2Deg;
                Assert.Less(movement, 10.0f,
                    $"Joint {i} should not move significantly with zero delta (moved {movement:F2}°)");
            }
        }
        else
        {
            Assert.Fail("Zero delta should always have a solution");
        }
    }

    #endregion

    #region Edge Case Diagnostics

    [Test]
    public void DiagnosticLargeDeltas_IdentifyLimit()
    {
        // Find at what delta size the IK starts failing
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.2f);
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6] { 0.0f, -Mathf.PI/4, Mathf.PI/4, 0.0f, Mathf.PI/4, 0.0f };

        float[] deltaSizes = { 0.001f, 0.01f, 0.05f, 0.1f, 0.2f, 0.5f };

        Debug.Log($"[DIAGNOSTIC] Delta size limit test:");

        foreach (float deltaSize in deltaSizes)
        {
            float[] deltaAction = new float[] { deltaSize, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

            Debug.Log($"  Delta {deltaSize}m: {(result != null ? "✓" : "✗")}");
        }

        // At least small deltas should work
        float[] smallDelta = new float[] { 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float[] resultSmall = solver.SolveIKDelta(currentPos, currentRot, smallDelta, currentAngles);
        Assert.IsNotNull(resultSmall, "Small delta (1cm) should have IK solution");
    }

    [Test]
    public void DiagnosticNearWorkspaceBoundary_Behavior()
    {
        // Test behavior near workspace limits
        Vector3 farPos = new Vector3(0.75f, 0.3f, 0.0f);  // Near max reach
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6] { 0.0f, -Mathf.PI/4, Mathf.PI/4, 0.0f, Mathf.PI/4, 0.0f };

        Debug.Log($"[DIAGNOSTIC] Workspace boundary test:");
        Debug.Log($"  Position: {farPos} (distance: {farPos.magnitude}m)");

        // Try moving away from origin (should fail)
        float[] deltaAway = new float[] { 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float[] resultAway = solver.SolveIKDelta(farPos, currentRot, deltaAway, currentAngles);
        Debug.Log($"  Move away: {(resultAway != null ? "✓ (unexpected)" : "✗ (expected)")}");

        // Try moving toward origin (should succeed)
        float[] deltaToward = new float[] { -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float[] resultToward = solver.SolveIKDelta(farPos, currentRot, deltaToward, currentAngles);
        Debug.Log($"  Move toward: {(resultToward != null ? "✓ (expected)" : "✗ (unexpected)")}");

        Assert.IsNull(resultAway, "Moving beyond workspace should fail");
        Assert.IsNotNull(resultToward, "Moving toward workspace center should succeed");
    }

    #endregion

    #region Helper Methods

    private bool AreAnglesEqual(float[] angles1, float[] angles2, float tolerance = 0.01f)
    {
        if (angles1.Length != angles2.Length) return false;

        for (int i = 0; i < angles1.Length; i++)
        {
            if (Mathf.Abs(angles1[i] - angles2[i]) > tolerance)
                return false;
        }
        return true;
    }

    #endregion
}
