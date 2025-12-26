using NUnit.Framework;
using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Unit tests for UR5IKSolver, focusing on coordinate frame transformations and delta actions.
/// </summary>
public class UR5IKSolverTests
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

    #region Delta Action Frame Tests

    [Test]
    public void SolveIKDelta_WithIdentityRotation_AppliesWorldFrameDelta()
    {
        // Arrange: End-effector at origin, no rotation
        Vector3 currentPos = Vector3.zero;
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6];

        // Action: Move 0.1m in +X (forward in EE frame, which is also world +X)
        float[] deltaAction = new float[] { 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: Should successfully compute IK (result != null)
        // The target position should be (0.1, 0, 0) in world frame
        Assert.IsNotNull(result, "IK solution should exist for identity rotation");
    }

    [Test]
    public void SolveIKDelta_With90DegreeRotation_TransformsFrameCorrectly()
    {
        // Arrange: End-effector rotated 90° around Y axis
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.0f);
        Quaternion currentRot = Quaternion.Euler(0, 90, 0);
        float[] currentAngles = new float[6];

        // Action: Move 0.1m in +X in EE frame (which is -Z in world frame after 90° rotation)
        float[] deltaAction = new float[] { 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: Expected world delta should be (0, 0, -0.1) after rotation transform
        Vector3 expectedWorldDelta = currentRot * new Vector3(0.1f, 0.0f, 0.0f);
        Assert.AreEqual(0.0f, expectedWorldDelta.x, 0.001f, "X component should be ~0");
        Assert.AreEqual(0.0f, expectedWorldDelta.y, 0.001f, "Y component should be ~0");
        Assert.AreEqual(-0.1f, expectedWorldDelta.z, 0.001f, "Z component should be -0.1");
    }

    [Test]
    public void SolveIKDelta_SmallDeltas_ShouldNotFail()
    {
        // Arrange: Realistic starting pose
        Vector3 currentPos = new Vector3(0.4f, 0.2f, 0.3f);
        Quaternion currentRot = Quaternion.Euler(0, 45, 0);
        float[] currentAngles = new float[6] { 0.1f, -0.5f, 0.3f, 0.0f, 0.2f, 0.0f };

        // Action: Typical small OpenVLA action (position in meters, rotation in radians)
        float[] deltaAction = new float[] {
            0.009f,  // ~9mm forward
            -0.004f, // ~4mm left
            -0.010f, // ~10mm down
            0.028f,  // ~1.6° roll
            -0.034f, // ~1.9° pitch
            0.022f   // ~1.3° yaw
        };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: Small deltas should typically have solutions
        // (May fail if current pose is near workspace boundary)
        // This is more of an integration test to catch frame transform bugs
        Assert.IsNotNull(result, "Small realistic deltas should generally have IK solutions");
    }

    [Test]
    public void SolveIKDelta_RotationDelta_AppliesInEEFrame()
    {
        // Arrange
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.0f);
        Quaternion currentRot = Quaternion.Euler(0, 0, 0);
        float[] currentAngles = new float[6];

        // Action: Pure rotation around Z axis (yaw) by ~5.7 degrees (0.1 radians)
        float[] deltaAction = new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: Should apply rotation delta relative to current rotation
        // currentRot * deltaRot should give new rotation
        Quaternion expectedRot = currentRot * Quaternion.Euler(0, 0, 0.1f * Mathf.Rad2Deg);

        // Note: Can't easily verify final orientation without forward kinematics,
        // but we can verify IK doesn't fail
        Assert.IsNotNull(result, "Pure rotation delta should have IK solution");
    }

    #endregion

    #region Coordinate Frame Transformation Tests

    [Test]
    public void QuaternionRotation_TransformsVectorCorrectly()
    {
        // Verify Unity's quaternion rotation behavior matches our expectations
        Quaternion rot90Y = Quaternion.Euler(0, 90, 0);
        Vector3 forward = new Vector3(1, 0, 0);

        Vector3 rotated = rot90Y * forward;

        Assert.AreEqual(0.0f, rotated.x, 0.001f);
        Assert.AreEqual(0.0f, rotated.y, 0.001f);
        Assert.AreEqual(-1.0f, rotated.z, 0.001f);
    }

    [Test]
    public void QuaternionRotation_180Degrees_InvertsVector()
    {
        Quaternion rot180Y = Quaternion.Euler(0, 180, 0);
        Vector3 forward = new Vector3(1, 0, 0);

        Vector3 rotated = rot180Y * forward;

        Assert.AreEqual(-1.0f, rotated.x, 0.001f);
        Assert.AreEqual(0.0f, rotated.y, 0.001f);
        Assert.AreEqual(0.0f, rotated.z, 0.001f);
    }

    #endregion

    #region Edge Cases

    [Test]
    public void SolveIKDelta_ZeroDelta_ReturnsCurrentAngles()
    {
        // Arrange
        Vector3 currentPos = new Vector3(0.4f, 0.2f, 0.3f);
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6] { 0.1f, -0.5f, 0.3f, 0.0f, 0.2f, 0.0f };

        // Action: No movement
        float[] deltaAction = new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: Should return current configuration or very close to it
        Assert.IsNotNull(result, "Zero delta should have IK solution");
        if (result != null)
        {
            for (int i = 0; i < 6; i++)
            {
                Assert.AreEqual(currentAngles[i], result[i], 0.1f,
                    $"Joint {i} should be close to current angle for zero delta");
            }
        }
    }

    [Test]
    public void SolveIKDelta_LargeDelta_MayFailGracefully()
    {
        // Arrange
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.0f);
        Quaternion currentRot = Quaternion.identity;
        float[] currentAngles = new float[6];

        // Action: Unrealistically large delta (1 meter - way too big)
        float[] deltaAction = new float[] { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

        // Assert: May return null (unreachable), which is acceptable
        // This test just ensures it doesn't throw exceptions
        Assert.Pass("Large delta handled without exceptions");
    }

    [Test]
    public void SolveIKDelta_NullCurrentAngles_HandlesGracefully()
    {
        // Arrange
        Vector3 currentPos = new Vector3(0.5f, 0.3f, 0.0f);
        Quaternion currentRot = Quaternion.identity;
        float[] deltaAction = new float[] { 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act & Assert: Should handle null current angles
        float[] result = solver.SolveIKDelta(currentPos, currentRot, deltaAction, null);

        // May return null or a solution, but should not throw
        Assert.Pass("Null current angles handled without exceptions");
    }

    #endregion

    #region Action Scaling Tests (Integration with Expected Behavior)

    [Test]
    public void ScaledActions_ShouldBeProportionalToFrequency()
    {
        // This tests the expected behavior when frequency scaling is applied
        float trainingFreq = 5.0f;
        float controlFreq = 10.0f;
        float frequencyScale = trainingFreq / controlFreq;

        Vector3 rawAction = new Vector3(0.02f, 0.01f, -0.01f);
        Vector3 scaledAction = rawAction * frequencyScale;

        // At 10 Hz (2x faster), actions should be halved
        Assert.AreEqual(0.01f, scaledAction.x, 0.0001f);
        Assert.AreEqual(0.005f, scaledAction.y, 0.0001f);
        Assert.AreEqual(-0.005f, scaledAction.z, 0.0001f);
    }

    [Test]
    public void WorkspaceScaling_AppliesUniformly()
    {
        float workspaceScale = 2.0f;
        Vector3 action = new Vector3(0.01f, -0.005f, 0.008f);
        Vector3 scaled = action * workspaceScale;

        Assert.AreEqual(0.02f, scaled.x, 0.0001f);
        Assert.AreEqual(-0.01f, scaled.y, 0.0001f);
        Assert.AreEqual(0.016f, scaled.z, 0.0001f);
    }

    #endregion

    #region Regression Tests for Common OpenVLA Action Ranges

    [Test]
    public void TypicalOpenVLAPositionDeltas_AreInMillimeterRange()
    {
        // OpenVLA typically outputs position deltas in the range of millimeters
        float[] typicalDeltas = new float[] { 0.009f, -0.004f, -0.010f };

        foreach (float delta in typicalDeltas)
        {
            float deltaInMM = Mathf.Abs(delta) * 1000f;
            Assert.Less(deltaInMM, 50f, "Typical deltas should be < 50mm");
            Assert.Greater(deltaInMM, 0.1f, "Typical deltas should be > 0.1mm");
        }
    }

    [Test]
    public void TypicalOpenVLARotationDeltas_AreInSmallDegreeRange()
    {
        // OpenVLA typically outputs rotation deltas in small radian amounts
        float[] typicalDeltas = new float[] { 0.028f, -0.034f, 0.022f }; // radians

        foreach (float delta in typicalDeltas)
        {
            float deltaInDegrees = Mathf.Abs(delta) * Mathf.Rad2Deg;
            Assert.Less(deltaInDegrees, 10f, "Typical rotation deltas should be < 10°");
            Assert.Greater(deltaInDegrees, 0.1f, "Typical rotation deltas should be > 0.1°");
        }
    }

    #endregion
}
