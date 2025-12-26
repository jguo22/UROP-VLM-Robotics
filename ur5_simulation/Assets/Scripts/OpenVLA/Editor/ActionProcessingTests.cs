using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Unit tests for action processing logic in UR5TCPServer.
/// Tests frequency scaling, workspace scaling, and action transformations.
/// </summary>
public class ActionProcessingTests
{
    #region Frequency Scaling Tests

    [Test]
    public void FrequencyScaling_SameFrequency_NoChange()
    {
        // Arrange
        float trainingFreq = 5.0f;
        float controlFreq = 5.0f;
        float frequencyScale = trainingFreq / controlFreq;
        float[] action = new float[] { 0.01f, -0.005f, 0.008f, 0.05f, -0.03f, 0.02f, 1.0f };

        // Act
        float[] scaled = ScaleAction(action, frequencyScale, 1.0f);

        // Assert
        for (int i = 0; i < 6; i++)
        {
            Assert.AreEqual(action[i], scaled[i], 0.0001f, $"Dimension {i} should be unchanged");
        }
    }

    [Test]
    public void FrequencyScaling_DoubleFrequency_HalvesActions()
    {
        // Arrange: Running at 2x training frequency
        float trainingFreq = 5.0f;
        float controlFreq = 10.0f;
        float frequencyScale = trainingFreq / controlFreq; // 0.5
        float[] action = new float[] { 0.02f, -0.01f, 0.016f, 0.1f, -0.06f, 0.04f, 1.0f };

        // Act
        float[] scaled = ScaleAction(action, frequencyScale, 1.0f);

        // Assert: Position and rotation deltas should be halved
        Assert.AreEqual(0.01f, scaled[0], 0.0001f);
        Assert.AreEqual(-0.005f, scaled[1], 0.0001f);
        Assert.AreEqual(0.008f, scaled[2], 0.0001f);
        Assert.AreEqual(0.05f, scaled[3], 0.0001f);
        Assert.AreEqual(-0.03f, scaled[4], 0.0001f);
        Assert.AreEqual(0.02f, scaled[5], 0.0001f);
        // Gripper should be unchanged
        Assert.AreEqual(1.0f, scaled[6], 0.0001f);
    }

    [Test]
    public void FrequencyScaling_HalfFrequency_DoublesActions()
    {
        // Arrange: Running at 0.5x training frequency
        float trainingFreq = 5.0f;
        float controlFreq = 2.5f;
        float frequencyScale = trainingFreq / controlFreq; // 2.0
        float[] action = new float[] { 0.01f, -0.005f, 0.008f, 0.05f, -0.03f, 0.02f, 0.0f };

        // Act
        float[] scaled = ScaleAction(action, frequencyScale, 1.0f);

        // Assert: Position and rotation deltas should be doubled
        Assert.AreEqual(0.02f, scaled[0], 0.0001f);
        Assert.AreEqual(-0.01f, scaled[1], 0.0001f);
        Assert.AreEqual(0.016f, scaled[2], 0.0001f);
        Assert.AreEqual(0.1f, scaled[3], 0.0001f);
        Assert.AreEqual(-0.06f, scaled[4], 0.0001f);
        Assert.AreEqual(0.04f, scaled[5], 0.0001f);
        // Gripper should be unchanged
        Assert.AreEqual(0.0f, scaled[6], 0.0001f);
    }

    #endregion

    #region Workspace Scaling Tests

    [Test]
    public void WorkspaceScaling_DoubleScale_DoublesActions()
    {
        // Arrange
        float workspaceScale = 2.0f;
        float[] action = new float[] { 0.01f, -0.005f, 0.008f, 0.05f, -0.03f, 0.02f, 1.0f };

        // Act
        float[] scaled = ScaleAction(action, 1.0f, workspaceScale);

        // Assert: Position and rotation deltas should be doubled
        Assert.AreEqual(0.02f, scaled[0], 0.0001f);
        Assert.AreEqual(-0.01f, scaled[1], 0.0001f);
        Assert.AreEqual(0.016f, scaled[2], 0.0001f);
        Assert.AreEqual(0.1f, scaled[3], 0.0001f);
        Assert.AreEqual(-0.06f, scaled[4], 0.0001f);
        Assert.AreEqual(0.04f, scaled[5], 0.0001f);
        // Gripper should be unchanged
        Assert.AreEqual(1.0f, scaled[6], 0.0001f);
    }

    [Test]
    public void WorkspaceScaling_HalfScale_HalvesActions()
    {
        // Arrange
        float workspaceScale = 0.5f;
        float[] action = new float[] { 0.02f, -0.01f, 0.016f, 0.1f, -0.06f, 0.04f, 0.0f };

        // Act
        float[] scaled = ScaleAction(action, 1.0f, workspaceScale);

        // Assert: Position and rotation deltas should be halved
        Assert.AreEqual(0.01f, scaled[0], 0.0001f);
        Assert.AreEqual(-0.005f, scaled[1], 0.0001f);
        Assert.AreEqual(0.008f, scaled[2], 0.0001f);
        Assert.AreEqual(0.05f, scaled[3], 0.0001f);
        Assert.AreEqual(-0.03f, scaled[4], 0.0001f);
        Assert.AreEqual(0.02f, scaled[5], 0.0001f);
        // Gripper should be unchanged
        Assert.AreEqual(0.0f, scaled[6], 0.0001f);
    }

    #endregion

    #region Combined Scaling Tests

    [Test]
    public void CombinedScaling_BothFactors_AppliesCorrectly()
    {
        // Arrange: 10 Hz control (2x faster) with 0.5x workspace scale
        float trainingFreq = 5.0f;
        float controlFreq = 10.0f;
        float frequencyScale = trainingFreq / controlFreq; // 0.5
        float workspaceScale = 0.5f;
        float combinedScale = frequencyScale * workspaceScale; // 0.25

        float[] action = new float[] { 0.04f, -0.02f, 0.032f, 0.2f, -0.12f, 0.08f, 1.0f };

        // Act
        float[] scaled = ScaleAction(action, frequencyScale, workspaceScale);

        // Assert: Should be scaled by 0.25x total
        Assert.AreEqual(0.01f, scaled[0], 0.0001f);
        Assert.AreEqual(-0.005f, scaled[1], 0.0001f);
        Assert.AreEqual(0.008f, scaled[2], 0.0001f);
        Assert.AreEqual(0.05f, scaled[3], 0.0001f);
        Assert.AreEqual(-0.03f, scaled[4], 0.0001f);
        Assert.AreEqual(0.02f, scaled[5], 0.0001f);
        Assert.AreEqual(1.0f, scaled[6], 0.0001f);
    }

    #endregion

    #region Gripper Action Tests

    [Test]
    public void GripperAction_NeverScaled_Open()
    {
        // Arrange
        float[] action = new float[] { 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.0f };

        // Act: Apply aggressive scaling
        float[] scaled = ScaleAction(action, 0.1f, 10.0f);

        // Assert: Gripper should remain unchanged
        Assert.AreEqual(0.0f, scaled[6], 0.0001f, "Gripper open should not be scaled");
    }

    [Test]
    public void GripperAction_NeverScaled_Close()
    {
        // Arrange
        float[] action = new float[] { 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 1.0f };

        // Act: Apply aggressive scaling
        float[] scaled = ScaleAction(action, 0.1f, 10.0f);

        // Assert: Gripper should remain unchanged
        Assert.AreEqual(1.0f, scaled[6], 0.0001f, "Gripper close should not be scaled");
    }

    [Test]
    public void GripperAction_IntermediateValues_Preserved()
    {
        // Arrange: OpenVLA outputs near-binary values
        float[] action = new float[] { 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.996f };

        // Act
        float[] scaled = ScaleAction(action, 2.0f, 0.5f);

        // Assert: Gripper should remain unchanged
        Assert.AreEqual(0.996f, scaled[6], 0.0001f);
    }

    #endregion

    #region Edge Cases

    [Test]
    public void ZeroActions_RemainZero()
    {
        // Arrange
        float[] action = new float[] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        // Act
        float[] scaled = ScaleAction(action, 2.5f, 1.5f);

        // Assert: All should remain zero
        for (int i = 0; i < 7; i++)
        {
            Assert.AreEqual(0.0f, scaled[i], 0.0001f, $"Dimension {i} should be zero");
        }
    }

    [Test]
    public void NegativeActions_MaintainSign()
    {
        // Arrange
        float[] action = new float[] { -0.01f, -0.005f, -0.008f, -0.05f, -0.03f, -0.02f, 0.5f };

        // Act
        float[] scaled = ScaleAction(action, 2.0f, 1.5f);

        // Assert: All scaled actions should remain negative
        for (int i = 0; i < 6; i++)
        {
            Assert.Less(scaled[i], 0.0f, $"Dimension {i} should remain negative");
        }
    }

    [Test]
    public void ScaleFactorOne_NoChange()
    {
        // Arrange
        float[] action = new float[] { 0.01f, -0.005f, 0.008f, 0.05f, -0.03f, 0.02f, 1.0f };

        // Act
        float[] scaled = ScaleAction(action, 1.0f, 1.0f);

        // Assert: All should be unchanged
        for (int i = 0; i < 7; i++)
        {
            Assert.AreEqual(action[i], scaled[i], 0.0001f, $"Dimension {i} should be unchanged");
        }
    }

    #endregion

    #region Realistic OpenVLA Actions

    [Test]
    public void RealisticOpenVLAAction_ScalesCorrectly()
    {
        // Arrange: Real action from the logs
        float[] action = new float[] {
            -0.008917196f,  // ~8.9mm
            -0.004085843f,  // ~4.1mm
            -0.01035585f,   // ~10.4mm
            0.02782038f,    // ~1.6°
            -0.0337147f,    // ~1.9°
            0.02243175f,    // ~1.3°
            0.9960784f      // gripper close
        };

        float trainingFreq = 5.0f;
        float controlFreq = 5.0f;
        float workspaceScale = 2.0f;

        // Act
        float[] scaled = ScaleAction(action, trainingFreq / controlFreq, workspaceScale);

        // Assert: Should be doubled (workspace scale only, freq scale = 1)
        Assert.AreEqual(action[0] * 2.0f, scaled[0], 0.0001f);
        Assert.AreEqual(action[1] * 2.0f, scaled[1], 0.0001f);
        Assert.AreEqual(action[2] * 2.0f, scaled[2], 0.0001f);
        Assert.AreEqual(action[3] * 2.0f, scaled[3], 0.0001f);
        Assert.AreEqual(action[4] * 2.0f, scaled[4], 0.0001f);
        Assert.AreEqual(action[5] * 2.0f, scaled[5], 0.0001f);
        Assert.AreEqual(action[6], scaled[6], 0.0001f);
    }

    #endregion

    #region Helper Methods

    /// <summary>
    /// Simulates the action scaling logic from UR5TCPServer.ProcessAction
    /// </summary>
    private float[] ScaleAction(float[] action, float frequencyScale, float workspaceScale)
    {
        float[] scaledAction = new float[action.Length];

        // Apply scaling to position and rotation deltas (first 6 dimensions)
        for (int i = 0; i < 6; i++)
        {
            scaledAction[i] = action[i] * frequencyScale * workspaceScale;
        }

        // Gripper action (last dimension) - no scaling needed
        scaledAction[6] = action[6];

        return scaledAction;
    }

    #endregion
}
