using System;
using UnityEngine;
using static ConstantsUR5;

[DefaultExecutionOrder(-1)]
public class SuctionController : MonoBehaviour
{
    private const bool ShowDebugInfo = true;

    private const float SuctionDistance = 0.01f;
    private const float SuctionOffsetY = -0.01f;

    public bool enableSuction = false;

    [Header("Visual Feedback")] public Material suctionActiveMaterial;

    public Material suctionInactiveMaterial;

    [HideInInspector] public GameObject[] targetBlocks;

    // Suction state
    [HideInInspector] public bool isBlockAttached;

    //private BlockSetup blockSetup;
    private ArticulationBody[] articulationChain;
    private Renderer blockRenderer;
    private Rigidbody blockRigidbody;
    private float currentDistance;
    private Transform endEffector;
    private Material originalBlockMaterial;
    private ArticulationBody[] robotJoints;
    private GameObject suctionIndicator;

    // Visual feedback
    private LineRenderer suctionLine;
    private GameObject targetBlock;
    private bool wasSuctionActive = false;

    private void Start()
    {
        InitializeComponents();
        //SetupVisualFeedback();
    }

    private void Update()
    {
        if (targetBlocks == null || targetBlocks.Length == 0)
        {
            currentDistance = float.MaxValue;
            targetBlock = null;
            return;
        }

        //currentDistances.Clear();
        float minDistance = float.MaxValue;
        int closestIndex = -1;

        // Calculate distances to all target blocks
        for (int i = 0; i < targetBlocks.Length; i++)
        {
            GameObject block = targetBlocks[i];
            if (block == null)
                continue;

            var blockCollider = block.GetComponent<Collider>();
            if (blockCollider == null)
                continue;
            // Vector3 blockPosition = blockCollider != null ? blockCollider.bounds.center : block.transform.position;
            Vector3 blockPosition = blockCollider.bounds.center;

            // if (blockCollider == null || isBlockAttached) blockPosition = block.transform.position;
            // else blockPosition = blockCollider.bounds.center;
            //Vector3 blockPosition = blockCollider != null ? blockCollider.ClosestPoint(endEffector.position) : block.transform.position;

            //Vector3 blockPosition = block.transform.position;
            //blockPosition.x += GearOriginOffsetX;

            Vector3 endEffectorPos = endEffector.position;
            endEffectorPos.y += SuctionOffsetY; // Adjust for suction cup offset

            float distance = Vector3.Distance(endEffectorPos, blockPosition); //targetBlocks[i].transform.position
            //currentDistances.Add(distance);

            if (distance < minDistance) // Find the closest block
            {
                minDistance = distance;
                closestIndex = i;
            }
        }

        if (closestIndex < 0)
        {
            currentDistance = float.MaxValue;
            targetBlock = null;
            return;
        }

        currentDistance = isBlockAttached ? 0.0f : minDistance; //isBlockAttached ? 0.0f : minDistance;
        targetBlock = targetBlocks[closestIndex];
        //Debug.Log(targetBlock.name + " Position: " + targetBlock.GetComponent<Collider>().bounds.center);

        // Get block components
        blockRigidbody = targetBlock.GetComponent<Rigidbody>();
        blockRenderer = targetBlock.GetComponent<Renderer>();
        if (blockRenderer != null) originalBlockMaterial = blockRenderer.material;

        // Calculate distance to nearest block
        //currentDistance = currentDistances.Min();
        //currentDistance = Vector3.Distance(endEffector.position, targetBlock.transform.position);

        // Handle suction activation/deactivation
        HandleSuctionState();

        // Update visual feedback
        UpdateVisualFeedback();

        // Handle block attachment/detachment
        HandleBlockAttachment();
    }

    // Debug information
    private void OnGUI()
    {
        if (!ShowDebugInfo)
            return;

        float refWidth = 1920f;
        float refHeight = 1080f;
        float scaleX = Screen.width / refWidth;
        float scaleY = Screen.height / refHeight;
        float scale = Mathf.Min(scaleX, scaleY);

        Matrix4x4 prevMatrix = GUI.matrix;
        GUI.matrix = Matrix4x4.TRS(
            Vector3.zero,
            Quaternion.identity,
            new Vector3(scale, scale, 1f)
        );

        var style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.normal.textColor = Color.white;

        float yOffset = 140f;

        string distanceStatus;
        Color statusColor;
        if (currentDistance <= SuctionDistance)
        {
            distanceStatus = "IN RANGE";
            statusColor = Color.green;
        }
        else if (currentDistance <= SuctionDistance * 2f)
        {
            distanceStatus = "APPROACHING";
            statusColor = Color.yellow;
        }
        else
        {
            distanceStatus = "TOO FAR";
            statusColor = Color.red;
        }

        style.normal.textColor = statusColor;

        GUI.Label(
            new Rect(10, yOffset, 400, 20),
            $"Suction Distance: {currentDistance:F4}m",
            style
        );
        GUI.Label(
            new Rect(10, yOffset + 20, 400, 20),
            $"Suction Threshold: {SuctionDistance}m",
            style
        );
        GUI.Label(new Rect(10, yOffset + 40, 400, 20), $"Block Attached: {isBlockAttached}", style);
        GUI.Label(
            new Rect(10, yOffset + 60, 400, 20),
            $"In Range: {IsWithinSuctionRange()}",
            style
        );
        GUI.Label(new Rect(10, yOffset + 80, 400, 20), $"Status: {distanceStatus}", style);

        GUI.matrix = prevMatrix;
    }

    private void InitializeComponents()
    {
        // Get articulation chain and setup joints
        articulationChain = GetComponentsInChildren<ArticulationBody>();

        if (articulationChain == null || articulationChain.Length == 0)
        {
            Debug.LogError("Articulation chain is not properly set up in SuctionController.");
            return;
        }

        for (int i = BaseIndex; i < BaseIndex + JointCount; i++)
        {
            ArticulationBody joint = articulationChain[i];
            joint.jointFriction = JointFriction;
            joint.angularDamping = AngularDamping;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = ForceLimit;
            joint.xDrive = currentDrive;
        }

        robotJoints = new ArticulationBody[JointCount + 1];
        Array.Copy(articulationChain, BaseIndex, robotJoints, 0, JointCount);
        robotJoints[JointCount] = articulationChain[EndEffectorIndex];

        // Get end effector transform
        endEffector = robotJoints[JointCount].transform;
    }

    private void HandleSuctionState()
    {
        bool shouldBeActive = enableSuction && currentDistance <= SuctionDistance;

        // State change detection
        if (shouldBeActive != wasSuctionActive)
            // if (shouldBeActive)
            // {
            //     Debug.Log(
            //         $"Suction activated - Distance: {currentDistance:F4}m (threshold: {suctionDistance}m)"
            //     );
            // }
            // else
            // {
            //     Debug.Log(
            //         $"Suction deactivated - Distance: {currentDistance:F4}m (threshold: {suctionDistance}m)"
            //     );
            // }
            wasSuctionActive = shouldBeActive;
    }

    private void HandleBlockAttachment()
    {
        if (targetBlock == null)
            return;

        bool shouldBeAttached = enableSuction && currentDistance <= SuctionDistance;

        if (shouldBeAttached && !isBlockAttached)
            AttachBlock();
        else if (!shouldBeAttached && isBlockAttached) DetachBlock();

        // Apply attraction force when suction is active but block isn't fully attached
        if (enableSuction && currentDistance <= SuctionDistance * 2f && !isBlockAttached)
        {
            // ApplyAttractionForce();
        }
    }

    private void AttachBlock()
    {
        if (isBlockAttached)
            return;

        isBlockAttached = true;

        // Make block kinematic and parent to end effector
        if (blockRigidbody != null) blockRigidbody.isKinematic = true;

        // Debug.Log("Can attach block");

        targetBlock.transform.SetParent(endEffector);
        //targetBlock.transform.localPosition = Vector3.zero;
        //targetBlock.transform.localRotation = Quaternion.identity;

        // Change block material to indicate suction
        if (blockRenderer != null && suctionActiveMaterial != null) blockRenderer.material = suctionActiveMaterial;

        // Debug.Log("Block attached to suction");
        //suctionDistance = 0.1f;
    }

    private void DetachBlock()
    {
        if (!isBlockAttached)
            return;

        isBlockAttached = false;
        //suctionDistance = 0.025f; // Reset suction distance to default value

        // Restore block physics
        // if (blockRigidbody != null)
        // {
        //     blockRigidbody.isKinematic = false;
        // }

        foreach (GameObject target in targetBlocks)
        {
            target.GetComponent<Rigidbody>().isKinematic = false;
            target.transform.SetParent(null);
        }

        //targetBlock.transform.SetParent(null);

        // Restore original block material
        if (blockRenderer != null && originalBlockMaterial != null) blockRenderer.material = originalBlockMaterial;

        // Debug.Log("Block detached from suction");
    }

    private void UpdateVisualFeedback()
    {
        // Update suction indicator
        if (suctionIndicator == null) return;
        bool shouldShowIndicator = enableSuction && currentDistance <= SuctionDistance * 3f;
        suctionIndicator.SetActive(shouldShowIndicator);

        if (!shouldShowIndicator) return;

        // Change color based on suction state
        var indicatorRenderer = suctionIndicator.GetComponent<Renderer>();

        if (indicatorRenderer == null) return;

        if (isBlockAttached)
            indicatorRenderer.material.color = Color.green;
        else if (currentDistance <= SuctionDistance)
            indicatorRenderer.material.color = Color.yellow;
        else
            indicatorRenderer.material.color = Color.red;
    }

    // Public methods for external control
    public void ToggleSuction()
    {
        enableSuction = !enableSuction;
        // Debug.Log($"Suction toggled: {enableSuction}");
    }

    public void SetSuctionState(bool active)
    {
        enableSuction = active;
        //Debug.Log($"Suction set to: {enableSuction}");
    }

    private bool IsWithinSuctionRange()
    {
        return currentDistance <= SuctionDistance;
    }

    public void ResetSuctionState()
    {
        enableSuction = false;
        isBlockAttached = false;
        wasSuctionActive = false;
        currentDistance = float.MaxValue;

        if (targetBlocks != null)
            foreach (GameObject target in targetBlocks)
            {
                if (target == null)
                    continue;

                var rb = target.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }

        targetBlock = null;
        blockRigidbody = null;
        blockRenderer = null;
    }
}