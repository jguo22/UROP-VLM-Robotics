using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using static ConstantsUR5;

[DefaultExecutionOrder(-1)]
public class SuctionController : MonoBehaviour
{
    [Header("Suction Settings")]
    [Tooltip("Enable/disable suction functionality")]
    public bool enableSuction = false;

    [Tooltip("Maximum distance for suction activation (1cm = 0.01 units)")]
    public float suctionDistance = SuctionDistance;

    [Tooltip("Strength of suction attraction force")]
    public float suctionForce = 10f;

    [Tooltip("Speed of attraction when suction is active")]
    public float attractionSpeed = 5f;

    [Tooltip("Show visual debug information")]
    public bool showDebugInfo = true;

    [Header("Visual Feedback")]
    public Material suctionActiveMaterial;
    public Material suctionInactiveMaterial;

    //private BlockSetup blockSetup;
    private ArticulationBody[] articulationChain;
    private ArticulationBody[] robotJoints;

    [HideInInspector]
    public GameObject[] targetBlocks;
    private GameObject targetBlock;
    private Transform endEffector;
    private Rigidbody blockRigidbody;
    private Renderer blockRenderer;
    private Material originalBlockMaterial;

    // Suction state
    [HideInInspector]
    public bool isBlockAttached;
    private bool wasSuctionActive = false;
    private float currentDistance;
    private List<float> currentDistances = new List<float>();

    // Visual feedback
    private LineRenderer suctionLine;
    private GameObject suctionIndicator;

    void Start()
    {
        InitializeComponents();
        //SetupVisualFeedback();
    }

    void InitializeComponents()
    {
        //blockSetup = GetComponent<BlockSetup>();
        //targetBlock = blockSetup.targetBlock;

        // if (targetBlock == null)
        // {
        //     Debug.LogError("Missing required components in SuctionController!");
        //     enabled = false;
        //     return;
        // }

        // Get articulation chain and setup joints
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();

        if (articulationChain == null || articulationChain.Length == 0)
        {
            Debug.LogError("Articulation chain is not properly set up in SuctionController.");
            return;
        }

        for (int i = BaseIndex; i < BaseIndex + JointCount; i++)
        {
            ArticulationBody joint = articulationChain[i];
            joint.gameObject.AddComponent<JointControl>();
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

        // Get block components
        // blockRigidbody = targetBlock.GetComponent<Rigidbody>();
        // blockRenderer = targetBlock.GetComponent<Renderer>();
        // if (blockRenderer != null)
        // {
        //     originalBlockMaterial = blockRenderer.material;
        // }
    }

    // void SetupVisualFeedback()
    // {
    //     // Create suction line renderer
    //     suctionLine = gameObject.AddComponent<LineRenderer>();
    //     suctionLine.material = new Material(Shader.Find("Sprites/Default"));
    //     suctionLine.color = Color.red;
    //     suctionLine.startWidth = 0.002f;
    //     suctionLine.endWidth = 0.002f;
    //     suctionLine.positionCount = 2;
    //     suctionLine.enabled = false;

    //     // Create suction indicator sphere
    //     suctionIndicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
    //     suctionIndicator.name = "SuctionIndicator";
    //     suctionIndicator.transform.localScale = Vector3.one * 0.02f; // 2cm sphere
    //     suctionIndicator.transform.SetParent(endEffector);
    //     suctionIndicator.transform.localPosition = Vector3.zero;

    //     // Remove collider from indicator
    //     Collider indicatorCollider = suctionIndicator.GetComponent<Collider>();
    //     if (indicatorCollider != null)
    //     {
    //         Destroy(indicatorCollider);
    //     }

    //     // Set indicator material
    //     Renderer indicatorRenderer = suctionIndicator.GetComponent<Renderer>();
    //     if (indicatorRenderer != null)
    //     {
    //         indicatorRenderer.material = suctionInactiveMaterial != null ? suctionInactiveMaterial : originalBlockMaterial;
    //     }

    //     suctionIndicator.SetActive(false);
    // }

    void Update()
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

            Collider blockCollider = block.GetComponent<Collider>();
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
        if (blockRenderer != null)
        {
            originalBlockMaterial = blockRenderer.material;
        }

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

    void HandleSuctionState()
    {
        bool shouldBeActive = enableSuction && currentDistance <= suctionDistance;

        // State change detection
        if (shouldBeActive != wasSuctionActive)
        {
            if (shouldBeActive)
            {
                Debug.Log(
                    $"Suction activated - Distance: {currentDistance:F4}m (threshold: {suctionDistance}m)"
                );
            }
            else
            {
                Debug.Log(
                    $"Suction deactivated - Distance: {currentDistance:F4}m (threshold: {suctionDistance}m)"
                );
            }
            wasSuctionActive = shouldBeActive;
        }
    }

    void HandleBlockAttachment()
    {
        if (targetBlock == null)
            return;

        bool shouldBeAttached = enableSuction && currentDistance <= suctionDistance;

        if (shouldBeAttached && !isBlockAttached)
        {
            AttachBlock();
        }
        else if (!shouldBeAttached && isBlockAttached)
        {
            DetachBlock();
        }

        // Apply attraction force when suction is active but block isn't fully attached
        if (enableSuction && currentDistance <= suctionDistance * 2f && !isBlockAttached)
        {
            ApplyAttractionForce();
        }
    }

    void AttachBlock()
    {
        if (isBlockAttached)
            return;

        isBlockAttached = true;

        // Make block kinematic and parent to end effector
        if (blockRigidbody != null)
        {
            blockRigidbody.isKinematic = true;
        }

        Debug.Log("Can attach block");

        targetBlock.transform.SetParent(endEffector);
        //targetBlock.transform.localPosition = Vector3.zero;
        //targetBlock.transform.localRotation = Quaternion.identity;

        // Change block material to indicate suction
        if (blockRenderer != null && suctionActiveMaterial != null)
        {
            blockRenderer.material = suctionActiveMaterial;
        }

        Debug.Log("Block attached to suction");
        //suctionDistance = 0.1f;
    }

    void DetachBlock()
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
        if (blockRenderer != null && originalBlockMaterial != null)
        {
            blockRenderer.material = originalBlockMaterial;
        }

        Debug.Log("Block detached from suction");
    }

    void ApplyAttractionForce()
    {
        if (blockRigidbody == null || blockRigidbody.isKinematic)
            return;

        Vector3 direction = (endEffector.position - targetBlock.transform.position).normalized;
        float forceMultiplier = Mathf.Lerp(0f, 1f, 1f - (currentDistance / (suctionDistance * 2f)));

        Vector3 attractionForce = direction * suctionForce * forceMultiplier;
        blockRigidbody.AddForce(attractionForce, ForceMode.Force);

        // Also apply some damping to prevent oscillation
        blockRigidbody.linearVelocity *= 0.9f;
        blockRigidbody.angularVelocity *= 0.9f;
    }

    void UpdateVisualFeedback()
    {
        // Update suction indicator
        if (suctionIndicator != null)
        {
            bool shouldShowIndicator = enableSuction && currentDistance <= suctionDistance * 3f;
            suctionIndicator.SetActive(shouldShowIndicator);

            if (shouldShowIndicator)
            {
                // Change color based on suction state
                Renderer indicatorRenderer = suctionIndicator.GetComponent<Renderer>();
                if (indicatorRenderer != null)
                {
                    if (isBlockAttached)
                    {
                        indicatorRenderer.material.color = Color.green;
                    }
                    else if (currentDistance <= suctionDistance)
                    {
                        indicatorRenderer.material.color = Color.yellow;
                    }
                    else
                    {
                        indicatorRenderer.material.color = Color.red;
                    }
                }
            }
        }

        // Update suction line
        // if (suctionLine != null)
        // {
        //     bool shouldShowLine = enableSuction && currentDistance <= suctionDistance * 2f;
        //     suctionLine.enabled = shouldShowLine;

        //     if (shouldShowLine)
        //     {
        //         suctionLine.SetPosition(0, endEffector.position);
        //         suctionLine.SetPosition(1, targetBlock.transform.position);

        //         // Change line color based on distance
        //         if (currentDistance <= suctionDistance)
        //         {
        //             //suctionLine.color = Color.green;
        //         }
        //         else
        //         {
        //             //suctionLine.color = Color.yellow;
        //         }
        //     }
        // }
    }

    // Public methods for external control
    public void ToggleSuction()
    {
        enableSuction = !enableSuction;
        Debug.Log($"Suction toggled: {enableSuction}");
    }

    public void SetSuctionState(bool active)
    {
        enableSuction = active;
        //Debug.Log($"Suction set to: {enableSuction}");
    }

    public bool IsBlockAttached()
    {
        return isBlockAttached;
    }

    public float GetDistanceToBlock()
    {
        return currentDistance;
    }

    public bool IsWithinSuctionRange()
    {
        return currentDistance <= suctionDistance;
    }

    public void ResetSuctionState()
    {
        enableSuction = false;
        isBlockAttached = false;
        wasSuctionActive = false;
        currentDistance = float.MaxValue;

        if (targetBlocks != null)
        {
            foreach (GameObject target in targetBlocks)
            {
                if (target == null)
                    continue;

                Rigidbody rb = target.GetComponent<Rigidbody>();
                if (rb != null)
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
        }

        targetBlock = null;
        blockRigidbody = null;
        blockRenderer = null;
    }

    // Debug information
    void OnGUI()
    {
        if (!showDebugInfo)
            return;

        GUIStyle style = new GUIStyle(GUI.skin.label);
        style.fontSize = 14;
        style.normal.textColor = Color.white;

        float yOffset = 140; // Below the existing GUI elements

        //GUI.Label(new Rect(10, yOffset, 400, 20), $"Suction Distance: {currentDistance:F4}m", style);
        //GUI.Label(new Rect(10, yOffset + 20, 400, 20), $"Suction Threshold: {suctionDistance}m", style);
        //GUI.Label(new Rect(10, yOffset + 40, 400, 20), $"Block Attached: {isBlockAttached}", style);
        //GUI.Label(new Rect(10, yOffset + 60, 400, 20), $"In Range: {IsWithinSuctionRange()}", style);

        // Distance indicator with color coding
        string distanceStatus;
        Color statusColor;
        if (currentDistance <= suctionDistance)
        {
            distanceStatus = "IN RANGE";
            statusColor = Color.green;
        }
        else if (currentDistance <= suctionDistance * 2f)
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
        //GUI.Label(new Rect(10, yOffset + 80, 400, 20), $"Status: {distanceStatus}", style);
    }
}
