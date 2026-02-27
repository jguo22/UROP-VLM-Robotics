using UnityEngine;
using System;

public class BlockSetup : MonoBehaviour
{
    [Header("Scene References")]
    public GameObject targetBlock;

    private void Start()
    {
        if (targetBlock == null)
        {
            Debug.LogError("Block is not assigned.");
            return;
        }
    }
}
