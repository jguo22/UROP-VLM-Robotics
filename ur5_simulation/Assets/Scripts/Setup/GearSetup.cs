using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GearSetup : MonoBehaviour
{
    public GameObject[] originalGears;

    void Start()
    {
        foreach (var gear in originalGears)
        {
            Debug.Log(
                "Gear: "
                    + gear.name
                    + " Center: "
                    + gear.GetComponent<Collider>().bounds.center.ToString("F5")
            );
        }
    }
}
