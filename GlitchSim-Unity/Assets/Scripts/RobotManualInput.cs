using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotManualInput : MonoBehaviour
{
    public GameObject robot;


    void Update()
    {
        // Check if robot GameObject is assigned
        if (robot == null)
        {
            Debug.LogError("Robot GameObject is not assigned in RobotManualInput. Please assign it in the Inspector.");
            return;
        }

        // Get the UnifiedJointController component
        UnifiedJointController jointController = robot.GetComponent<UnifiedJointController>();
        if (jointController == null)
        {
            Debug.LogError("UnifiedJointController component not found on the assigned robot GameObject.");
            return;
        }

        // Check if joints array is initialized
        if (jointController.joints == null)
        {
            Debug.LogError("Joints array is not initialized in UnifiedJointController.");
            return;
        }

        // Note: The UnifiedJointController already handles input internally in its Update method
        // This manual input script may be redundant, but keeping for compatibility
        bool anyInputDetected = false;
        for (int i = 0; i < jointController.joints.Length; i++)
        {
            float inputVal = Input.GetAxis(jointController.joints[i].inputAxis);
            if (Mathf.Abs(inputVal) > 0.01f) // Small threshold to avoid noise
            {
                Debug.Log($"RobotManualInput: Input detected on joint {i} ({jointController.joints[i].inputAxis}): {inputVal:F3}");
                anyInputDetected = true;
            }
        }
        
        // The UnifiedJointController handles the actual joint movement
        // This script now just monitors for input but doesn't directly control joints

    }
}
