using System.Collections;
using System.Collections.Generic;
//using System.Diagnostics;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [System.Serializable]
    public struct Joint
    {
        public string inputAxis;
        public GameObject robotPart;
    }
    public Joint[] joints;

    // CONTROL

    public void StopAllJointRotations()
    {
        Debug.Log("Stopping all joint rotations.");
        for (int i = 0; i < joints.Length; i++)
        {
            GameObject robotPart = joints[i].robotPart;
            Debug.Log($"Stopping joint {i}: {robotPart.name}");
            UpdateRotationState(RotationDirection.None, robotPart);
        }
    }

    public void RotateJoint(int jointIndex, RotationDirection direction)
    {
        Debug.Log($"Rotating joint {jointIndex} in direction: {direction}");
        StopAllJointRotations();
        Joint joint = joints[jointIndex];
        Debug.Log($"Selected joint {jointIndex}: {joint.robotPart.name}");
        UpdateRotationState(direction, joint.robotPart);
    }

    // HELPERS

    static void UpdateRotationState(RotationDirection direction, GameObject robotPart)
    {
        ArticulationJointController jointController = robotPart.GetComponent<ArticulationJointController>();
        Debug.Log($"Updating rotation state for {robotPart.name} to {direction}");
        jointController.rotationState = direction;
    }
}