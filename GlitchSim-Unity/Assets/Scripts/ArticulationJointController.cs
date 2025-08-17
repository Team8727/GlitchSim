using System;
using System.Collections;
using System.Collections.Generic;
//using System.Diagnostics;
using UnityEngine;

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class ArticulationJointController : MonoBehaviour
{
    [Header("Control Settings")]
    public RotationDirection rotationState = RotationDirection.None;
    public float speed = 300.0f;

    [Header("Joint Configuration")]
    public bool useVelocityControl = false; // Toggle between position and velocity control
    public float maxForce = 1000f; // For velocity control
    public float damping = 10f; // For velocity control

    private ArticulationBody articulation;
    private ArticulationJointType jointType;
    private int primaryDriveIndex = 0; // Which drive to use (0=x, 1=y, 2=z)

    // LIFE CYCLE
    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
        if (articulation != null)
        {
            jointType = articulation.jointType;
            DeterminePrimaryDriveAxis();
            ConfigureDriveSettings();
        }
    }

    void FixedUpdate()
    {
        if (articulation == null || rotationState == RotationDirection.None)
            return;

        switch (jointType)
        {
            case ArticulationJointType.RevoluteJoint:
                HandleRevoluteJoint();
                break;
            case ArticulationJointType.PrismaticJoint:
                HandlePrismaticJoint();
                break;
            case ArticulationJointType.SphericalJoint:
                HandleSphericalJoint();
                break;
            case ArticulationJointType.FixedJoint:
                // Fixed joints don't move
                Debug.LogWarning($"{gameObject.name} has a fixed joint - cannot apply rotation");
                break;
        }
    }

    // JOINT TYPE HANDLERS
    void HandleRevoluteJoint()
    {
        if (useVelocityControl)
        {
            // Use velocity control for continuous rotation
            float targetVelocity = (float)rotationState * speed * Mathf.Deg2Rad;
            SetDriveVelocity(targetVelocity);
        }
        else
        {
            // Use position control for precise positioning
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float currentRotation = GetCurrentRotation();
            float rotationGoal = currentRotation + rotationChange;
            SetDriveTarget(rotationGoal);
        }
    }

    void HandlePrismaticJoint()
    {
        // For prismatic joints, we move linearly instead of rotating
        float movementChange = (float)rotationState * speed * Time.fixedDeltaTime * 0.01f; // Convert to meters
        float currentPosition = GetCurrentPosition();
        float positionGoal = currentPosition + movementChange;
        SetDriveTarget(positionGoal);
    }

    void HandleSphericalJoint()
    {
        // For spherical joints, we can rotate around multiple axes
        // This example rotates around the primary axis, but you could extend it
        if (useVelocityControl)
        {
            float targetVelocity = (float)rotationState * speed * Mathf.Deg2Rad;
            SetDriveVelocity(targetVelocity);
        }
        else
        {
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float currentRotation = GetCurrentRotation();
            float rotationGoal = currentRotation + rotationChange;
            SetDriveTarget(rotationGoal);
        }
    }

    // DRIVE CONFIGURATION
    void DeterminePrimaryDriveAxis()
    {
        // Determine which axis is the primary one based on joint configuration
        // For ArticulationBody, we'll use a simpler approach since the lock properties
        // are different than ConfigurableJoint

        switch (jointType)
        {
            case ArticulationJointType.RevoluteJoint:
                // For revolute joints, typically use X-axis (twist) as primary
                // You can customize this based on your joint setup
                primaryDriveIndex = 0; // X-axis
                break;

            case ArticulationJointType.PrismaticJoint:
                // For prismatic joints, typically use X-axis as primary
                // You can customize this based on your joint setup
                primaryDriveIndex = 0; // X-axis
                break;

            case ArticulationJointType.SphericalJoint:
                // Default to X-axis for spherical joints
                primaryDriveIndex = 0;
                break;
        }

        Debug.Log($"{gameObject.name} - Joint Type: {jointType}, Primary Drive Index: {primaryDriveIndex}");
    }

    void ConfigureDriveSettings()
    {
        // Configure the appropriate drive based on the primary axis
        ArticulationDrive drive = GetDrive(primaryDriveIndex);

        if (useVelocityControl)
        {
            drive.stiffness = 0f;
            drive.damping = damping;
            drive.forceLimit = maxForce;
        }
        else
        {
            drive.stiffness = 10000f; // High stiffness for position control
            drive.damping = 500f;
            drive.forceLimit = maxForce;
        }

        SetDrive(primaryDriveIndex, drive);
    }

    // MOVEMENT HELPERS
    float GetCurrentRotation()
    {
        if (articulation == null || articulation.jointPosition.dofCount <= primaryDriveIndex)
        {
            Debug.LogWarning($"No joint position data available for {gameObject.name} at index {primaryDriveIndex}");
            return 0f;
        }

        float currentRotationRads = articulation.jointPosition[primaryDriveIndex];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    float GetCurrentPosition()
    {
        if (articulation == null || articulation.jointPosition.dofCount <= primaryDriveIndex)
        {
            Debug.LogWarning($"No joint position data available for {gameObject.name} at index {primaryDriveIndex}");
            return 0f;
        }

        return articulation.jointPosition[primaryDriveIndex]; // Already in meters for prismatic
    }

    void SetDriveTarget(float target)
    {
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        drive.target = target;
        SetDrive(primaryDriveIndex, drive);
    }

    void SetDriveVelocity(float targetVelocity)
    {
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        drive.targetVelocity = targetVelocity;
        SetDrive(primaryDriveIndex, drive);
    }

    ArticulationDrive GetDrive(int index)
    {
        switch (index)
        {
            case 0: return articulation.xDrive;
            case 1: return articulation.yDrive;
            case 2: return articulation.zDrive;
            default: return articulation.xDrive;
        }
    }

    void SetDrive(int index, ArticulationDrive drive)
    {
        switch (index)
        {
            case 0: articulation.xDrive = drive; break;
            case 1: articulation.yDrive = drive; break;
            case 2: articulation.zDrive = drive; break;
        }
    }

    // PUBLIC API
    public void SetRotationDirection(RotationDirection direction)
    {
        rotationState = direction;
    }

    public void SetSpeed(float newSpeed)
    {
        speed = newSpeed;
    }

    public void SetControlMode(bool velocityControl)
    {
        useVelocityControl = velocityControl;
        ConfigureDriveSettings();
    }

    public ArticulationJointType GetJointType()
    {
        return jointType;
    }

    public float GetCurrentValue()
    {
        return jointType == ArticulationJointType.PrismaticJoint ?
               GetCurrentPosition() : GetCurrentRotation();
    }
}