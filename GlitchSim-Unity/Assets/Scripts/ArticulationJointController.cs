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
    public float speed = 1800.0f; // Increased from 300 to 1800 for more visible movement

    [Header("Joint Configuration")]
    public bool useVelocityControl = false; // Toggle between position and velocity control
    public bool enforceLimits = true; // Toggle to enable/disable limit enforcement
    [Range(0.1f, 10.0f)]
    public float speedMultiplier = 1.0f; // Additional speed multiplier for fine-tuning
    public float maxForce = 1000f; // For velocity control
    public float damping = 10f; // For velocity control

    private ArticulationBody articulation;
    private ArticulationJointType jointType;
    private int primaryDriveIndex = 0; // Which drive to use (0=x, 1=y, 2=z)
    private float lowerLimit;
    private float upperLimit;
    private bool usingDefaultLimits = false;

    // LIFE CYCLE
    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
        if (articulation != null)
        {
            // Check if this is a root ArticulationBody
            if (articulation.isRoot)
            {
                Debug.LogWarning($"{gameObject.name} - This is a ROOT ArticulationBody. Root bodies cannot be controlled as they have no joint. This component will be disabled.");
                this.enabled = false;
                return;
            }
            
            jointType = articulation.jointType;
            DeterminePrimaryDriveAxis();
            SetupJointLimits();
            ConfigureDriveSettings();
        }
        else
        {
            Debug.LogError($"{gameObject.name} - No ArticulationBody component found! ArticulationJointController requires an ArticulationBody.");
            this.enabled = false;
        }
    }

    void FixedUpdate()
    {
        if (articulation == null || rotationState == RotationDirection.None)
            return;

        Debug.Log($"{gameObject.name} - Driving {jointType} with direction: {rotationState}, speed: {speed}");

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
        float currentRotation = GetCurrentRotation();
        
        if (useVelocityControl)
        {
            // Use velocity control for continuous rotation, but check limits if enforced
            float targetVelocity = (float)rotationState * speed * Mathf.Deg2Rad;
            
            if (enforceLimits)
            {
                // Stop if we're at the limit and trying to move further in that direction
                if ((currentRotation >= upperLimit && rotationState == RotationDirection.Positive) ||
                    (currentRotation <= lowerLimit && rotationState == RotationDirection.Negative))
                {
                    targetVelocity = 0f;
                    Debug.Log($"{gameObject.name} - Revolute joint at limit! Current: {currentRotation:F2}°, Limits: [{lowerLimit:F2}, {upperLimit:F2}]");
                }
            }
            
            Debug.Log($"{gameObject.name} - Setting revolute velocity: {targetVelocity:F3} rad/s (current rotation: {currentRotation:F2}°)");
            SetDriveVelocity(targetVelocity);
        }
        else
        {
            // Use position control for precise positioning
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float rotationGoal = currentRotation + rotationChange;
            
            // Clamp the rotation goal to joint limits if enforced
            if (enforceLimits)
            {
                float originalGoal = rotationGoal;
                rotationGoal = Mathf.Clamp(rotationGoal, lowerLimit, upperLimit);
                if (Mathf.Abs(originalGoal - rotationGoal) > 0.001f)
                {
                    Debug.Log($"{gameObject.name} - Revolute goal clamped from {originalGoal:F2}° to {rotationGoal:F2}°");
                }
            }
            
            Debug.Log($"{gameObject.name} - Setting revolute target: {rotationGoal:F2}° (current: {currentRotation:F2}°, change: {rotationChange:F2}°)");
            SetDriveTarget(rotationGoal);
        }
    }

    void HandlePrismaticJoint()
    {
        float currentPosition = GetCurrentPosition();
        
        if (useVelocityControl)
        {
            // Use velocity control for continuous movement, but check limits if enforced
            float targetVelocity = (float)rotationState * speed * 0.01f; // Convert to m/s
            
            if (enforceLimits)
            {
                // Stop if we're at the limit and trying to move further in that direction
                if ((currentPosition >= upperLimit && rotationState == RotationDirection.Positive) ||
                    (currentPosition <= lowerLimit && rotationState == RotationDirection.Negative))
                {
                    targetVelocity = 0f;
                    Debug.Log($"{gameObject.name} - Prismatic joint at limit! Current: {currentPosition:F3}m, Limits: [{lowerLimit:F3}, {upperLimit:F3}]");
                }
            }
            
            Debug.Log($"{gameObject.name} - Setting prismatic velocity: {targetVelocity:F3} m/s (current position: {currentPosition:F3}m)");
            SetDriveVelocity(targetVelocity);
        }
        else
        {
            // For prismatic joints, we move linearly instead of rotating
            float movementChange = (float)rotationState * speed * Time.fixedDeltaTime * 0.01f; // Convert to meters
            float positionGoal = currentPosition + movementChange;
            
            // Clamp the position goal to joint limits if enforced
            if (enforceLimits)
            {
                float originalGoal = positionGoal;
                positionGoal = Mathf.Clamp(positionGoal, lowerLimit, upperLimit);
                if (Mathf.Abs(originalGoal - positionGoal) > 0.001f)
                {
                    Debug.Log($"{gameObject.name} - Prismatic goal clamped from {originalGoal:F3}m to {positionGoal:F3}m");
                }
            }
            
            Debug.Log($"{gameObject.name} - Setting prismatic target: {positionGoal:F3}m (current: {currentPosition:F3}m, change: {movementChange:F3}m)");
            SetDriveTarget(positionGoal);
            
            // Immediately check some key properties
            Debug.Log($"{gameObject.name} - IMMEDIATE CHECK: Immovable={articulation.immovable}, Mass={articulation.mass:F2}, Enabled={articulation.enabled}");
            
            // Check if position actually changed after a few frames
            StartCoroutine(CheckPositionAfterDelay(currentPosition, positionGoal));
        }
    }

    System.Collections.IEnumerator CheckPositionAfterDelay(float expectedOldPos, float expectedNewPos)
    {
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate(); // Wait 2 physics frames
        
        float actualPosition = GetCurrentPosition();
        float expectedChange = expectedNewPos - expectedOldPos;
        float actualChange = actualPosition - expectedOldPos;
        
        Debug.Log($"{gameObject.name} - Position Check: Expected change={expectedChange:F3}m, Actual change={actualChange:F3}m, Current pos={actualPosition:F3}m");
        
        if (Mathf.Abs(actualChange) < 0.001f && Mathf.Abs(expectedChange) > 0.001f)
        {
            Debug.LogWarning($"{gameObject.name} - Prismatic joint not responding to drive commands! Check ArticulationBody configuration.");
            
            // Detailed diagnostics for non-responsive joint
            Debug.LogError($"{gameObject.name} - DIAGNOSTIC INFO:");
            Debug.LogError($"  Expected change: {expectedChange:F6}m, Actual change: {actualChange:F6}m");
            Debug.LogError($"  Current position: {actualPosition:F6}m, Previous: {expectedOldPos:F6}m");
            
            // Check if this is a root ArticulationBody (which can't move)
            if (articulation.isRoot)
            {
                Debug.LogError($"{gameObject.name} - This is a ROOT ArticulationBody! Root bodies cannot move. Make sure this joint has a parent ArticulationBody.");
            }
            
            // Check parent hierarchy
            Transform parent = articulation.transform.parent;
            if (parent != null)
            {
                ArticulationBody parentBody = parent.GetComponent<ArticulationBody>();
                if (parentBody == null)
                {
                    Debug.LogError($"{gameObject.name} - Parent '{parent.name}' does not have an ArticulationBody! Prismatic joints need proper ArticulationBody hierarchy.");
                }
                else if (parentBody.immovable && !parentBody.isRoot)
                {
                    Debug.LogError($"{gameObject.name} - Parent ArticulationBody '{parent.name}' is IMMOVABLE but not root! Non-root bodies should be movable.");
                }
                else if (parentBody.immovable && parentBody.isRoot)
                {
                    Debug.Log($"{gameObject.name} - Parent '{parent.name}' is root and immovable (this is correct and expected).");
                }
                else
                {
                    Debug.Log($"{gameObject.name} - Parent ArticulationBody '{parent.name}': Type={parentBody.jointType}, Immovable={parentBody.immovable}, IsRoot={parentBody.isRoot}, Mass={parentBody.mass:F2}");
                }
            }
            else
            {
                Debug.LogError($"{gameObject.name} - No parent found! ArticulationBody joints need a parent.");
            }
            
            // Check current drive configuration
            ArticulationDrive currentDrive = GetDrive(primaryDriveIndex);
            Debug.LogError($"{gameObject.name} - Drive Config: Type={currentDrive.driveType}, Target={currentDrive.target:F6}, Stiffness={currentDrive.stiffness:F1}, Damping={currentDrive.damping:F1}");
            
            // Check for common issues
            if (currentDrive.stiffness < 1000f)
            {
                Debug.LogError($"{gameObject.name} - STIFFNESS TOO LOW! ({currentDrive.stiffness:F1}) Try 10000 or higher for position control.");
            }
            
            if (articulation.mass < 0.01f)
            {
                Debug.LogError($"{gameObject.name} - MASS TOO LOW! ({articulation.mass:F3}) Very low mass can cause physics issues.");
            }
            
            if (articulation.jointFriction > 1.0f)
            {
                Debug.LogError($"{gameObject.name} - HIGH JOINT FRICTION! ({articulation.jointFriction:F2}) High friction can prevent movement.");
            }
            
            LogDriveStatus();
            DiagnosePrismaticJoint();
        }
    }

    void HandleSphericalJoint()
    {
        // For spherical joints, we can rotate around multiple axes
        // This example rotates around the primary axis, but you could extend it
        if (useVelocityControl)
        {
            // Use velocity control for continuous rotation, but check limits if enforced
            float targetVelocity = (float)rotationState * speed * Mathf.Deg2Rad;
            
            if (enforceLimits)
            {
                float currentRotation = GetCurrentRotation();
                // Stop if we're at the limit and trying to move further in that direction
                if ((currentRotation >= upperLimit && rotationState == RotationDirection.Positive) ||
                    (currentRotation <= lowerLimit && rotationState == RotationDirection.Negative))
                {
                    targetVelocity = 0f;
                }
            }
            
            SetDriveVelocity(targetVelocity);
        }
        else
        {
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float currentRotation = GetCurrentRotation();
            float rotationGoal = currentRotation + rotationChange;
            
            // Clamp the rotation goal to joint limits if enforced
            if (enforceLimits)
            {
                rotationGoal = Mathf.Clamp(rotationGoal, lowerLimit, upperLimit);
            }
            
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

    void SetupJointLimits()
    {
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        
        // Get limits from the ArticulationBody drive
        lowerLimit = drive.lowerLimit;
        upperLimit = drive.upperLimit;
        
        // Check if limits are effectively unlimited (both are 0)
        bool hasUnlimitedRange = (lowerLimit == 0f && upperLimit == 0f);
        
        if (hasUnlimitedRange)
        {
            usingDefaultLimits = false;
            enforceLimits = false; // Automatically disable limit enforcement for unlimited joints
            Debug.Log($"{gameObject.name} - Joint has unlimited range (limits are 0). Limit enforcement disabled.");
        }
        else
        {
            usingDefaultLimits = false;
            // Convert radians to degrees for revolute joints if needed (for display and comparison)
            if (jointType == ArticulationJointType.RevoluteJoint || jointType == ArticulationJointType.SphericalJoint)
            {
                lowerLimit = Mathf.Rad2Deg * lowerLimit;
                upperLimit = Mathf.Rad2Deg * upperLimit;
            }
            
            Debug.Log($"{gameObject.name} - Joint Limits: Lower={lowerLimit:F2}, Upper={upperLimit:F2} " +
                      $"({(jointType == ArticulationJointType.PrismaticJoint ? "meters" : "degrees")})");
        }
    }

    void ConfigureDriveSettings()
    {
        // Configure the appropriate drive based on the primary axis
        ArticulationDrive drive = GetDrive(primaryDriveIndex);

        Debug.Log($"{gameObject.name} - Configuring drive for {jointType} on axis {primaryDriveIndex}");

        if (useVelocityControl)
        {
            // For velocity control, we need different settings based on joint type
            drive.driveType = ArticulationDriveType.Velocity;
            drive.stiffness = 0f; // No position stiffness for velocity control
            drive.damping = damping;
            drive.forceLimit = maxForce;
            drive.target = 0f; // Clear any position target
            drive.targetVelocity = 0f; // Will be set during movement
            
            Debug.Log($"{gameObject.name} - Configured for VELOCITY control: damping={damping}, forceLimit={maxForce}");
        }
        else
        {
            // For position control, settings vary by joint type
            drive.driveType = ArticulationDriveType.Target;
            drive.targetVelocity = 0f; // Clear any velocity target
            
            if (jointType == ArticulationJointType.PrismaticJoint)
            {
                // Prismatic joints often need different stiffness/damping than revolute
                drive.stiffness = 50000f; // Higher stiffness for linear movement
                drive.damping = 1000f; // Higher damping for stability
                drive.forceLimit = maxForce * 2f; // More force for linear movement
                
                // Safely get current position, default to 0 if not available
                float currentPos = 0f;
                if (!articulation.isRoot && articulation.jointPosition.dofCount > primaryDriveIndex)
                {
                    currentPos = GetCurrentPosition();
                }
                drive.target = currentPos; // Start at current position or 0
            }
            else // Revolute or Spherical
            {
                drive.stiffness = 10000f;
                drive.damping = 500f;
                drive.forceLimit = maxForce;
                
                // Safely get current rotation, default to 0 if not available
                float currentRot = 0f;
                if (!articulation.isRoot && articulation.jointPosition.dofCount > primaryDriveIndex)
                {
                    currentRot = GetCurrentRotation();
                }
                drive.target = currentRot * Mathf.Deg2Rad; // Convert back to radians for ArticulationBody
            }
            
            Debug.Log($"{gameObject.name} - Configured for TARGET control: stiffness={drive.stiffness}, damping={drive.damping}, forceLimit={drive.forceLimit}, initialTarget={drive.target:F3}");
        }

        SetDrive(primaryDriveIndex, drive);
        
        // Verify the configuration was applied
        ArticulationDrive currentDrive = GetDrive(primaryDriveIndex);
        Debug.Log($"{gameObject.name} - Drive verification: Type={currentDrive.driveType}, Target={currentDrive.target:F3}, TargetVelocity={currentDrive.targetVelocity:F3}, Stiffness={currentDrive.stiffness:F1}");
    }

    // MOVEMENT HELPERS
    float GetCurrentRotation()
    {
        if (articulation == null)
        {
            Debug.LogWarning($"No ArticulationBody available for {gameObject.name}");
            return 0f;
        }
        
        if (articulation.isRoot)
        {
            Debug.LogWarning($"{gameObject.name} is a root ArticulationBody - no joint position data available");
            return 0f;
        }
        
        if (articulation.jointPosition.dofCount <= primaryDriveIndex)
        {
            Debug.LogWarning($"No joint position data available for {gameObject.name} at index {primaryDriveIndex}. DOF count: {articulation.jointPosition.dofCount}");
            return 0f;
        }

        float currentRotationRads = articulation.jointPosition[primaryDriveIndex];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    float GetCurrentPosition()
    {
        if (articulation == null)
        {
            Debug.LogWarning($"No ArticulationBody available for {gameObject.name}");
            return 0f;
        }
        
        if (articulation.isRoot)
        {
            Debug.LogWarning($"{gameObject.name} is a root ArticulationBody - no joint position data available");
            return 0f;
        }
        
        if (articulation.jointPosition.dofCount <= primaryDriveIndex)
        {
            Debug.LogWarning($"No joint position data available for {gameObject.name} at index {primaryDriveIndex}. DOF count: {articulation.jointPosition.dofCount}");
            return 0f;
        }

        return articulation.jointPosition[primaryDriveIndex]; // Already in meters for prismatic
    }

    void SetDriveTarget(float target)
    {
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        float oldTarget = drive.target;
        
        // For revolute joints, target needs to be in radians
        if (jointType == ArticulationJointType.RevoluteJoint || jointType == ArticulationJointType.SphericalJoint)
        {
            drive.target = target * Mathf.Deg2Rad; // Convert degrees to radians
            Debug.Log($"{gameObject.name} - Setting revolute target: {target:F3}° ({drive.target:F6} rad), was {oldTarget * Mathf.Rad2Deg:F3}°");
        }
        else // Prismatic joint
        {
            drive.target = target; // Already in meters
            Debug.Log($"{gameObject.name} - Setting prismatic target: {target:F3}m, was {oldTarget:F3}m");
        }
        
        SetDrive(primaryDriveIndex, drive);
        
        // Verify the drive was actually set
        ArticulationDrive verifyDrive = GetDrive(primaryDriveIndex);
        Debug.Log($"{gameObject.name} - Drive verified: Target={verifyDrive.target:F6}, DriveType={verifyDrive.driveType}, Stiffness={verifyDrive.stiffness:F1}");
        Debug.Log($"{gameObject.name} - ArticulationBody: Mass={articulation.mass:F2}, Friction={articulation.jointFriction:F3}, Gravity={articulation.useGravity}");
    }

    void SetDriveVelocity(float targetVelocity)
    {
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        float oldVelocity = drive.targetVelocity;
        
        // Velocity is already in correct units (rad/s for revolute, m/s for prismatic)
        drive.targetVelocity = targetVelocity;
        drive.target = 0f; // Clear position target when using velocity control
        
        SetDrive(primaryDriveIndex, drive);
        
        string units = (jointType == ArticulationJointType.PrismaticJoint) ? "m/s" : "rad/s";
        Debug.Log($"{gameObject.name} - Drive velocity: {oldVelocity:F3} -> {targetVelocity:F3} {units} (DriveType: {drive.driveType})");
        
        // Verify
        ArticulationDrive verifyDrive = GetDrive(primaryDriveIndex);
        Debug.Log($"{gameObject.name} - Velocity verified: {verifyDrive.targetVelocity:F3} {units}, Target cleared: {verifyDrive.target:F6}");
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
        Debug.Log($"{gameObject.name} - Rotation direction changed from {rotationState} to {direction}");
        rotationState = direction;
    }

    public void SetSpeed(float newSpeed)
    {
        Debug.Log($"{gameObject.name} - Speed changed from {speed} to {newSpeed}");
        speed = newSpeed;
    }

    public void SetControlMode(bool velocityControl)
    {
        Debug.Log($"{gameObject.name} - Control mode changed from {(useVelocityControl ? "Velocity" : "Position")} to {(velocityControl ? "Velocity" : "Position")}");
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

    public void GetJointLimits(out float lower, out float upper)
    {
        lower = lowerLimit;
        upper = upperLimit;
    }

    public bool IsAtLimit()
    {
        if (!enforceLimits) return false;
        
        float currentValue = GetCurrentValue();
        float tolerance = 0.01f; // Small tolerance for floating point comparison
        
        return Mathf.Abs(currentValue - lowerLimit) < tolerance || 
               Mathf.Abs(currentValue - upperLimit) < tolerance;
    }

    public bool IsUsingDefaultLimits()
    {
        return usingDefaultLimits;
    }

    public void LogDriveStatus()
    {
        if (articulation == null) return;
        
        ArticulationDrive drive = GetDrive(primaryDriveIndex);
        Debug.Log($"{gameObject.name} - Drive Status:");
        Debug.Log($"  DriveType: {drive.driveType}");
        Debug.Log($"  Target: {drive.target:F3}");
        Debug.Log($"  TargetVelocity: {drive.targetVelocity:F3}");
        Debug.Log($"  Stiffness: {drive.stiffness:F1}");
        Debug.Log($"  Damping: {drive.damping:F1}");
        Debug.Log($"  ForceLimit: {drive.forceLimit:F1}");
        Debug.Log($"  Current Position: {GetCurrentValue():F3}");
        Debug.Log($"  Joint Type: {jointType}");
        Debug.Log($"  Primary Drive Index: {primaryDriveIndex}");
    }

    public void DiagnosePrismaticJoint()
    {
        if (articulation == null || jointType != ArticulationJointType.PrismaticJoint) return;
        
        Debug.Log($"{gameObject.name} - PRISMATIC JOINT DIAGNOSIS:");
        Debug.Log($"  ArticulationBody Enabled: {articulation.enabled}");
        Debug.Log($"  Mass: {articulation.mass:F2}");
        Debug.Log($"  Immovable: {articulation.immovable}");
        Debug.Log($"  Use Gravity: {articulation.useGravity}");
        Debug.Log($"  Joint Type: {articulation.jointType}");
        Debug.Log($"  DOF Count: {articulation.dofCount}");
        Debug.Log($"  Joint Position Count: {articulation.jointPosition.dofCount}");
        Debug.Log($"  Joint Velocity Count: {articulation.jointVelocity.dofCount}");
        
        // Check all three drives
        Debug.Log($"  X Drive: Type={articulation.xDrive.driveType}, Target={articulation.xDrive.target:F3}, Stiffness={articulation.xDrive.stiffness:F1}");
        Debug.Log($"  Y Drive: Type={articulation.yDrive.driveType}, Target={articulation.yDrive.target:F3}, Stiffness={articulation.yDrive.stiffness:F1}");
        Debug.Log($"  Z Drive: Type={articulation.zDrive.driveType}, Target={articulation.zDrive.target:F3}, Stiffness={articulation.zDrive.stiffness:F1}");
        
        // Check parent
        if (articulation.isRoot)
        {
            Debug.Log($"  Is Root: true");
        }
        else
        {
            Debug.Log($"  Parent: {(articulation.transform.parent != null ? articulation.transform.parent.name : "null")}");
        }
    }

    [ContextMenu("Diagnose Joint")]
    public void DiagnoseJointFromMenu()
    {
        LogDriveStatus();
        if (jointType == ArticulationJointType.PrismaticJoint)
        {
            DiagnosePrismaticJoint();
        }
    }

    [ContextMenu("Check Physics Settings")]
    public void CheckPhysicsSettings()
    {
        Debug.Log($"{gameObject.name} - PHYSICS SETTINGS CHECK:");
        Debug.Log($"  Time.fixedDeltaTime: {Time.fixedDeltaTime:F4}s ({1f/Time.fixedDeltaTime:F1} Hz)");
        Debug.Log($"  Physics.defaultSolverIterations: {Physics.defaultSolverIterations}");
        Debug.Log($"  Physics.defaultSolverVelocityIterations: {Physics.defaultSolverVelocityIterations}");
        
        // Check for ArticulationBody-specific issues
        if (articulation != null)
        {
            Debug.Log($"  ArticulationBody.solverIterations: {articulation.solverIterations}");
            Debug.Log($"  ArticulationBody.solverVelocityIterations: {articulation.solverVelocityIterations}");
            Debug.Log($"  ArticulationBody.sleepThreshold: {articulation.sleepThreshold:F4}");
            
            // Check if body is sleeping
            if (articulation.IsSleeping())
            {
                Debug.LogWarning($"  ArticulationBody is SLEEPING! Call WakeUp() to activate it.");
                articulation.WakeUp();
            }
            else
            {
                Debug.Log($"  ArticulationBody is AWAKE");
            }
        }
    }

    [ContextMenu("Force Wake Up")]
    public void ForceWakeUp()
    {
        if (articulation != null)
        {
            articulation.WakeUp();
            Debug.Log($"{gameObject.name} - Forced ArticulationBody wake up");
        }
    }

    [ContextMenu("Test Movement - Slow")]
    public void TestMovementSlow()
    {
        Debug.Log($"{gameObject.name} - Testing slow movement (300°/s)");
        speed = 300f;
        rotationState = RotationDirection.Positive;
    }

    [ContextMenu("Test Movement - Fast")]
    public void TestMovementFast()
    {
        Debug.Log($"{gameObject.name} - Testing fast movement (1800°/s)");
        speed = 1800f;
        rotationState = RotationDirection.Positive;
    }

    [ContextMenu("Test Movement - Very Fast")]
    public void TestMovementVeryFast()
    {
        Debug.Log($"{gameObject.name} - Testing very fast movement (3600°/s)");
        speed = 3600f;
        rotationState = RotationDirection.Positive;
    }

    [ContextMenu("Stop Movement")]
    public void StopMovement()
    {
        Debug.Log($"{gameObject.name} - Stopping movement");
        rotationState = RotationDirection.None;
    }

    [ContextMenu("Explain Drive System")]
    public void ExplainDriveSystem()
    {
        Debug.Log($"{gameObject.name} - UNITY ARTICULATION DRIVE SYSTEM EXPLANATION:");
        Debug.Log("=== Drive Types ===");
        Debug.Log("Target: Uses 'target' property for position/angle control");
        Debug.Log("Velocity: Uses 'targetVelocity' property for speed control");
        Debug.Log("Force: Uses 'targetValue' property for direct force");
        Debug.Log("Acceleration: Uses 'targetValue' property for acceleration");
        Debug.Log("");
        Debug.Log("=== Current Configuration ===");
        Debug.Log($"Joint Type: {jointType}");
        Debug.Log($"Control Mode: {(useVelocityControl ? "Velocity" : "Position")}");
        Debug.Log($"Primary Drive Index: {primaryDriveIndex} ({(primaryDriveIndex == 0 ? "X" : primaryDriveIndex == 1 ? "Y" : "Z")})");
        Debug.Log("");
        
        ArticulationDrive currentDrive = GetDrive(primaryDriveIndex);
        Debug.Log($"=== Drive Settings (Axis {primaryDriveIndex}) ===");
        Debug.Log($"DriveType: {currentDrive.driveType}");
        Debug.Log($"Target: {currentDrive.target:F6}");
        Debug.Log($"TargetVelocity: {currentDrive.targetVelocity:F6}");
        Debug.Log($"Stiffness: {currentDrive.stiffness:F1}");
        Debug.Log($"Damping: {currentDrive.damping:F1}");
        Debug.Log($"ForceLimit: {currentDrive.forceLimit:F1}");
        Debug.Log($"LowerLimit: {currentDrive.lowerLimit:F6}");
        Debug.Log($"UpperLimit: {currentDrive.upperLimit:F6}");
        Debug.Log("");
        
        Debug.Log("=== Expected Behavior ===");
        if (currentDrive.driveType == ArticulationDriveType.Target)
        {
            Debug.Log("TARGET mode: Joint should move towards the 'target' value");
            Debug.Log($"Current target: {currentDrive.target:F6}");
            Debug.Log($"Current position: {(jointType == ArticulationJointType.PrismaticJoint ? GetCurrentPosition() : GetCurrentRotation() * Mathf.Deg2Rad):F6}");
        }
        else if (currentDrive.driveType == ArticulationDriveType.Velocity)
        {
            Debug.Log("VELOCITY mode: Joint should move at 'targetVelocity' speed");
            Debug.Log($"Current targetVelocity: {currentDrive.targetVelocity:F6}");
        }
    }
    public void QuickPrismaticCheck()
    {
        if (articulation == null)
        {
            Debug.LogError($"{gameObject.name} - No ArticulationBody found!");
            return;
        }

        Debug.Log($"{gameObject.name} - QUICK CHECK:");
        Debug.Log($"  Immovable: {articulation.immovable}");
        Debug.Log($"  Mass: {articulation.mass}");
        Debug.Log($"  Enabled: {articulation.enabled}");
        Debug.Log($"  Joint Type: {articulation.jointType}");
        Debug.Log($"  Primary Drive Index: {primaryDriveIndex}");
        
        ArticulationDrive currentDrive = GetDrive(primaryDriveIndex);
        Debug.Log($"  Current Drive - Type: {currentDrive.driveType}, Target: {currentDrive.target}, Stiffness: {currentDrive.stiffness}");
        
        Debug.Log($"  Current Position: {GetCurrentPosition()}");
        Debug.Log($"  Enforce Limits: {enforceLimits}");
    }
}