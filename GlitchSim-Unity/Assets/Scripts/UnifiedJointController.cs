using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Advanced Unified Joint Controller for Unity Articulation Bodies
/// 
/// This controller automatically detects the type and drive configuration of articulation bodies
/// and applies the appropriate control method based on the detected settings:
/// 
/// Supported Joint Types:
/// - RevoluteJoint (rotational)
/// - PrismaticJoint (linear)
/// - SphericalJoint (3-axis rotation, X-axis only for now)
/// - FixedJoint (warning only, cannot be actuated)
/// 
/// Supported Drive Types:
/// - Target: Position/rotation control with target values
/// - Velocity: Direct velocity control
/// - Force: Direct force/torque application
/// 
/// Features:
/// - Automatic joint type and drive detection
/// - Individual joint speed and deadzone control
/// - Optional custom joint limits (override Unity limits)
/// - Two movement modes:
///   * Incremental: Smooth continuous movement (default)
///   * Drive to Limits: Input drives directly to joint extremes
/// - Proper limit checking and enforcement
/// - Simultaneous or sequential joint movement
/// - Comprehensive debug logging
/// - Runtime joint inspection and verification utilities
/// 
/// For Target Drive Prismatic Joints:
/// - Automatically detects and respects Unity joint upper/lower limits
/// - Option to use custom limits instead
/// - Can drive incrementally or directly to limits based on input direction
/// - Provides detailed logging of current position vs limits
/// </summary>

public enum JointMovementState { Fixed = 0, Positive = 1, Negative = -1 };

public class UnifiedJointController : MonoBehaviour
{
    [System.Serializable]
    public struct JointConfig
    {
        [Header("Input Configuration")]
        public string inputAxis;
        public GameObject jointObject;
        
        [Header("Control Parameters")]
        public float speed;
        public float inputDeadzone;
        
        [Header("Movement Mode")]
        public bool driveToLimits;           // If true, input drives directly to limits instead of incremental movement
        public bool forceTargetBehavior;     // If true, always use target-based control regardless of detected drive type
        
        [Header("Joint Limits (Optional)")]
        public bool useCustomLimits;
        public float minLimit;
        public float maxLimit;
        
        [Header("Runtime Info (Read-Only)")]
        [SerializeField] private ArticulationJointType detectedJointType;
        [SerializeField] private ArticulationDriveType detectedDriveType;
        [SerializeField] private string jointInfo;
        
        // Properties to access private fields
        public ArticulationJointType DetectedJointType => detectedJointType;
        public ArticulationDriveType DetectedDriveType => detectedDriveType;
        public string JointInfo => jointInfo;
        
        // Methods to set private fields (called by controller)
        public void SetDetectedInfo(ArticulationJointType jointType, ArticulationDriveType driveType, string info)
        {
            detectedJointType = jointType;
            detectedDriveType = driveType;
            jointInfo = info;
        }
    }
    
    [Header("Global Settings")]
    public bool allowSimultaneousMovement = true;
    public bool enableDebugLogging = true;
    
    [Header("Joint Configuration")]
    public JointConfig[] joints;
    
    private Dictionary<int, ArticulationBody> articulationBodies = new Dictionary<int, ArticulationBody>();
    
    void Start()
    {
        InitializeJoints();
    }
    
    void InitializeJoints()
    {
        if (joints == null || joints.Length == 0)
        {
            Debug.LogWarning("No joints configured in UnifiedJointController");
            return;
        }
        
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i].jointObject == null)
            {
                Debug.LogError($"Joint {i}: jointObject is null");
                continue;
            }
            
            ArticulationBody articulation = joints[i].jointObject.GetComponent<ArticulationBody>();
            if (articulation == null)
            {
                Debug.LogError($"Joint {i} ({joints[i].jointObject.name}): No ArticulationBody component found");
                continue;
            }
            
            // Cache the articulation body
            articulationBodies[i] = articulation;
            
            // Detect joint type and drive configuration
            DetectAndConfigureJoint(i, articulation);
            
            // Set default deadzone if not specified
            if (joints[i].inputDeadzone <= 0)
            {
                var jointConfig = joints[i];
                jointConfig.inputDeadzone = 0.01f;
                joints[i] = jointConfig;
            }
            
            if (enableDebugLogging)
            {
                Debug.Log($"Joint {i} initialized: {joints[i].JointInfo}");
            }
        }
    }
    
    void DetectAndConfigureJoint(int jointIndex, ArticulationBody articulation)
    {
        var jointConfig = joints[jointIndex];
        
        ArticulationJointType jointType = articulation.jointType;
        
        // More thorough drive type detection for ArticulationBody
        ArticulationDriveType driveType = ArticulationDriveType.Target; // Default to Target
        string driveInfo = "";
        
        // Check which drives are actually configured and active
        var xDrive = articulation.xDrive;
        var yDrive = articulation.yDrive;
        var zDrive = articulation.zDrive;
        
        switch (jointType)
        {
            case ArticulationJointType.PrismaticJoint:
                // For prismatic joints, check which axis is actually the prismatic axis
                // Usually it's X, but could be Y or Z depending on configuration
                if (articulation.linearLockX == ArticulationDofLock.LimitedMotion)
                {
                    driveType = xDrive.driveType;
                    driveInfo = $"X-axis: {driveType}";
                }
                else if (articulation.linearLockY == ArticulationDofLock.LimitedMotion)
                {
                    driveType = yDrive.driveType;
                    driveInfo = $"Y-axis: {driveType}";
                }
                else if (articulation.linearLockZ == ArticulationDofLock.LimitedMotion)
                {
                    driveType = zDrive.driveType;
                    driveInfo = $"Z-axis: {driveType}";
                }
                else
                {
                    // Fallback to X-drive if no specific axis is found
                    driveType = xDrive.driveType;
                    driveInfo = $"X-axis (fallback): {driveType}";
                    Debug.LogWarning($"Prismatic joint {articulation.name}: No clearly defined prismatic axis found, using X-axis");
                }
                break;
                
            case ArticulationJointType.RevoluteJoint:
                // For revolute joints, check which axis is the rotation axis
                if (articulation.twistLock == ArticulationDofLock.LimitedMotion)
                {
                    driveType = xDrive.driveType;
                    driveInfo = $"Twist (X): {driveType}";
                }
                else if (articulation.swingYLock == ArticulationDofLock.LimitedMotion)
                {
                    driveType = yDrive.driveType;
                    driveInfo = $"Swing Y: {driveType}";
                }
                else if (articulation.swingZLock == ArticulationDofLock.LimitedMotion)
                {
                    driveType = zDrive.driveType;
                    driveInfo = $"Swing Z: {driveType}";
                }
                else
                {
                    // Fallback to X-drive
                    driveType = xDrive.driveType;
                    driveInfo = $"Twist (X, fallback): {driveType}";
                    Debug.LogWarning($"Revolute joint {articulation.name}: No clearly defined rotation axis found, using twist (X-axis)");
                }
                break;
                
            case ArticulationJointType.SphericalJoint:
                // For spherical joints, typically use X-drive but note it's multi-axis
                driveType = xDrive.driveType;
                driveInfo = $"Multi-axis (X primary): {driveType}";
                break;
                
            case ArticulationJointType.FixedJoint:
                driveType = ArticulationDriveType.Target; // Fixed joints don't move, but use Target as default
                driveInfo = "Fixed (no drive)";
                break;
                
            default:
                driveType = xDrive.driveType;
                driveInfo = $"Unknown joint, X-drive: {driveType}";
                break;
        }
        
        string jointInfo = $"{articulation.name} - Type: {jointType}, Drive: {driveInfo}";
        
        // Add lock information for debugging
        if (enableDebugLogging)
        {
            Debug.Log($"[JOINT DETECTION] {articulation.name}:");
            Debug.Log($"  Joint Type: {jointType}");
            Debug.Log($"  DOF Count: {articulation.dofCount}");
            Debug.Log($"  Joint Position DOF Count: {articulation.jointPosition.dofCount}");
            Debug.Log($"  Linear Locks - X: {articulation.linearLockX}, Y: {articulation.linearLockY}, Z: {articulation.linearLockZ}");
            Debug.Log($"  Angular Locks - Twist: {articulation.twistLock}, SwingY: {articulation.swingYLock}, SwingZ: {articulation.swingZLock}");
            Debug.Log($"  Drive Types - X: {xDrive.driveType}, Y: {yDrive.driveType}, Z: {zDrive.driveType}");
            Debug.Log($"  Selected Drive: {driveInfo}");
        }
        
        
        // Add additional info based on joint type
        switch (jointType)
        {
            case ArticulationJointType.RevoluteJoint:
                var activeDrive = GetActiveDriveForRevolute(articulation);
                float minDeg = Mathf.Rad2Deg * activeDrive.lowerLimit;
                float maxDeg = Mathf.Rad2Deg * activeDrive.upperLimit;
                jointInfo += $", Limits: [{minDeg:F1}°, {maxDeg:F1}°]";
                break;
            case ArticulationJointType.PrismaticJoint:
                var activePrismaticDrive = GetActiveDriveForPrismatic(articulation);
                jointInfo += $", Limits: [{activePrismaticDrive.lowerLimit:F3}m, {activePrismaticDrive.upperLimit:F3}m]";
                break;
            case ArticulationJointType.SphericalJoint:
                jointInfo += " (Multi-axis)";
                break;
        }
        
        jointConfig.SetDetectedInfo(jointType, driveType, jointInfo);
        joints[jointIndex] = jointConfig;
    }
    
    // Helper method to get the active drive for revolute joints
    ArticulationDrive GetActiveDriveForRevolute(ArticulationBody articulation)
    {
        if (articulation.twistLock == ArticulationDofLock.LimitedMotion)
            return articulation.xDrive;
        else if (articulation.swingYLock == ArticulationDofLock.LimitedMotion)
            return articulation.yDrive;
        else if (articulation.swingZLock == ArticulationDofLock.LimitedMotion)
            return articulation.zDrive;
        else
            return articulation.xDrive; // fallback
    }
    
    // Helper method to get the active drive for prismatic joints
    ArticulationDrive GetActiveDriveForPrismatic(ArticulationBody articulation)
    {
        if (articulation.linearLockX == ArticulationDofLock.LimitedMotion)
            return articulation.xDrive;
        else if (articulation.linearLockY == ArticulationDofLock.LimitedMotion)
            return articulation.yDrive;
        else if (articulation.linearLockZ == ArticulationDofLock.LimitedMotion)
            return articulation.zDrive;
        else
            return articulation.xDrive; // fallback
    }
    
    void Update()
    {
        // Read input for each joint and update movement state
        bool anyInputDetected = false;
        
        for (int i = 0; i < joints.Length; i++)
        {
            float inputVal = Input.GetAxis(joints[i].inputAxis);
            JointMovementState movementState = GetMovementStateFromInput(inputVal, joints[i].inputDeadzone);
            
            if (Mathf.Abs(inputVal) > joints[i].inputDeadzone)
            {
                if (enableDebugLogging)
                {
                    Debug.Log($"[INPUT] Joint {i} ({joints[i].jointObject.name}): Input '{joints[i].inputAxis}' = {inputVal:F3} -> Movement: {movementState}");
                }
            }
            
            if (movementState != JointMovementState.Fixed)
            {
                anyInputDetected = true;
                if (enableDebugLogging)
                {
                    Debug.Log($"[JOINT CONTROL] Driving Joint {i} ({joints[i].jointObject.name}) - Type: {joints[i].DetectedJointType}, Drive: {joints[i].DetectedDriveType}, Movement: {movementState}");
                }
                UpdateJoint(i, movementState);
                
                if (!allowSimultaneousMovement)
                {
                    return; // Only move one joint at a time if simultaneous movement is disabled
                }
            }
        }
        
        // Stop all joints if no input (only if simultaneous movement is enabled)
        if (!anyInputDetected && allowSimultaneousMovement)
        {
            StopAllJoints();
        }
    }
    
    void UpdateJoint(int jointIndex, JointMovementState movementState)
    {
        if (!articulationBodies.ContainsKey(jointIndex))
        {
            Debug.LogError($"Joint {jointIndex}: ArticulationBody not cached. Make sure Start() was called.");
            return;
        }
        
        JointConfig joint = joints[jointIndex];
        ArticulationBody articulation = articulationBodies[jointIndex];
        float inputValue = (float)movementState;
        
        if (enableDebugLogging)
        {
            Debug.Log($"[JOINT UPDATE] {joint.jointObject.name}: Type={joint.DetectedJointType}, Drive={joint.DetectedDriveType}, Input={inputValue:F2}");
        }
        
        // Apply movement based on joint type and drive type
        switch (joint.DetectedJointType)
        {
            case ArticulationJointType.RevoluteJoint:
                UpdateRevoluteJoint(articulation, movementState, joint);
                break;
            case ArticulationJointType.PrismaticJoint:
                UpdatePrismaticJoint(articulation, movementState, joint);
                break;
            case ArticulationJointType.SphericalJoint:
                UpdateSphericalJoint(articulation, movementState, joint);
                break;
            case ArticulationJointType.FixedJoint:
                Debug.LogWarning($"Joint {jointIndex}: Attempting to move a FixedJoint. This joint cannot be actuated.");
                break;
            default:
                Debug.LogError($"Joint {jointIndex}: Unsupported joint type {joint.DetectedJointType}");
                break;
        }
    }
    
    void UpdateRevoluteJoint(ArticulationBody articulation, JointMovementState movementState, JointConfig joint)
    {
        var drive = articulation.xDrive;
        float inputValue = (float)movementState;
        
        // Check if we should override the drive type behavior
        ArticulationDriveType effectiveDriveType = joint.forceTargetBehavior ? ArticulationDriveType.Target : joint.DetectedDriveType;
        
        switch (effectiveDriveType)
        {
            case ArticulationDriveType.Target:
                UpdateTargetDriveRevolute(articulation, drive, inputValue, joint);
                break;
            case ArticulationDriveType.Velocity:
                UpdateVelocityDriveRevolute(articulation, drive, inputValue, joint);
                break;
            case ArticulationDriveType.Force:
                UpdateForceDriveRevolute(articulation, drive, inputValue, joint);
                break;
            default:
                Debug.LogWarning($"Unsupported drive type {joint.DetectedDriveType} for revolute joint {joint.jointObject.name}");
                break;
        }
    }
    
    void UpdateTargetDriveRevolute(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Convert current position from radians to degrees
        float currentRotationRads = GetSafeJointPosition(articulation, 0);
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        
        // Determine the effective limits (custom limits override Unity limits)
        float effectiveMinLimit, effectiveMaxLimit;
        if (joint.useCustomLimits)
        {
            effectiveMinLimit = joint.minLimit;
            effectiveMaxLimit = joint.maxLimit;
        }
        else
        {
            // Use the Unity ArticulationBody joint limits (convert from radians to degrees)
            effectiveMinLimit = Mathf.Rad2Deg * drive.lowerLimit;
            effectiveMaxLimit = Mathf.Rad2Deg * drive.upperLimit;
        }
        
        float rotationGoal;
        
        if (joint.driveToLimits)
        {
            // Drive directly to limits based on input direction
            if (inputValue > 0)
            {
                rotationGoal = effectiveMaxLimit; // Drive to upper limit
            }
            else if (inputValue < 0)
            {
                rotationGoal = effectiveMinLimit; // Drive to lower limit
            }
            else
            {
                rotationGoal = currentRotation; // Stay at current position
            }
            
            if (enableDebugLogging)
            {
                Debug.Log($"[REVOLUTE TARGET DRIVE] {joint.jointObject.name}: Driving to limits - Current: {currentRotation:F2}°, Target: {rotationGoal:F2}°, Limits: [{effectiveMinLimit:F2}°, {effectiveMaxLimit:F2}°]");
            }
        }
        else
        {
            // Incremental movement (original behavior)
            float rotationChange = inputValue * joint.speed * Time.deltaTime;
            rotationGoal = currentRotation + rotationChange;
            
            // Clamp to limits
            rotationGoal = Mathf.Clamp(rotationGoal, effectiveMinLimit, effectiveMaxLimit);
            
            if (enableDebugLogging)
            {
                Debug.Log($"[REVOLUTE TARGET DRIVE] {joint.jointObject.name}: Incremental movement - Current: {currentRotation:F2}°, Change: {rotationChange:F2}°, Target: {rotationGoal:F2}°, Limits: [{effectiveMinLimit:F2}°, {effectiveMaxLimit:F2}°]");
            }
        }
        
        // Check if we're at the limits and warn if trying to go beyond
        if (enableDebugLogging)
        {
            if (rotationGoal <= effectiveMinLimit && inputValue < 0)
            {
                Debug.Log($"[REVOLUTE TARGET DRIVE] {joint.jointObject.name}: At minimum limit ({effectiveMinLimit:F2}°)");
            }
            else if (rotationGoal >= effectiveMaxLimit && inputValue > 0)
            {
                Debug.Log($"[REVOLUTE TARGET DRIVE] {joint.jointObject.name}: At maximum limit ({effectiveMaxLimit:F2}°)");
            }
        }
        
        drive.target = rotationGoal;
        articulation.xDrive = drive;
    }
    
    void UpdateVelocityDriveRevolute(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Set target velocity directly
        float targetVelocity = inputValue * joint.speed;
        
        if (enableDebugLogging)
        {
            Debug.Log($"[REVOLUTE VELOCITY DRIVE] {joint.jointObject.name}: Setting target velocity to {targetVelocity:F2}°/s (input: {inputValue:F2}, speed: {joint.speed})");
        }
        
        drive.targetVelocity = targetVelocity;
        articulation.xDrive = drive;
    }
    
    void UpdateForceDriveRevolute(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Apply torque directly for Force drive type
        float targetTorque = inputValue * joint.speed;
        
        if (enableDebugLogging)
        {
            Debug.Log($"[REVOLUTE FORCE DRIVE] {joint.jointObject.name}: Applying torque {targetTorque:F2} N⋅m (input: {inputValue:F2}, speed: {joint.speed})");
        }
        
        // For ArticulationBody Force drive, Unity might expect us to set the target as the torque value
        drive.target = targetTorque;
        articulation.xDrive = drive;
        
        // Additional torque application for more responsive behavior
        if (Mathf.Abs(targetTorque) > 0.01f)
        {
            // Scale down the external torque since we're also using the drive
            float externalTorque = targetTorque * 0.1f; // 10% of drive torque as external boost
            Vector3 torqueVector = articulation.transform.right * externalTorque;
            articulation.AddTorque(torqueVector, ForceMode.Force);
            
            if (enableDebugLogging)
            {
                Debug.Log($"[REVOLUTE FORCE DRIVE] {joint.jointObject.name}: Drive torque: {targetTorque:F2}N⋅m, External boost: {externalTorque:F2}N⋅m");
            }
        }
    }
    
    void UpdatePrismaticJoint(ArticulationBody articulation, JointMovementState movementState, JointConfig joint)
    {
        // Get the correct drive for this prismatic joint
        ArticulationDrive activeDrive = GetActiveDriveForPrismatic(articulation);
        float inputValue = (float)movementState;
        
        // Debug: Log the actual detected drive type vs what Unity reports
        if (enableDebugLogging)
        {
            Debug.Log($"[PRISMATIC JOINT DEBUG] {joint.jointObject.name}:");
            Debug.Log($"  Detected DriveType: {joint.DetectedDriveType}");
            Debug.Log($"  Active Drive Type: {activeDrive.driveType}");
            Debug.Log($"  Linear Locks - X: {articulation.linearLockX}, Y: {articulation.linearLockY}, Z: {articulation.linearLockZ}");
            Debug.Log($"  forceTargetBehavior: {joint.forceTargetBehavior}");
        }
        
        // Check if we should override the drive type behavior
        ArticulationDriveType effectiveDriveType = joint.forceTargetBehavior ? ArticulationDriveType.Target : joint.DetectedDriveType;
        
        if (enableDebugLogging && joint.forceTargetBehavior && joint.DetectedDriveType != ArticulationDriveType.Target)
        {
            Debug.Log($"[PRISMATIC JOINT] {joint.jointObject.name}: Overriding {joint.DetectedDriveType} drive to use Target behavior");
        }
        
        switch (effectiveDriveType)
        {
            case ArticulationDriveType.Target:
                UpdateTargetDrivePrismatic(articulation, activeDrive, inputValue, joint);
                break;
            case ArticulationDriveType.Velocity:
                UpdateVelocityDrivePrismatic(articulation, activeDrive, inputValue, joint);
                break;
            case ArticulationDriveType.Force:
                UpdateForceDrivePrismatic(articulation, activeDrive, inputValue, joint);
                break;
            default:
                Debug.LogWarning($"[PRISMATIC JOINT] {joint.jointObject.name}: Unsupported drive type {joint.DetectedDriveType}");
                break;
        }
    }
    
    void UpdateTargetDrivePrismatic(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Get current position from the correct axis
        float currentPosition = GetPrismaticJointPosition(articulation);
        
        // Determine the effective limits (custom limits override Unity limits)
        float effectiveMinLimit, effectiveMaxLimit;
        if (joint.useCustomLimits)
        {
            effectiveMinLimit = joint.minLimit;
            effectiveMaxLimit = joint.maxLimit;
        }
        else
        {
            // Use the Unity ArticulationBody joint limits
            effectiveMinLimit = drive.lowerLimit;
            effectiveMaxLimit = drive.upperLimit;
        }
        
        float positionGoal;
        
        if (joint.driveToLimits)
        {
            // Drive directly to limits based on input direction
            if (inputValue > 0)
            {
                positionGoal = effectiveMaxLimit; // Drive to upper limit
            }
            else if (inputValue < 0)
            {
                positionGoal = effectiveMinLimit; // Drive to lower limit
            }
            else
            {
                positionGoal = currentPosition; // Stay at current position
            }
            
            if (enableDebugLogging)
            {
                Debug.Log($"[PRISMATIC TARGET DRIVE] {joint.jointObject.name}: Driving to limits - Current: {currentPosition:F3}m, Target: {positionGoal:F3}m, Limits: [{effectiveMinLimit:F3}m, {effectiveMaxLimit:F3}m]");
            }
        }
        else
        {
            // Incremental movement (original behavior)
            float positionChange = inputValue * joint.speed * Time.deltaTime;
            positionGoal = currentPosition + positionChange;
            
            // Clamp to limits
            positionGoal = Mathf.Clamp(positionGoal, effectiveMinLimit, effectiveMaxLimit);
            
            // Safety check for unreasonable position changes
            if (Mathf.Abs(positionChange) > 1.0f && enableDebugLogging)
            {
                Debug.LogWarning($"[PRISMATIC TARGET DRIVE] {joint.jointObject.name}: Large position change ({positionChange:F3}m). Consider reducing speed ({joint.speed})");
            }
            
            if (enableDebugLogging)
            {
                Debug.Log($"[PRISMATIC TARGET DRIVE] {joint.jointObject.name}: Incremental movement - Current: {currentPosition:F3}m, Change: {positionChange:F3}m, Target: {positionGoal:F3}m, Limits: [{effectiveMinLimit:F3}m, {effectiveMaxLimit:F3}m]");
            }
        }
        
        // Check if we're at the limits and warn if trying to go beyond
        if (enableDebugLogging)
        {
            if (positionGoal <= effectiveMinLimit && inputValue < 0)
            {
                Debug.Log($"[PRISMATIC TARGET DRIVE] {joint.jointObject.name}: At minimum limit ({effectiveMinLimit:F3}m)");
            }
            else if (positionGoal >= effectiveMaxLimit && inputValue > 0)
            {
                Debug.Log($"[PRISMATIC TARGET DRIVE] {joint.jointObject.name}: At maximum limit ({effectiveMaxLimit:F3}m)");
            }
        }
        
        // Apply the drive to the correct axis
        drive.target = positionGoal;
        SetPrismaticDrive(articulation, drive);
    }
    
    // Helper method to get the current position from the correct prismatic axis
    float GetPrismaticJointPosition(ArticulationBody articulation)
    {
        // For ArticulationBody, jointPosition[0] represents the position of the primary DOF
        // regardless of which axis (X, Y, or Z) is actually the prismatic axis
        // The jointPosition array typically only has one element for single-DOF joints
        if (articulation.jointPosition.dofCount > 0)
        {
            return articulation.jointPosition[0];
        }
        else
        {
            if (enableDebugLogging)
            {
                Debug.LogWarning($"[PRISMATIC JOINT] {articulation.name}: No DOF found in jointPosition, returning 0");
            }
            return 0f;
        }
    }
    
    // Helper method to safely get joint position for any joint type
    float GetSafeJointPosition(ArticulationBody articulation, int index = 0)
    {
        if (articulation.jointPosition.dofCount > index)
        {
            return articulation.jointPosition[index];
        }
        else
        {
            if (enableDebugLogging)
            {
                Debug.LogWarning($"[JOINT] {articulation.name}: DOF index {index} out of range (dofCount: {articulation.jointPosition.dofCount}), returning 0");
            }
            return 0f;
        }
    }
    
    // Helper method to set the drive on the correct prismatic axis
    void SetPrismaticDrive(ArticulationBody articulation, ArticulationDrive drive)
    {
        if (articulation.linearLockX == ArticulationDofLock.LimitedMotion)
            articulation.xDrive = drive;
        else if (articulation.linearLockY == ArticulationDofLock.LimitedMotion)
            articulation.yDrive = drive;
        else if (articulation.linearLockZ == ArticulationDofLock.LimitedMotion)
            articulation.zDrive = drive;
        else
            articulation.xDrive = drive; // fallback
    }
    
    void UpdateVelocityDrivePrismatic(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Set target velocity directly
        float targetVelocity = inputValue * joint.speed;
        
        if (enableDebugLogging)
        {
            Debug.Log($"[PRISMATIC VELOCITY DRIVE] {joint.jointObject.name}: Setting target velocity to {targetVelocity:F3} m/s (input: {inputValue:F2}, speed: {joint.speed})");
        }
        
        drive.targetVelocity = targetVelocity;
        SetPrismaticDrive(articulation, drive);
    }
    
    void UpdateForceDrivePrismatic(ArticulationBody articulation, ArticulationDrive drive, float inputValue, JointConfig joint)
    {
        // Apply force directly for Force drive type
        float targetForce = inputValue * joint.speed;
        
        if (enableDebugLogging)
        {
            Debug.Log($"[PRISMATIC FORCE DRIVE] {joint.jointObject.name}: Applying force {targetForce:F2} N (input: {inputValue:F2}, speed: {joint.speed})");
        }
        
        // For ArticulationBody Force drive, Unity might expect us to set the target as the force value
        // This is different from Target drive where target is a position
        // Let's try the proper ArticulationDrive approach first
        drive.target = targetForce;
        SetPrismaticDrive(articulation, drive);
        
        // Additional force application for more responsive behavior
        // Apply external force if the drive alone isn't sufficient
        if (Mathf.Abs(targetForce) > 0.01f)
        {
            // Scale down the external force since we're also using the drive
            float externalForce = targetForce * 0.1f; // 10% of drive force as external boost
            
            // Apply force along the correct prismatic axis
            Vector3 forceVector = Vector3.zero;
            if (articulation.linearLockX == ArticulationDofLock.LimitedMotion)
                forceVector = articulation.transform.right * externalForce; // X-axis
            else if (articulation.linearLockY == ArticulationDofLock.LimitedMotion)
                forceVector = articulation.transform.up * externalForce; // Y-axis
            else if (articulation.linearLockZ == ArticulationDofLock.LimitedMotion)
                forceVector = articulation.transform.forward * externalForce; // Z-axis
            else
                forceVector = articulation.transform.right * externalForce; // fallback to X
            
            articulation.AddForce(forceVector, ForceMode.Force);
            
            if (enableDebugLogging)
            {
                Debug.Log($"[PRISMATIC FORCE DRIVE] {joint.jointObject.name}: Drive force: {targetForce:F2}N, External boost: {externalForce:F2}N");
            }
        }
    }
    
    void UpdateSphericalJoint(ArticulationBody articulation, JointMovementState movementState, JointConfig joint)
    {
        // Spherical joints are more complex - for now, just control the X axis
        // In a full implementation, you'd want separate input axes for X, Y, Z rotation
        Debug.LogWarning($"Spherical joint {joint.jointObject.name}: Only X-axis control implemented. Consider using separate controllers for full 3-axis control.");
        
        var xDrive = articulation.xDrive;
        float inputValue = (float)movementState;
        
        switch (joint.DetectedDriveType)
        {
            case ArticulationDriveType.Target:
                float currentRotation = Mathf.Rad2Deg * GetSafeJointPosition(articulation, 0);
                float rotationChange = inputValue * joint.speed * Time.deltaTime;
                float rotationGoal = currentRotation + rotationChange;
                
                if (joint.useCustomLimits)
                {
                    rotationGoal = Mathf.Clamp(rotationGoal, joint.minLimit, joint.maxLimit);
                }
                
                xDrive.target = rotationGoal;
                break;
            case ArticulationDriveType.Velocity:
                xDrive.targetVelocity = inputValue * joint.speed;
                break;
            case ArticulationDriveType.Force:
                xDrive.target = inputValue * joint.speed;
                break;
        }
        
        articulation.xDrive = xDrive;
        
        if (enableDebugLogging)
        {
            Debug.Log($"Spherical Joint X-axis - Drive Type: {joint.DetectedDriveType}, Input: {inputValue:F2}");
        }
    }
    
    void StopAllJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (!articulationBodies.ContainsKey(i)) continue;
            
            ArticulationBody articulation = articulationBodies[i];
            JointConfig joint = joints[i];
            
            // Stop movement based on drive type
            switch (joint.DetectedDriveType)
            {
                case ArticulationDriveType.Target:
                    // Keep current position as target (stops movement)
                    var drive = articulation.xDrive;
                    if (joint.DetectedJointType == ArticulationJointType.RevoluteJoint)
                    {
                        drive.target = Mathf.Rad2Deg * GetSafeJointPosition(articulation, 0);
                    }
                    else
                    {
                        drive.target = GetSafeJointPosition(articulation, 0);
                    }
                    articulation.xDrive = drive;
                    break;
                    
                case ArticulationDriveType.Velocity:
                    // Set velocity to zero
                    var velocityDrive = articulation.xDrive;
                    velocityDrive.targetVelocity = 0f;
                    articulation.xDrive = velocityDrive;
                    break;
                    
                case ArticulationDriveType.Force:
                    // Set force to zero
                    var forceDrive = articulation.xDrive;
                    forceDrive.target = 0f;
                    articulation.xDrive = forceDrive;
                    break;
            }
        }
    }
    
    static JointMovementState GetMovementStateFromInput(float inputVal, float deadzone = 0.01f)
    {
        if (inputVal > deadzone)
        {
            return JointMovementState.Positive;
        }
        else if (inputVal < -deadzone)
        {
            return JointMovementState.Negative;
        }
        else
        {
            return JointMovementState.Fixed;
        }
    }
    
    // Utility methods for runtime inspection
    public void LogJointInfo()
    {
        Debug.Log("=== Joint Configuration Info ===");
        for (int i = 0; i < joints.Length; i++)
        {
            Debug.Log($"Joint {i}: {joints[i].JointInfo}");
        }
    }
    
    public void VerifyJointLimits()
    {
        Debug.Log("=== Joint Limits Verification ===");
        for (int i = 0; i < joints.Length; i++)
        {
            if (!articulationBodies.ContainsKey(i)) continue;
            
            ArticulationBody articulation = articulationBodies[i];
            JointConfig joint = joints[i];
            var drive = articulation.xDrive;
            
            Debug.Log($"Joint {i} ({joint.jointObject.name}):");
            Debug.Log($"  Type: {joint.DetectedJointType}, Drive: {joint.DetectedDriveType}");
            
            if (joint.DetectedJointType == ArticulationJointType.RevoluteJoint)
            {
                float unityMinDeg = Mathf.Rad2Deg * drive.lowerLimit;
                float unityMaxDeg = Mathf.Rad2Deg * drive.upperLimit;
                Debug.Log($"  Unity Limits: [{unityMinDeg:F2}°, {unityMaxDeg:F2}°]");
                
                if (joint.useCustomLimits)
                {
                    Debug.Log($"  Custom Limits: [{joint.minLimit:F2}°, {joint.maxLimit:F2}°] (ACTIVE)");
                }
            }
            else if (joint.DetectedJointType == ArticulationJointType.PrismaticJoint)
            {
                Debug.Log($"  Unity Limits: [{drive.lowerLimit:F3}m, {drive.upperLimit:F3}m]");
                
                if (joint.useCustomLimits)
                {
                    Debug.Log($"  Custom Limits: [{joint.minLimit:F3}m, {joint.maxLimit:F3}m] (ACTIVE)");
                }
            }
            
            Debug.Log($"  Drive to Limits Mode: {(joint.driveToLimits ? "ENABLED" : "DISABLED")}");
            Debug.Log($"  Current Position: {(joint.DetectedJointType == ArticulationJointType.RevoluteJoint ? (Mathf.Rad2Deg * GetSafeJointPosition(articulation, 0)).ToString("F2") + "°" : GetSafeJointPosition(articulation, 0).ToString("F3") + "m")}");
        }
    }
    
    public void TestDriveToLimits(int jointIndex)
    {
        if (!IsJointConfigured(jointIndex))
        {
            Debug.LogError($"Joint {jointIndex} is not configured");
            return;
        }
        
        JointConfig joint = joints[jointIndex];
        ArticulationBody articulation = articulationBodies[jointIndex];
        
        if (joint.DetectedDriveType != ArticulationDriveType.Target)
        {
            Debug.LogWarning($"Joint {jointIndex} is not using Target drive type. Drive to limits only works with Target drive.");
            return;
        }
        
        Debug.Log($"Testing drive to limits for joint {jointIndex} ({joint.jointObject.name})");
        
        // Test drive to minimum limit
        Debug.Log("Driving to minimum limit...");
        UpdateTargetDrivePrismatic(articulation, articulation.xDrive, -1f, joint);
        
        // You can call this method in a coroutine to test both limits with delay
    }
    
    public void RedetectJointTypes()
    {
        Debug.Log("=== Re-detecting Joint Types ===");
        for (int i = 0; i < joints.Length; i++)
        {
            if (!articulationBodies.ContainsKey(i)) continue;
            
            ArticulationBody articulation = articulationBodies[i];
            Debug.Log($"Joint {i} ({joints[i].jointObject.name}):");
            Debug.Log($"  Cached DetectedDriveType: {joints[i].DetectedDriveType}");
            Debug.Log($"  Current Unity driveType: {articulation.xDrive.driveType}");
            
            if (joints[i].DetectedDriveType != articulation.xDrive.driveType)
            {
                Debug.LogWarning($"  MISMATCH! Re-detecting...");
                DetectAndConfigureJoint(i, articulation);
                Debug.Log($"  Updated DetectedDriveType: {joints[i].DetectedDriveType}");
            }
        }
    }
    
    public ArticulationBody GetArticulationBody(int jointIndex)
    {
        return articulationBodies.ContainsKey(jointIndex) ? articulationBodies[jointIndex] : null;
    }
    
    public bool IsJointConfigured(int jointIndex)
    {
        return jointIndex >= 0 && jointIndex < joints.Length && articulationBodies.ContainsKey(jointIndex);
    }
}
