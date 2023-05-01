// Author: Cameron Rosenthal @Supernova1114

using RootMotion.FinalIK;
using UnityEngine;


public class IK_Controller : MonoBehaviour
{
    [Header("Speed Adjustment")]
    [SerializeField] private float turretRotationSpeed = 25;
    [SerializeField] private float armMovementSpeed = 5;
    [SerializeField] private float gripPitchSpeed = 20;
    [SerializeField] private float gripRollSpeed = 20;

    [Header("Limits")]
    [SerializeField] private bool IKTargetLimits = true;
    [SerializeField] private bool turretRotLimits = true;

    [SerializeField]
    private float[,] IKLimits = {

        {-14f, -7.847f},   // IK Target  = (Min X, Max X)   
        {-4.67f, 10.8f},     // IK Target  = (Min Y, Max Y)
        {-225f, 225f}        // Turret Rot = (Min Angle, Max Angle)
    
    };

    [Header("IK Joints")]
    [SerializeField] private Transform turretJoint;
    [SerializeField] private Transform IKTarget;

    [SerializeField] private Transform gripPitchJoint;
    [SerializeField] private Transform gripEnd;

    [SerializeField] private Transform IKTargetHome;


    [Header("Other Joints")]
    [SerializeField] private Transform gripRollJoint;

    [Header("Gripper")]
    [SerializeField] private float desiredGripPitchAngle;
    [SerializeField] private bool useGripIK = true;
    

    private InputManager inputManager;

    private void Start()
    {
        inputManager = InputManager.GetInstance();
    }

    

    private void Update() // TODO - combine rotation setters for turretJoint localRotation
    {

        Vector2 left_stick = inputManager.leftStickAction.ReadValue<Vector2>();
        Vector2 right_stick = inputManager.rightStickAction.ReadValue<Vector2>();

        // TODO - May need to fix some issues with this line
        // Rotation input from stick with greater value.
        float rotation_input = Mathf.Abs(left_stick.x) > Mathf.Abs(right_stick.x) ? left_stick.x : right_stick.x;

        // Move IK target Up and Down / Forward and Backwards
        IKTarget.localPosition += new Vector3(-left_stick.y, right_stick.y) * armMovementSpeed * Time.deltaTime;

        // Rotate turret joint
        if (Mathf.Abs(rotation_input) > 0)
            turretJoint.localRotation = Quaternion.AngleAxis(turretRotationSpeed * Time.deltaTime * rotation_input, turretJoint.up) * turretJoint.localRotation;

        // Handle IK for gripper pitch ------------------------------------------------------------

        if (useGripIK)
        {
            // TODO - Translate global angle to local angle.
            gripPitchJoint.rotation = Quaternion.Euler(gripPitchJoint.eulerAngles.x, gripPitchJoint.eulerAngles.y, desiredGripPitchAngle);
            gripPitchJoint.localEulerAngles = new Vector3(0, 0, gripPitchJoint.localEulerAngles.z);
        }

        // Handle gripper pitch angle -------------------------------------------------------

        float pitchDir = inputManager.gripPitchUpAction.IsPressed() ? -1 : inputManager.gripPitchDownAction.IsPressed() ? 1 : 0;
        
        if (Mathf.Abs(pitchDir) > 0)
            desiredGripPitchAngle += pitchDir * gripPitchSpeed * Time.deltaTime;

        // Handle gripper roll angle ---------------------------------------------------------
        // TODO - May want to separate normal control from IK control ???

        float rollDir = inputManager.gripRollLeftAction.IsPressed() ? -1 : inputManager.gripRollRightAction.IsPressed() ? 1 : 0;

        if (Mathf.Abs(rollDir) > 0)
            gripRollJoint.localRotation = Quaternion.Euler(gripRollJoint.localEulerAngles.x + (rollDir * gripRollSpeed * Time.deltaTime), gripRollJoint.localRotation.y, gripRollJoint.localRotation.z);


        // Move IK target forward or backward along gripper direction -----------------------------

        Vector3 gripperDirection = (gripEnd.position - gripPitchJoint.position).normalized;

        int moveDir = inputManager.armForwardAction.IsPressed() ? 1 : inputManager.armBackwardAction.IsPressed() ? -1 : 0;

        if (Mathf.Abs(moveDir) > 0)
            IKTarget.position += gripperDirection * moveDir * armMovementSpeed * Time.deltaTime;

        // Reset local Z to zero.
        IKTarget.localPosition = new Vector3(IKTarget.localPosition.x, IKTarget.localPosition.y, 0);

        // Enforce IK target bounds -------------------------------

        if (IKTargetLimits)
        {
            Vector3 localPos = IKTarget.localPosition;

            for (int axis = 0; axis < 2; axis++)
            {
                float lowerLimit = IKLimits[axis, 0];
                float upperLimit = IKLimits[axis, 1];

                localPos[axis] = Mathf.Clamp(localPos[axis], lowerLimit, upperLimit);
            }

            IKTarget.localPosition = localPos;
        }

        // Enforce turret rotation limits ------------------------------

        if (turretRotLimits)
        {
            // TODO
        }

    }

}
