// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Controller : MonoBehaviour
{
    [Header("Speed Adjustment")]
    [SerializeField] private float turretRotationSpeed = 25;
    [SerializeField] private float armMovementSpeed = 5;
    [SerializeField] private float armMovementOtherSpeed = 3;
    [SerializeField] private float gripPitchSpeed = 20;

    [Header("Limits")]
    [SerializeField] private bool IKTargetLimits = true;
    [SerializeField] private bool turretRotLimits = true;

    [SerializeField]
    private float[,] IKLimits = {

        {-13.8f, -7f},   // IK Target  = (Min X, Max X)   
        {-4.67f, 10.8f}, // IK Target  = (Min Y, Max Y)
        {-225f, 225f}    // Turret Rot = (Min Angle, Max Angle)
    
    };

    [Header("IKJoints")]
    [SerializeField] private Transform turretJoint;
    [SerializeField] private Transform IKTarget;

    [SerializeField] private Transform gripPitchJoint;
    [SerializeField] private Transform gripEnd;

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
        Vector2 right_stick = inputManager.rightStickAction.ReadValue<Vector2>();

        // Move IK target Up and Down
        IKTarget.localPosition += new Vector3(0, right_stick.y * armMovementSpeed * Time.deltaTime);

        // Rotate turret joint using right_stick
        turretJoint.localRotation = Quaternion.Euler(turretJoint.localEulerAngles.x, turretJoint.localEulerAngles.y + (turretRotationSpeed * Time.deltaTime * right_stick.x), turretJoint.localEulerAngles.z);

        Vector2 left_stick = inputManager.leftStickAction.ReadValue<Vector2>();

        // Move IK target Forward and Backward.
        IKTarget.localPosition += new Vector3(left_stick.y * -armMovementSpeed * Time.deltaTime, 0);

        // Rotate turret joint using left_stick

        turretJoint.localRotation = Quaternion.Euler(turretJoint.localEulerAngles.x, turretJoint.localEulerAngles.y + (turretRotationSpeed * Time.deltaTime * left_stick.x), turretJoint.localEulerAngles.z);

        // TODO - Need to add this back in as reading from left stick.
        /*// Move IK target forward or backward.
        if (inputManager.armForwardAction.IsPressed())
        {
            IKTarget.localPosition += new Vector3(-armMovementOtherSpeed * Time.deltaTime, 0);
        }
        else if (inputManager.armBackwardAction.IsPressed())
        {
            IKTarget.localPosition += new Vector3(armMovementOtherSpeed * Time.deltaTime, 0);
        }*/

        // Handle IK for gripper pitch ------------------------------------------------------------

        if (useGripIK)
        {
            gripPitchJoint.rotation = Quaternion.Euler(gripPitchJoint.eulerAngles.x, gripPitchJoint.eulerAngles.y, desiredGripPitchAngle);
            gripPitchJoint.localEulerAngles = new Vector3(0, 0, gripPitchJoint.localEulerAngles.z);
        }

        if (inputManager.gripPitchUpAction.IsPressed())
        {
            desiredGripPitchAngle += -gripPitchSpeed * Time.deltaTime;
        }
        else if (inputManager.gripPitchDownAction.IsPressed())
        {
            desiredGripPitchAngle += gripPitchSpeed * Time.deltaTime;
        }

        // Move IK target forward or backward along gripper direction -----------------------------

        Vector3 gripperDirection = (gripEnd.position - gripPitchJoint.position).normalized;
        
        if (inputManager.armForwardAction.IsPressed())
        {
            IKTarget.position += gripperDirection * armMovementOtherSpeed * Time.deltaTime;
        }
        else if (inputManager.armBackwardAction.IsPressed())
        {
            IKTarget.position += -gripperDirection * armMovementOtherSpeed * Time.deltaTime;
        }

        IKTarget.localPosition = new Vector3(IKTarget.localPosition.x, IKTarget.localPosition.y, 0);

        // Enforce IK target bounds -------------------------------

        if (IKTargetLimits)
        {
            Vector3 localPos = IKTarget.localPosition;

            if (localPos.x > IKLimits[0, 1])
            {
                localPos.x = IKLimits[0, 1];
            }
            else if (localPos.x < IKLimits[0, 0])
            {
                localPos.x = IKLimits[0, 0];
            }

            if (localPos.y > IKLimits[1, 1])
            {
                localPos.y = IKLimits[1, 1];
            }
            else if (localPos.y < IKLimits[1, 0])
            {
                localPos.y = IKLimits[1, 0];
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
