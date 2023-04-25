// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Controller : MonoBehaviour
{
    [Header("Speed Adjustment")]
    [SerializeField] private float turretRotationSpeed = 25;
    [SerializeField] private float armMovementSpeed = 5;
    [SerializeField] private float armMovementOtherSpeed = 3;

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

    private InputManager inputManager;


    private void Start()
    {
        inputManager = InputManager.GetInstance();
    }

    private void Update()
    {
        Vector2 movement = inputManager.armPlanarMovementAction.ReadValue<Vector2>();

        // Move IK target up and down
        IKTarget.localPosition += new Vector3(0, movement.y * armMovementSpeed * Time.deltaTime);

        // Rotate turret joint
        turretJoint.localRotation = Quaternion.Euler(turretJoint.localEulerAngles.x, turretJoint.localEulerAngles.y + (turretRotationSpeed * Time.deltaTime * movement.x), turretJoint.localEulerAngles.z);

        // Move IK target forward or backward.
        if (inputManager.armForwardAction.IsPressed())
        {
            IKTarget.localPosition += new Vector3(-armMovementOtherSpeed * Time.deltaTime, 0);
        }
        else if (inputManager.armBackwardAction.IsPressed())
        {
            IKTarget.localPosition += new Vector3(armMovementOtherSpeed * Time.deltaTime, 0);
        }

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
