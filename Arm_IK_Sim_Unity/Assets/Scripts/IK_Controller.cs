// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Controller : MonoBehaviour
{
    [SerializeField] private float turretRotationSpeed = 25;
    [SerializeField] private float armMovementSpeed = 5;
    [SerializeField] private float armMovementOtherSpeed = 3;
    [SerializeField] private bool enforceLimits = true;
    [SerializeField] private Vector2 maxTargetBounds;
    [SerializeField] private Vector2 minTargetBounds;
    [SerializeField] private Vector2 rotationLimits;

    [SerializeField] private Transform turretJoint;
    //[SerializeField] private Transform targetJoint;
    [SerializeField] private Transform target;
    [SerializeField] private Transform elbowJoint;

    InputManager inputManager;

    private void Start()
    {
        inputManager = InputManager.GetInstance();
    }

    private void FixedUpdate()
    {
    }

    private void Update()
    {
        Vector2 movement = inputManager.GetArmPlanarMovement();

        target.localPosition += new Vector3(0, movement.y * armMovementSpeed * Time.deltaTime);

        turretJoint.localRotation = Quaternion.Euler(turretJoint.localEulerAngles.x, turretJoint.localEulerAngles.y + (turretRotationSpeed * Time.deltaTime * movement.x), turretJoint.localEulerAngles.z);
        //targetJoint.localRotation = Quaternion.Euler(targetJoint.localEulerAngles.x, targetJoint.localEulerAngles.y, targetJoint.localEulerAngles.z + -(turretRotationSpeed * Time.deltaTime * movement.y));

        //Vector2 elbowToTarget = (Vector2)(target.position - elbowJoint.position).normalized;

        if (inputManager.m_raiseArmAction.IsPressed())
        {
            target.localPosition += new Vector3(-armMovementOtherSpeed * Time.deltaTime, 0);
        }
        else if (inputManager.m_lowerArmAction.IsPressed())
        {
            target.localPosition += new Vector3(armMovementOtherSpeed * Time.deltaTime, 0);
        }

        // Enforce bounds -------------------------------

        if (enforceLimits)
        {
            Vector3 localPos = target.localPosition;

            if (localPos.x > maxTargetBounds.x)
            {
                localPos.x = maxTargetBounds.x;
            }
            else if (localPos.x < minTargetBounds.x)
            {
                localPos.x = minTargetBounds.x;
            }

            if (localPos.y > maxTargetBounds.y)
            {
                localPos.y = maxTargetBounds.y;
            }
            else if (localPos.y < minTargetBounds.y)
            {
                localPos.y = minTargetBounds.y;
            }

            target.localPosition = localPos;
        }
        

    }

}
