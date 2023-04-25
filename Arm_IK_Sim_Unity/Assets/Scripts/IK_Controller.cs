// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Controller : MonoBehaviour
{
    [Header("Speed Adjustment")]
    [SerializeField] private float m_turretRotationSpeed = 25;
    [SerializeField] private float m_armMovementSpeed = 5;
    [SerializeField] private float m_armMovementOtherSpeed = 3;

    [Header("Limits")]
    [SerializeField] private bool m_IKTargetLimits = true;
    [SerializeField] private bool m_TurretRotLimits = true;

    [SerializeField]
    private float[,] m_IKLimits = {

        {-13.8f, -7f},   // IK Target  = (Min X, Max X)   
        {-4.67f, 10.8f}, // IK Target  = (Min Y, Max Y)
        {-225f, 225f}    // Turret Rot = (Min Angle, Max Angle)
    
    };

    [Header("IKJoints")]
    [SerializeField] private Transform m_turretJoint;
    [SerializeField] private Transform m_IKTarget;

    private InputManager inputManager;


    private void Start()
    {
        inputManager = InputManager.GetInstance();
    }

    private void Update()
    {
        Vector2 movement = inputManager.m_armPlanarMovementAction.ReadValue<Vector2>();

        // Move IK target up and down
        m_IKTarget.localPosition += new Vector3(0, movement.y * m_armMovementSpeed * Time.deltaTime);

        // Rotate turret joint
        m_turretJoint.localRotation = Quaternion.Euler(m_turretJoint.localEulerAngles.x, m_turretJoint.localEulerAngles.y + (m_turretRotationSpeed * Time.deltaTime * movement.x), m_turretJoint.localEulerAngles.z);

        // Move IK target forward or backward.
        if (inputManager.m_armForwardAction.IsPressed())
        {
            m_IKTarget.localPosition += new Vector3(-m_armMovementOtherSpeed * Time.deltaTime, 0);
        }
        else if (inputManager.m_armBackwardAction.IsPressed())
        {
            m_IKTarget.localPosition += new Vector3(m_armMovementOtherSpeed * Time.deltaTime, 0);
        }

        // Enforce IK target bounds -------------------------------

        if (m_IKTargetLimits)
        {
            Vector3 localPos = m_IKTarget.localPosition;

            if (localPos.x > m_IKLimits[0, 1])
            {
                localPos.x = m_IKLimits[0, 1];
            }
            else if (localPos.x < m_IKLimits[0, 0])
            {
                localPos.x = m_IKLimits[0, 0];
            }

            if (localPos.y > m_IKLimits[1, 1])
            {
                localPos.y = m_IKLimits[1, 1];
            }
            else if (localPos.y < m_IKLimits[1, 0])
            {
                localPos.y = m_IKLimits[1, 0];
            }

            m_IKTarget.localPosition = localPos;
        }

        // Enforce turret rotation limits ------------------------------

        if (m_TurretRotLimits)
        {
            // TODO
        }

    }

}
