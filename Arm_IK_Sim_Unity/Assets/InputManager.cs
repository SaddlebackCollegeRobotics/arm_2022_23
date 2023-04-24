using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    private PlayerInput m_playerInput;

    private InputAction m_armPlanarMovementAction;
    private InputAction m_safetyAction;

    private InputAction m_raiseArmAction;
    private InputAction m_lowerArmAction;

    private InputAction m_speedTrimUpAction;
    private InputAction m_speedTrimDownAction;

    private InputAction m_gripPitch;
    private InputAction m_gripRoll;

    private InputAction m_gripOpen;
    private InputAction m_gripClose;

    private InputAction m_pokerOut;
    private InputAction m_pokerIn;

    private Vector2 m_armPlanarMovement;


    private void Awake()
    {
        // Set up player input
        m_playerInput = GetComponent<PlayerInput>();

        m_playerInput.actions.FindAction("ArmPlanarMovement");
        m_playerInput.actions.FindAction("Safety");

        m_playerInput.actions.FindAction("RaiseArm");
        m_playerInput.actions.FindAction("LowerArm");

        m_playerInput.actions.FindAction("SpeedTrimUp");
        m_playerInput.actions.FindAction("SpeedTrimDown");

        m_playerInput.actions.FindAction("GripPitch");
        m_playerInput.actions.FindAction("GripRoll");

        m_playerInput.actions.FindAction("GripOpen");
        m_playerInput.actions.FindAction("GripClose");

        m_playerInput.actions.FindAction("PokerOut");
        m_playerInput.actions.FindAction("PokerIn");



        /*m_aimLockAction.started += OnAimLock;
        m_aimLockAction.canceled += OnAimLock;*/
    }

    // Update is called once per frame
    void Update()
    {





    }

}
