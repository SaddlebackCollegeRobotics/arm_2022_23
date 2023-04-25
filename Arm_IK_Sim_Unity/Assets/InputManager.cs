using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    private static InputManager instance;

    private PlayerInput m_playerInput;

    [HideInInspector] private InputAction m_armPlanarMovementAction;
    [HideInInspector] public InputAction m_safetyAction;

    [HideInInspector] public InputAction m_raiseArmAction;
    [HideInInspector] public InputAction m_lowerArmAction;

    [HideInInspector] public InputAction m_speedTrimUpAction;
    [HideInInspector] public InputAction m_speedTrimDownAction;

    [HideInInspector] public InputAction m_gripPitchAction;
    [HideInInspector] public InputAction m_gripRollAction;

    [HideInInspector] public InputAction m_gripOpenAction;
    [HideInInspector] public InputAction m_gripCloseAction;

    [HideInInspector] public InputAction m_pokerOutAction;
    [HideInInspector] public InputAction m_pokerInAction;

    private Vector2 m_armPlanarMovement;

    private void Awake()
    {   
        instance = this;

        // Set up player input
        m_playerInput = GetComponent<PlayerInput>();

        // Instantiate input actions ---------------------------------------------------------

        m_armPlanarMovementAction =  m_playerInput.actions.FindAction("ArmPlanarMovement");
        m_safetyAction = m_playerInput.actions.FindAction("Safety");

        m_raiseArmAction = m_playerInput.actions.FindAction("RaiseArm");
        m_lowerArmAction = m_playerInput.actions.FindAction("LowerArm");

        m_speedTrimUpAction = m_playerInput.actions.FindAction("SpeedTrimUp");
        m_speedTrimDownAction = m_playerInput.actions.FindAction("SpeedTrimDown");

        m_gripPitchAction = m_playerInput.actions.FindAction("GripPitch");
        m_gripRollAction = m_playerInput.actions.FindAction("GripRoll");

        m_gripOpenAction = m_playerInput.actions.FindAction("GripOpen");
        m_gripCloseAction = m_playerInput.actions.FindAction("GripClose");

        m_pokerOutAction = m_playerInput.actions.FindAction("PokerOut");
        m_pokerInAction = m_playerInput.actions.FindAction("PokerIn");
        
        // Assign event functions ------------------------------------------------------------

        /*m_aimLockAction.started += OnAimLock;
        m_aimLockAction.canceled += OnAimLock;*/
    }

    private void Update()
    {
        m_armPlanarMovement = m_armPlanarMovementAction.ReadValue<Vector2>();
    }

    public Vector2 GetArmPlanarMovement()
    {
        return m_armPlanarMovement;
    }

    public static InputManager GetInstance()
    {
        return instance;
    }

}
