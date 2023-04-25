using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    private static InputManager instance;

    private PlayerInput playerInput;

    [HideInInspector] public InputAction armPlanarMovementAction;
    [HideInInspector] public InputAction safetyAction;

    [HideInInspector] public InputAction armForwardAction;
    [HideInInspector] public InputAction armBackwardAction;

    [HideInInspector] public InputAction speedTrimUpAction;
    [HideInInspector] public InputAction speedTrimDownAction;

    [HideInInspector] public InputAction gripPitchAction;
    [HideInInspector] public InputAction gripRollAction;

    [HideInInspector] public InputAction gripOpenAction;
    [HideInInspector] public InputAction gripCloseAction;

    [HideInInspector] public InputAction pokerOutAction;
    [HideInInspector] public InputAction pokerInAction;

    private void Awake()
    {   
        instance = this;

        playerInput = GetComponent<PlayerInput>();

        // Instantiate input actions ---------------------------------------------------------

        armPlanarMovementAction =  playerInput.actions.FindAction("ArmPlanarMovement");
        safetyAction = playerInput.actions.FindAction("Safety");

        armForwardAction = playerInput.actions.FindAction("ArmForward");
        armBackwardAction = playerInput.actions.FindAction("ArmBackward");

        speedTrimUpAction = playerInput.actions.FindAction("SpeedTrimUp");
        speedTrimDownAction = playerInput.actions.FindAction("SpeedTrimDown");

        gripPitchAction = playerInput.actions.FindAction("GripPitch");
        gripRollAction = playerInput.actions.FindAction("GripRoll");

        gripOpenAction = playerInput.actions.FindAction("GripOpen");
        gripCloseAction = playerInput.actions.FindAction("GripClose");

        pokerOutAction = playerInput.actions.FindAction("PokerOut");
        pokerInAction = playerInput.actions.FindAction("PokerIn");
        
        // Assign event functions ------------------------------------------------------------

        /*aimLockAction.started += OnAimLock;
        aimLockAction.canceled += OnAimLock;*/
    }

    public static InputManager GetInstance()
    {
        return instance;
    }

}
