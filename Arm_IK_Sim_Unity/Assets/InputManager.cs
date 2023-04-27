using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    private static InputManager instance;

    private PlayerInput playerInput;

    [HideInInspector] public InputAction leftStickAction;
    [HideInInspector] public InputAction rightStickAction;

    [HideInInspector] public InputAction safetyAction;

    [HideInInspector] public InputAction armForwardAction;
    [HideInInspector] public InputAction armBackwardAction;

    [HideInInspector] public InputAction speedTrimUpAction;
    [HideInInspector] public InputAction speedTrimDownAction;

    [HideInInspector] public InputAction gripPitchUpAction;
    [HideInInspector] public InputAction gripPitchDownAction;


    [HideInInspector] public InputAction gripRollLeftAction;
    [HideInInspector] public InputAction gripRollRightAction;

    [HideInInspector] public InputAction gripOpenAction;
    [HideInInspector] public InputAction gripCloseAction;

    [HideInInspector] public InputAction pokerOutAction;
    [HideInInspector] public InputAction pokerInAction;

    private void Awake()
    {   
        instance = this;

        playerInput = GetComponent<PlayerInput>();

        // Instantiate input actions ---------------------------------------------------------

        leftStickAction =  playerInput.actions.FindAction("LeftStick");
        rightStickAction =  playerInput.actions.FindAction("RightStick");

        safetyAction = playerInput.actions.FindAction("Safety");

        armForwardAction = playerInput.actions.FindAction("ArmForward");
        armBackwardAction = playerInput.actions.FindAction("ArmBackward");

        speedTrimUpAction = playerInput.actions.FindAction("SpeedTrimUp");
        speedTrimDownAction = playerInput.actions.FindAction("SpeedTrimDown");

        gripPitchUpAction = playerInput.actions.FindAction("GripPitchUp");
        gripPitchDownAction = playerInput.actions.FindAction("GripPitchDown");

        gripRollLeftAction = playerInput.actions.FindAction("GripRollLeft");
        gripRollRightAction = playerInput.actions.FindAction("GripRollRight");

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
