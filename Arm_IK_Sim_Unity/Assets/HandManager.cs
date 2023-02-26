using UnityEngine;

public class HandManager : MonoBehaviour
{
    // Note: UI buttons use Trigger Event components to call functions here.

    [SerializeField] private HandIKSolver handIKSolver;
    [SerializeField] private Transform rollJoint;

    [Header("Pitch")]
    [SerializeField] private float pitchSpeedFactor = 20;
    private float pitchDir = 0;

    [Header("Roll")]
    [SerializeField] private float rollSpeedFactor = 20;
    private float rollDir = 0;

    [Header("Fingers")]
    [SerializeField] private float fingerSpeedFactor = 100;
    private float gripDir = 0;
    private Vector2 input_axis;
    private Vector2 input_triggers;
    private bool use_gamepad = false;


    // Update is called once per frame
    void Update()
    {
        // Handle gamepad control
        if (use_gamepad)
        {
            HandleGamepadInput();
        }
        
        // Update desired rotation for pitch IK solver
        if (pitchDir != 0)
        {
            handIKSolver.ChangePitch(-pitchDir * pitchSpeedFactor * Time.deltaTime);
        }

        // Update roll
        rollJoint.localRotation = Quaternion.Euler(rollJoint.localEulerAngles.x + (rollDir * rollSpeedFactor * Time.deltaTime), 0, 0);

        print(fingerSpeedFactor * gripDir);

        // Update finger movement direction.
        if (DataPublisher.instance)
            DataPublisher.instance.dataArray[0] = gripDir * fingerSpeedFactor;
    }


    private void HandleGamepadInput()
    {
        input_triggers = new Vector2(Input.GetAxis("Left_Trigger"), Input.GetAxis("Right_Trigger"));
        input_axis = new Vector2(Input.GetAxis("RS_X"), Input.GetAxis("RS_Y")); // Right stick x and y

        rollDir = 0;
        pitchDir = 0;
        gripDir = 0;

        // Handle gamepad input
        if (input_triggers.y < 0)
        {
            if (Input.GetButton("Right_Bumper"))
            {
                // Control roll
                rollDir = input_axis.x;
            }
            else
            {
                // Control pitch
                pitchDir = input_axis.y;
            }
        }

        // Handle grip controls
        if (input_triggers.y < 0)
        {
            if (Input.GetButton("o_button"))
            {
                // Open grip
                gripDir = 1;

            }
            else if (Input.GetButton("x_button"))
            {
                // Close grip
                gripDir = -1;
            }

            
        }

    }


    // Change pitch of hand (-1, 0, 1)
    public void ChangePitch(int direction)
    {
        pitchDir = direction;
    }

    // Change roll of hand (-1, 0, 1)
    public void ChangeRoll(int direction)
    {
        rollDir = direction;
    }

    // Open and close fingers on hand (-1, 0, 1)
    public void ChangeFingerDir(int direction)
    {
        gripDir = direction;
    }

    // Enable IK on pitch of hand
    public void EnableIKPitchSolver(bool b)
    {
        handIKSolver.EnableSolver(b);
    }

    public void SetPitchSpeed(float speed)
    {
        pitchSpeedFactor = speed;
    }

    public void SetRollSpeed(float speed)
    {
        rollSpeedFactor = speed;
    }

    public void SetGrabSpeed(float speed)
    {
        fingerSpeedFactor = speed;
    }

    public void EnableGamepad(bool b)
    {
        use_gamepad = b;
    }

}
