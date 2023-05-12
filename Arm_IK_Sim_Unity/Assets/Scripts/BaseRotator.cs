using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseRotator : DragableObject
{
    [SerializeField] private float speedFactor = 1f; // Factor to tell how fast to rotate based on mouse x offset.
    [SerializeField] private float smoothTime = 0.2f;
    private RootMotion.FinalIK.RotationLimitHinge limitHinge;
    private float initialRotation; // Initial rotation of the object when first pressed.
    private Vector3 currentVelocity = Vector3.zero;
    private Vector3 currentJoystickValue = Vector3.zero;
    private bool moveWithGamepad = false;
    private float currentRotation;
    private float hatHorizontal;
    private float left_trigger;
    private bool wasHoldingSafetyTrigger = false;

    void Start()
    {
        limitHinge = GetComponent<RootMotion.FinalIK.RotationLimitHinge>();
    }

    // Update is called once per frame
    void Update()
    {
        if (moveWithGamepad)
        {
            HandleGamepadInput();
        }

        if (wasHoldingSafetyTrigger)
        {
            LimitAngle();
            transform.rotation = Quaternion.Euler(0, currentRotation, 0);
        }
    }


    protected override void OnMousePressed()
    {
        initialRotation = transform.rotation.eulerAngles.y;
    }


    protected override void WhileMousePressed()
    {
        currentRotation = GetCurrentMouseOffset().x - GetInitialMouseOffset().x + initialRotation;
    }


    protected override void OnMouseReleased(){}


    private void HandleGamepadInput()
    {
        left_trigger = Input.GetAxis("Left_Trigger");
        hatHorizontal = Input.GetAxis("HatHorizontal");

        if (left_trigger < 0)
        {
            wasHoldingSafetyTrigger = true;
            
            currentJoystickValue = Vector3.SmoothDamp(currentJoystickValue, new Vector3(hatHorizontal, 0, 0), ref currentVelocity, smoothTime);
            currentRotation = transform.rotation.eulerAngles.y + (-currentJoystickValue.x * speedFactor * Time.deltaTime);
        }
        else if (wasHoldingSafetyTrigger)
        {
            wasHoldingSafetyTrigger = false;
            currentVelocity = Vector3.zero;
            currentJoystickValue = Vector3.zero;
        }
    }


    private void LimitAngle()
    {
        currentRotation = convertAngleRange(currentRotation);

        if (currentRotation > limitHinge.max)
        {
            currentRotation = limitHinge.max;
        }
        else if (currentRotation < limitHinge.min)
        {
            currentRotation = limitHinge.min;
        }
    }


    public void SetMoveWithGamepad(bool b)
    {
        moveWithGamepad = b;
    }

    // Converts angle range from (0 to 360) to (-180 to 180)
    private float convertAngleRange(float angle)
    {
        if (angle > 180)
            angle -= 360;

        return angle;
    }

    public void SetSpeedFactor(float speed)
    {
        speedFactor = speed;
    }

}
