// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Target : DragableObject
{
    [SerializeField] private Collider2D targetBounds;      // Position limiting bounds of the target.
    [SerializeField] private float smoothTime = 0.5f;      // Smooths target movement while using axis control.
    [SerializeField] private float speedFactor = 1.5f;     // A factor to control target speed while using axis control.
    
    private bool moveWithGamepad = false; // Should the target move using gamepad / keyboard axis.

    private float horizontal; // Horizontal controls axis.
    private float vertical; // Vertical controls axis.
    private Vector3 currentVelocity = Vector3.zero; // Used in the smoothing calculation.


    // Start is called before the first frame update
    void Start()
    {
        
    }


    // Update is called once per frame
    void Update()
    {
        if (moveWithGamepad)
        {
            MoveUsingAxis();
        }

        LimitBounds();
    }


    protected override void OnMousePressed(){}


    protected override void WhileMousePressed()
    {
        Vector3 mouseWorldPos = GetMouseWorldPos();
        Vector3 mouseOffset = GetInitialMouseOffset();
        transform.position = new Vector3(mouseWorldPos.x + mouseOffset.x, mouseWorldPos.y + mouseOffset.y, transform.position.z);
    }


    protected override void OnMouseReleased(){}


    // Limit where the target can move.
    private void LimitBounds()
    {
        Vector3 currPos = transform.position;
        
        if (currPos.x > targetBounds.bounds.max.x)
        {
            currPos.x = targetBounds.bounds.max.x;
        }
        else if (currPos.x < targetBounds.bounds.min.x)
        {
            currPos.x = targetBounds.bounds.min.x;
        }

        if (currPos.y > targetBounds.bounds.max.y)
        {
            currPos.y = targetBounds.bounds.max.y;
        }
        else if (currPos.y < targetBounds.bounds.min.y)
        {
            currPos.y = targetBounds.bounds.min.y;
        }

        transform.position = currPos;
    }


    // Move the target using gamepad / keyboard axis.
    private void MoveUsingAxis()
    {
        horizontal = Input.GetAxis("Horizontal");
        vertical = Input.GetAxis("Vertical");
        
        transform.position = Vector3.SmoothDamp(transform.position, transform.position + new Vector3(horizontal, vertical, 0) * speedFactor * Time.deltaTime, ref currentVelocity, smoothTime);
    }

    public void SetMoveWithGamepad(bool b)
    {
        moveWithGamepad = b;
    }

    public void SetSpeedFactor(float speed)
    {
        speedFactor = speed;
    }

}
