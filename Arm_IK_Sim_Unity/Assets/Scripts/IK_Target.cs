using UnityEngine;

public class IK_Target : MonoBehaviour
{
    [SerializeField] private Camera currentCamera;    // The scene camera.
    [SerializeField] private Collider2D targetBounds; // Position limiting bounds of the target.

    private Vector3 mousePos;    // Current mouse position.
    private Vector3 mouseOffset; // Let mouse click anywhere on target.

    private bool isClicked; // Is the target clicked?
    [SerializeField] private bool moveWithGamepad = false;
    [SerializeField] private float smoothTime = 0.5f;
    [SerializeField] private float speedFactor = 1.5f;
    
    private float horizontal;
    private float vertical;
    private Vector3 currentVelocity = Vector3.zero;

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


    // When target is pressed
    private void OnMouseDown()
    {
        mousePos = GetMouseWorldPos();
        mouseOffset = transform.position - mousePos;

        isClicked = true;
    }


    // While target is pressed.
    private void OnMouseDrag()
    {
        if (isClicked)
        {
            mousePos = GetMouseWorldPos();
            transform.position = new Vector3(mousePos.x + mouseOffset.x, mousePos.y + mouseOffset.y, transform.position.z);
        }
    }


    // When target is unpressed.
    private void OnMouseUp()
    {
        isClicked = false;
    }


    // Get the mouse position in world space.
    private Vector3 GetMouseWorldPos()
    {
        return currentCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, GetObjectScreenPosZ()));
    }

    
    // Get the z screen distance from object to camera.
    private float GetObjectScreenPosZ()
    {
        return currentCamera.WorldToScreenPoint(transform.position).z;
    }


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


    private void MoveUsingAxis()
    {
        horizontal = Input.GetAxis("Horizontal");
        vertical = Input.GetAxis("Vertical");
        
        transform.position = Vector3.SmoothDamp(transform.position, transform.position + new Vector3(horizontal, vertical, 0) * speedFactor, ref currentVelocity, smoothTime);
    }


    public void EnableGamepad(bool m)
    {
        moveWithGamepad = m;
    }

}
