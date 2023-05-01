// Author: Cameron Rosenthal @Supernova1114

using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class DragableObject : MonoBehaviour
{
    [SerializeField] private Camera currentCamera;
    private bool isObjectPressed = false;
    private Vector3 currentMouseOffset = Vector3.zero;
    private Vector3 initialMouseOffset = Vector3.zero;

    
    // Get the mouse position in world space.
    public Vector3 GetMouseWorldPos()
    {
        return currentCamera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, GetObjectScreenPosZ()));
    }


    // Is object pressed with mouse.
    public bool IsObjectPressed()
    {
        return isObjectPressed;
    }


    // Get offset of mouse to object transform in world space.
    public Vector3 GetCurrentMouseOffset()
    {
        return currentMouseOffset;
    }


    // Get initial mouse offset when object is first pressed.
    public Vector3 GetInitialMouseOffset()
    {
        return initialMouseOffset;
    }


    // Return camera that is being used.
    public Camera GetCurrentCamera()
    {
        return currentCamera;
    }


    // To be implemented in child class --------------
    protected abstract void WhileMousePressed();
    protected abstract void OnMouseReleased();
    protected abstract void OnMousePressed();
    // -----------------------------------------------


    // Target is pressed
    private void OnMouseDown()
    {
        isObjectPressed = true;
        initialMouseOffset = transform.position - GetMouseWorldPos();

        OnMousePressed();
    }


    // Target is released.
    private void OnMouseUp()
    {
        isObjectPressed = false;
        OnMouseReleased();
    }


    // Target continues to be pressed.
    private void OnMouseDrag()
    {
        currentMouseOffset = transform.position - GetMouseWorldPos();
        WhileMousePressed();
    }

    
    // Get the z screen distance from object to camera.
    private float GetObjectScreenPosZ()
    {
        return currentCamera.WorldToScreenPoint(transform.position).z;
    }

}