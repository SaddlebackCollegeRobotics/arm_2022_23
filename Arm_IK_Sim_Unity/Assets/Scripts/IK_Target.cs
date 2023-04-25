// Author: Cameron Rosenthal @Supernova1114

using UnityEngine;


public class IK_Target : MonoBehaviour
{
    InputManager inputManager;

    private void Start()
    {
        inputManager = InputManager.GetInstance();
    }

    private void FixedUpdate()
    {
    }

    private void Update()
    {
        Vector2 movement = inputManager.GetArmPlanarMovement();

        transform.position += new Vector3(movement.x, 0, movement.y).normalized * 2 * Time.deltaTime;

        if (inputManager.m_raiseArmAction.IsPressed())
        {
            transform.position += Vector3.up * 2 * Time.deltaTime;
        }
        else if (inputManager.m_lowerArmAction.IsPressed())
        {
            transform.position += Vector3.down * 2 * Time.deltaTime;
        }
    }

}
