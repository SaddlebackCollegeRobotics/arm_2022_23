using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IK_Target : MonoBehaviour
{
    [SerializeField] private Camera currentCamera;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Vector3 clickPos = currentCamera.ScreenToWorldPoint(Input.mousePosition);
            transform.position = clickPos;
        }
    }
}
