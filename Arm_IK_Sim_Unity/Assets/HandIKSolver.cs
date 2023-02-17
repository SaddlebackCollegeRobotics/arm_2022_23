using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandIKSolver : MonoBehaviour
{
    [SerializeField] private Transform handJoint;
    [SerializeField] private float desiredRotation = 0;

    Vector3 currentRotation;
    bool enableSolver = false;
    

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (enableSolver)
        {
            currentRotation = handJoint.eulerAngles;
            handJoint.rotation = Quaternion.Euler(currentRotation.x, currentRotation.y, desiredRotation);
        }
    }


    public void SetDesiredRotation(float angle)
    {
        desiredRotation = angle;
    }

    public float GetDesiredRotation()
    {
        return desiredRotation;
    }

    public void EnableSolver(bool b)
    {
        enableSolver = b;
    }

}
