using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseRotatorVis : MonoBehaviour
{
    [SerializeField] Transform target;
    private Vector3 currentRotation;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        currentRotation = transform.eulerAngles;
        transform.rotation = Quaternion.Euler(currentRotation.x, currentRotation.y, target.rotation.eulerAngles.y + 90);
    }
}
