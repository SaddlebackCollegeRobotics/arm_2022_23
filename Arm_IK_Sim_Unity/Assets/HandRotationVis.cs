using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandRotationVis : MonoBehaviour
{
    [SerializeField] private Transform targetPitchJoint;
    [SerializeField] private Transform pitchJoint;
    [SerializeField] private Transform targetRollJoint;
    [SerializeField] private Transform rollJoint;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        pitchJoint.rotation = targetPitchJoint.rotation;
        rollJoint.rotation = targetRollJoint.rotation;
    }
}
