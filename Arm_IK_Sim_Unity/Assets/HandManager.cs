using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class HandManager : MonoBehaviour
{
    // Note: UI buttons use Trigger Event components to call functions here.

    [SerializeField] private HandIKSolver handIKSolver;
    [SerializeField] private Transform rollJoint;

    [Header("Pitch")]
    [SerializeField] private float pitchSpeedFactor = 20;
    private bool enablePitch = true;
    private int pitchDir = 0;

    [Header("Roll")]
    [SerializeField] private float rollSpeedFactor = 20;
    private bool enableRoll = true;
    private int rollDir = 0;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (enablePitch)
        {
            handIKSolver.SetDesiredRotation(handIKSolver.GetDesiredRotation() + (pitchDir * pitchSpeedFactor * Time.deltaTime));
        }

        if (enableRoll)
        {
            rollJoint.localRotation = Quaternion.Euler(rollJoint.localEulerAngles.x + (rollDir * rollSpeedFactor * Time.deltaTime), 0, 0);
        }
    }

    // Change pitch of hand (-1, 0, 1)
    public void ChangePitch(int direction)
    {
        pitchDir = direction;
    }

    public void ChangeRoll(int direction)
    {
        rollDir = direction;
    }

    public void EnableIKPitchSolver(bool b)
    {
        handIKSolver.EnableSolver(b);
    }

}
