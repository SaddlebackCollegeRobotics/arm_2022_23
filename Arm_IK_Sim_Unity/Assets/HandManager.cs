using UnityEngine;

public class HandManager : MonoBehaviour
{
    // Note: UI buttons use Trigger Event components to call functions here.

    [SerializeField] private HandIKSolver handIKSolver;
    [SerializeField] private Transform rollJoint;

    [Header("Pitch")]
    [SerializeField] private float pitchSpeedFactor = 20;
    private int pitchDir = 0;

    [Header("Roll")]
    [SerializeField] private float rollSpeedFactor = 20;
    private int rollDir = 0;

    [Header("Fingers")]
    [SerializeField] private float fingerSpeedFactor = 30;
    private int fingerDir = 0;


    // Update is called once per frame
    void Update()
    {
        // Update desired rotation for pitch IK solver
        if (pitchDir != 0)
            handIKSolver.ChangePitch(-pitchDir * pitchSpeedFactor * Time.deltaTime);

        // Update roll
        rollJoint.localRotation = Quaternion.Euler(rollJoint.localEulerAngles.x + (rollDir * rollSpeedFactor * Time.deltaTime), 0, 0);

        // Update finger movement direction.
        if (DataPublisher.instance)
            DataPublisher.instance.dataArray[0] = fingerDir * fingerSpeedFactor;
    }

    // Change pitch of hand (-1, 0, 1)
    public void ChangePitch(int direction)
    {
        pitchDir = direction;
    }

    // Change roll of hand (-1, 0, 1)
    public void ChangeRoll(int direction)
    {
        rollDir = direction;
    }

    // Open and close fingers on hand (-1, 0, 1)
    public void ChangeFingerDir(int direction)
    {
        fingerDir = direction;
    }

    // Enable IK on pitch of hand
    public void EnableIKPitchSolver(bool b)
    {
        handIKSolver.EnableSolver(b);
    }

    public void SetPitchSpeed(float speed)
    {
        pitchSpeedFactor = speed;
    }

    public void SetRollSpeed(float speed)
    {
        rollSpeedFactor = speed;
    }

    public void SetGrabSpeed(float speed)
    {
        fingerSpeedFactor = speed;
    }

}
