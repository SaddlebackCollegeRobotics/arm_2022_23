using Unity.VisualScripting;
using UnityEngine;

// TODO
// Rotation for IK is set globally and not locally, causing limits of 90 and -90 while IK is active


public class HandIKSolver : MonoBehaviour
{
    [SerializeField] private Transform handJoint;
    [SerializeField] private float desiredRotation = 0;

    private RootMotion.FinalIK.RotationLimitHinge pitchLimit;

    Vector3 currentRotation;
    bool enableSolver = false;


    // Start is called before the first frame update
    void Start()
    {
        pitchLimit = handJoint.GetComponent<RootMotion.FinalIK.RotationLimitHinge>();
    }

    // Update is called once per frame
    void Update()
    {
        currentRotation = handJoint.eulerAngles;

        if (enableSolver)
        {
            handJoint.rotation = Quaternion.Euler(currentRotation.x, currentRotation.y, desiredRotation);
        }
    }


    public void SetDesiredRotation(float angle)
    {
        angle = convertAngleRange(angle);

        // Take into account limits
        if (angle > pitchLimit.max)
            desiredRotation = pitchLimit.max;
        else if (angle < pitchLimit.min)
            desiredRotation = pitchLimit.min;
        else
            desiredRotation = angle;

    }

    public void ChangePitch(float addition)
    {
        if (enableSolver)
            SetDesiredRotation(addition + desiredRotation);
        else
            handJoint.rotation = Quaternion.Euler(currentRotation.x, currentRotation.y, currentRotation.z + addition);

    }

    private void ResetDesiredRotation()
    {
        SetDesiredRotation(handJoint.eulerAngles.z);
    }

    public float GetDesiredRotation()
    {
        return desiredRotation;
    }

    public void EnableSolver(bool b)
    {
        if (b)
            ResetDesiredRotation();

        enableSolver = b;
    }

    private float convertAngleRange(float angle)
    {
        if (angle > 180)
            angle -= 360;

        return angle;
    }

}
