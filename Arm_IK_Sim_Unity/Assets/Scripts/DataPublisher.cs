using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class DataPublisher : MonoBehaviour
{
    public static DataPublisher instance;

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.Float32MultiArray> chatter_pub;
    [SerializeField] private Transform[] jointList; // Arm joints
    private std_msgs.msg.Float32MultiArray msg;

    public float[] dataArray = new float[1]; // Extra data to be published.


    private void Awake()
    {
        instance = this;
    }

    // Start is called before the first frame update
    void Start()
    {
        // Init Ros2 stuff
        ros2Unity = transform.parent.GetComponent<ROS2UnityComponent>();

        if (ros2Unity.Ok())
        {
            msg = new std_msgs.msg.Float32MultiArray();
            msg.Data = new float[6];

            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.Float32MultiArray>("/Arm_Controls_Sim"); 
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        // Note: Interior angles for arm joints

        // Shoulder joint.
        msg.Data[0] = 360 - jointList[0].localEulerAngles.z;
        // Elbow joint.
        msg.Data[1] = 180 - jointList[1].localEulerAngles.z;
        
        // Base rotation.
        msg.Data[2] = convertAngleRange(jointList[2].localEulerAngles.y);

        // Hand Pitch
        msg.Data[3] = convertAngleRange(jointList[3].localEulerAngles.z);

        // Hand Roll
        msg.Data[4] = convertAngleRange(jointList[4].localEulerAngles.x);

        // Hand Fingers Open/Close
        msg.Data[5] = dataArray[0];

        //print(msg.Data[0] + " " + msg.Data[1] + " " + msg.Data[2] + " " + msg.Data[3]);

        chatter_pub.Publish(msg);
    }


    // Converts angle range from (0 to 360) to (-180 to 180)
    private float convertAngleRange(float angle)
    {
        if (angle > 180)
            angle -= 360;

        return angle;
    }

    public void SetDataArray(float val, int index)
    {
        //dataArray[index] = val;
    }

}
