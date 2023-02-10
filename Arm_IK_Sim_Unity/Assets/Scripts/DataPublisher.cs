using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class DataPublisher : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.Float32MultiArray> chatter_pub;
    [SerializeField] private Transform[] jointList;
    private std_msgs.msg.Float32MultiArray msg;

    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = transform.parent.GetComponent<ROS2UnityComponent>();

        if (ros2Unity.Ok())
        {
            msg = new std_msgs.msg.Float32MultiArray();
            msg.Data = new float[3];

            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.Float32MultiArray>("/Arm_Controls_Sim"); 
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        // Shoulder joint.
        msg.Data[0] = jointList[0].localRotation.eulerAngles.z;
        // Elbow joint.
        msg.Data[1] = jointList[1].localRotation.eulerAngles.z;
        // Base rotation.
        msg.Data[2] = jointList[2].localRotation.eulerAngles.y;
        

        chatter_pub.Publish(msg);
    }
}
