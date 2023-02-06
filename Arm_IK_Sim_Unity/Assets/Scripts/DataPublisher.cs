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
            msg.Data = new float[2];

            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.Float32MultiArray>("chatter"); 
        }
        
    }

    // Update is called once per frame
    void Update()
    {

        for (int i = 0; i < jointList.Length; i++)
        {
            msg.Data[i] = jointList[i].rotation.eulerAngles.z;
        }

        

        chatter_pub.Publish(msg);
    }
}
