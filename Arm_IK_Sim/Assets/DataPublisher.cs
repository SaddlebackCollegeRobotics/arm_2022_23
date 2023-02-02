using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class DataPublisher : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.String> chatter_pub;
    [SerializeField] private GameObject robotJoint;


    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = transform.parent.GetComponent<ROS2UnityComponent>();

        if (ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("ROS2UnityListenerNode");
            chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.String>("chatter"); 
        }
        
    }

    // Update is called once per frame
    void Update()
    {
        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.Data = "Angle: " + robotJoint.transform.localEulerAngles.z;
        chatter_pub.Publish(msg);
    }
}
