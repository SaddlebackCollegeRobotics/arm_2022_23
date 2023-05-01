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
    [SerializeField] private Transform[] actuatorJointList;
    [SerializeField] private Transform[] jointList; // Arm joints
    private std_msgs.msg.Float32MultiArray msg;

    public float[] dataArray = new float[1]; // Extra data to be published.

    InputManager inputManager;


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
            msg.Data = new float[8];

            ros2Node = ros2Unity.CreateNode("ROS2UnityNode");
            chatter_pub = ros2Node.CreatePublisher<std_msgs.msg.Float32MultiArray>("/arm/control_instruction_IK"); 
        }

        inputManager = InputManager.GetInstance();
    }

    // Update is called once per frame
    void Update()
    {
        if (ros2Unity.Ok())
        {
            // Bicep actuator length.
            msg.Data[0] = (actuatorJointList[0].position - actuatorJointList[1].position).magnitude;

            // Forearm actuator length.
            msg.Data[1] = (actuatorJointList[2].position - actuatorJointList[3].position).magnitude;
            
            // Turret rotation.
            msg.Data[2] = convertAngleRange(jointList[0].localEulerAngles.y);

            // Hand pitch
            msg.Data[3] = convertAngleRange(jointList[1].localEulerAngles.z);

            // Hand roll
            msg.Data[4] = convertAngleRange(jointList[2].localEulerAngles.x);

            int fingerDir = inputManager.gripCloseAction.IsPressed() ? -1 : inputManager.gripOpenAction.IsPressed() ? 1 : 0;
            int pokerDir = inputManager.pokerInAction.IsPressed() ? -1 : inputManager.pokerOutAction.IsPressed() ? 1 : 0;
            
            // Handle fingers Open/Close
            msg.Data[5] = fingerDir;
            
            // Handle poker In/Out
            msg.Data[6] = pokerDir;

            // Handle safety trigger
            msg.Data[7] = inputManager.safetyAction.IsPressed() ? 1 : 0;

            chatter_pub.Publish(msg);
        }
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
