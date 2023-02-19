using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIManager : MonoBehaviour
{
    [SerializeField] private TextMeshProUGUI[] textList;
    [SerializeField] private Transform[] jointList;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Base rotation
        textList[0].text = (int)convertAngleRange(jointList[0].localEulerAngles.y) + "°";
        // Hand Pitch
        textList[1].text = (int)convertAngleRange(jointList[1].localEulerAngles.z) + "°";
        // Hand Roll
        textList[2].text = (int)convertAngleRange(jointList[2].localEulerAngles.x) + "°";
        // Bicep
        textList[3].text = (int)convertAngleRange(jointList[3].localEulerAngles.z) + "°";
        // Forearm
        textList[4].text = (int)convertAngleRange(jointList[4].localEulerAngles.z) + "°";
    }


    private float convertAngleRange(float angle)
    {
        if (angle > 180)
            angle -= 360;

        return angle;
    }

}
