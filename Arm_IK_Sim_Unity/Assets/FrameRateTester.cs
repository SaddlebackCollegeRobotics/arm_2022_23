using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FrameRateTester : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SetTargetFrameRate(int fps)
    {
        Application.targetFrameRate = fps;
    }


}
