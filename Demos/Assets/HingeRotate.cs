﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HingeRotate : MonoBehaviour {

    NewtonActuator na = null;
    float targetAngle = 120.0f;

    // Use this for initialization
    void Start () {

        na = GetComponent<NewtonActuator>();
	}
	
	// Update is called once per frame
	void Update ()
    {
        if (na)
        {
            float angle = na.GetJointAngle();
            if (targetAngle - angle < 1.0f)
            {
                na.TargetAngle = targetAngle;
            }
            else
            {
                na.TargetAngle = -targetAngle;
            }
        }	
	}
}
