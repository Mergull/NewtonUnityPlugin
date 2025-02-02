﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StressTest : MonoBehaviour
{

    public NewtonWorld world = null;
    public int BodyCount = 200;
    public float Delay = 1.0f;
    public int Radius = 50;

    private GameObject groupContainer = null;
    private float timer = 0;

    // Use this for initialization
    void Start()
    {

        CreateBodies();

    }

    // Update is called once per frame
    void Update()
    {

        timer += Time.deltaTime;

        if (timer > Delay)
        {
            if (groupContainer)
            {
                Destroy(groupContainer);
                groupContainer = null;

                CreateBodies();
                timer = 0;
            }
        }
    }

    void CreateBodies()
    {
        groupContainer = new GameObject("GroupContainer");

        for (var i = 0; i < BodyCount; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.transform.position = Random.insideUnitSphere * Radius + new Vector3(0, Radius, 0);

            var body = go.AddComponent<NewtonBody>();
            body.m_mass = 1.0f;
            body.m_world = world;

            var coll = go.AddComponent<NewtonBoxCollider>();
            coll.m_size = new Vector3(1, 1, 1);

            go.transform.SetParent(groupContainer.transform);
        }
    }

}

