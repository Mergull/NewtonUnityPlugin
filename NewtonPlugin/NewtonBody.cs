﻿/*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;
using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public delegate void OnContactCallback(IntPtr otherBody, IntPtr normal, float normalImpact, float penetration);

public interface NewtonUserData
{
}

[DisallowMultipleComponent]
[AddComponentMenu("Newton Physics/Rigid Body")]
public class NewtonBody : MonoBehaviour
{
    void Start()
    {
        //m_onContactCallback = new OnContactCallback(OnContact);

        var scripts = GetComponents<NewtonBodyScript>();
        foreach (var script in scripts)
        {
            m_scripts.Add(script);
            //script.m_onContactCallback = script.OnContact;
        }

        //if(m_body != null)m_body.SetCallbacks(m_onContactCallback);

        InitRigidBody();
    }

    public virtual void OnDestroy()
    {
        //Debug.Log("body");
        if (m_world != null)
            m_world.UnregisterBody(this);

        // Destroy native body
        DestroyRigidBody();
    }

    // Update is called once per frame
    public virtual void OnUpdateTranform()
    {
        IntPtr positionPtr = m_body.GetInterpolatedPosition();
        IntPtr rotationPtr = m_body.GetInterpolatedRotation();
        if(m_interpolate)
        {
            positionPtr = m_body.GetInterpolatedPosition();
            rotationPtr = m_body.GetInterpolatedRotation();
        }
        else
        {
            positionPtr = m_body.GetPosition();
            rotationPtr = m_body.GetRotation();
        }
        Marshal.Copy(positionPtr, m_positionPtr, 0, 3);
        Marshal.Copy(rotationPtr, m_rotationPtr, 0, 4);
        transform.position = new Vector3(m_positionPtr[0], m_positionPtr[1], m_positionPtr[2]);
        //transform.rotation = new Quaternion(m_rotationPtr[1], m_rotationPtr[2], m_rotationPtr[3], m_rotationPtr[0]);
        transform.rotation = new Quaternion(m_rotationPtr[0], m_rotationPtr[1], m_rotationPtr[2], m_rotationPtr[3]);
    }

    void OnDrawGizmosSelected()
    {
        if (m_showGizmo)
        {
            Matrix4x4 bodyMatrix = Matrix4x4.identity;
            bodyMatrix.SetTRS(transform.position, transform.rotation, Vector3.one);
            Gizmos.matrix = bodyMatrix;

            Gizmos.color = Color.red;
            Gizmos.DrawRay(m_centerOfMass, bodyMatrix.GetColumn(0) * m_gizmoScale);

            Gizmos.color = Color.green;
            Gizmos.DrawRay(m_centerOfMass, bodyMatrix.GetColumn(1) * m_gizmoScale);

            Gizmos.color = Color.blue;
            Gizmos.DrawRay(m_centerOfMass, bodyMatrix.GetColumn(2) * m_gizmoScale);
        }
    }

    public virtual void InitRigidBody()
    {
        if (initialized)
            return;

        m_onContactCallback = new OnContactCallback(OnContact);

        CreateBodyAndCollision();

        SetCenterOfMass();

        m_body.SetLinearDamping(m_linearDamping);
        m_body.SetAngularDamping(m_angularDamping.x, m_angularDamping.y, m_angularDamping.z);
        m_body.SetAutoSleep(m_autoSleep);
        m_body.SetContinuousCollisionMode(m_continuousCollision);

        var handle = GCHandle.Alloc(this);
        m_body.SetUserData(GCHandle.ToIntPtr(handle));
        m_body.SetCallbacks(m_onContactCallback);

        m_world.RegisterBody(this);
        initialized = true;
    }

    private void OnContact(IntPtr otherBody, IntPtr normal, float normalImpact, float penetration)
    {
        foreach (NewtonBodyScript script in m_scripts)
        {
            if (script.m_contactNotification)
            {
                float[] floats = new float[3];
                Marshal.Copy(normal, floats, 0, 3);
                Vector3 n = new Vector3(floats[0], floats[1], floats[2]);
                //dNewtonBody b = (dNewtonBody)Marshal.PtrToStructure(otherBody, typeof(dNewtonBody));
                var handle = GCHandle.FromIntPtr(otherBody);
                script.OnContact((NewtonBody)handle.Target, normalImpact, n, penetration);
            }
        }
    }

    void SetCenterOfMass ()
    {
        m_body.SetCenterOfMass(m_centerOfMass.x, m_centerOfMass.y, m_centerOfMass.z, m_Ixx, m_Iyy, m_Izz, m_CalculateInertia);
    }

    public virtual void DestroyRigidBody()
    {
        if (m_body != null)
        {
            var handle = GCHandle.FromIntPtr(m_body.GetUserData());
            m_body.SetUserData(IntPtr.Zero);
            handle.Free();

            m_body.Dispose();
            m_body = null;
        }

        if (m_collision != null)
        {
            m_collision.Destroy();
            m_collision = null;
        }
    }

    public dNewtonBody GetBody()
    {
        if (m_world.GetWorld() == null) { throw new NullReferenceException("Native world instance is null. The World component was probably destroyed"); }
        if (!initialized)
        {
            InitRigidBody();
        }
        return m_body;
    }

    public Vector4 GetMass()
    {
        IntPtr massPtr = IntPtr.Zero;

        massPtr = m_body.GetMass();

        return (Vector4)Marshal.PtrToStructure(massPtr, typeof(Vector4));
    }

    public Vector4 GetInvMass()
    {
        IntPtr massPtr = IntPtr.Zero;

        massPtr = m_body.GetInvMass();

        return (Vector4)Marshal.PtrToStructure(massPtr, typeof(Vector4));
    }

    public void CalculateBuoyancyForces(Vector4 plane, ref Vector3 force, ref Vector3 torque, float bodyDensity)
    {
        if(m_body != null)
        {
            IntPtr planePtr = Marshal.AllocHGlobal(Marshal.SizeOf(plane));
            IntPtr forcePtr = Marshal.AllocHGlobal(Marshal.SizeOf(force));
            IntPtr torquePtr = Marshal.AllocHGlobal(Marshal.SizeOf(torque));

            Marshal.StructureToPtr(plane, planePtr, false);

            m_body.CalculateBuoyancyForces(planePtr, forcePtr, torquePtr, bodyDensity);

            force = (Vector3)Marshal.PtrToStructure(forcePtr, typeof(Vector3));
            torque = (Vector3)Marshal.PtrToStructure(torquePtr, typeof(Vector3));

            Marshal.FreeHGlobal(planePtr);
            Marshal.FreeHGlobal(forcePtr);
            Marshal.FreeHGlobal(torquePtr);
        }
    }

    public void ApplyExternaForces ()
    {
        // Apply force & torque accumulators
        m_body.AddForce(m_forceAcc.x, m_forceAcc.y, m_forceAcc.z);
        m_body.AddTorque(m_torqueAcc.x, m_torqueAcc.y, m_torqueAcc.z);
        m_forceAcc = Vector3.zero;
        m_torqueAcc = Vector3.zero;
    }

    public Vector3 Position
    {
        get
        {
            if (m_body != null)
            {
                IntPtr positionPtr = m_body.GetPosition();
                Marshal.Copy(positionPtr, m_positionPtr, 0, 3);
                return new Vector3(m_positionPtr[0], m_positionPtr[1], m_positionPtr[2]);
            }
            return Vector3.zero;
        }

        set
        {
            if (m_body != null)
            {
                m_body.SetPosition(value.x, value.y, value.z);
            }
        }

    }

    public Quaternion Rotation
    {
        get
        {
            if (m_body != null)
            {
                IntPtr rotationPtr = m_body.GetRotation();
                Marshal.Copy(rotationPtr, m_rotationPtr, 0, 4);
                //return new Quaternion(m_rotationPtr[1], m_rotationPtr[2], m_rotationPtr[3], m_rotationPtr[0]);
                return new Quaternion(m_rotationPtr[0], m_rotationPtr[1], m_rotationPtr[2], m_rotationPtr[3]);
            }
            return Quaternion.identity;
        }

        set
        {
            if (m_body != null)
            {
                //m_body.SetRotation(value.z, value.w, value.x, value.y);
                m_body.SetRotation(value.x, value.y, value.z, value.w);
            }
        }
    }

    public Vector3 Velocity
    {
        get
        {
            if(m_body != null)
            {
                IntPtr velPtr = m_body.GetVelocity();
                Marshal.Copy(velPtr, m_vec3Ptr, 0, 3);
                return new Vector3(m_vec3Ptr[0], m_vec3Ptr[1], m_vec3Ptr[2]);
            }
            return Vector3.zero;
        }

        set
        {
            if (m_body != null)
            {
                m_body.SetVelocity(value.x, value.y, value.z);
            }
        }

    }

    public Vector3 Omega
    {
        get
        {
            if (m_body != null)
            {
                IntPtr omgPtr = m_body.GetOmega();
                Marshal.Copy(omgPtr, m_vec3Ptr, 0, 3);
                return new Vector3(m_vec3Ptr[0], m_vec3Ptr[1], m_vec3Ptr[2]);
            }
            return Vector3.zero;
        }

        set
        {
            if (m_body != null)
            {
                m_body.SetOmega(value.x,value.y,value.z);
            }
        }

    }

    public Vector3 CenterOfMass
    {
        get
        {
            if (m_body != null)
            {
                IntPtr comPtr = m_body.GetCenterOfMass();
                Marshal.Copy(comPtr, m_vec3Ptr, 0, 3);
                return new Vector3(m_vec3Ptr[0], m_vec3Ptr[1], m_vec3Ptr[2]);
            }
            return Vector3.zero;
        }

        set
        {
            if (m_body != null)
            {
                m_body.SetCenterOfMass(value.x, value.y, value.z, m_Ixx, m_Iyy, m_Izz, m_CalculateInertia);
            }
        }

    }

    public float LinearDamping
    {
        get
        {
            if (m_body != null)
            {
                m_linearDamping = m_body.GetLinearDamping();
            }
            return m_linearDamping;
        }

        set
        {
            m_linearDamping = value;
            if (m_body != null)
            {
                m_body.SetLinearDamping(value);
            }
        }
    }

    public Vector3 AngularDamping
    {
        get
        {
            if (m_body != null)
            {
                IntPtr dampingPtr = m_body.GetAngularDamping();
                Marshal.Copy(dampingPtr, m_vec3Ptr, 0, 3);
                m_angularDamping = new Vector3(m_vec3Ptr[0], m_vec3Ptr[1], m_vec3Ptr[2]);
            }
            return m_angularDamping;
        }

        set
        {
            m_angularDamping = new Vector3(value.x, value.y, value.z);
            if (m_body != null)
            {
                m_body.SetAngularDamping(value.x, value.y, value.z);
            }
        }

    }


    public bool SleepState
    {
        get
        {
            if(m_body != null)
                return m_body.GetSleepState();

            return false;
        }
        set
        {
            if (m_body != null)
            {
                m_body.SetSleepState(value);
            }
        }
    }

    public bool AutoSleep
    {
        get
        {
            if (m_body != null)
                return m_body.GetAutoSleep();

            return false;
        }
        set
        {
            if (m_body != null)
            {
                m_body.SetAutoSleep(value);
            }
        }
    }

    protected virtual void CreateBodyAndCollision()
    {
        if(m_collision == null && m_body == null)
        {
            m_collision = new NewtonBodyCollision(this);
            m_body = new dNewtonDynamicBody(m_world.GetWorld(), m_collision.GetShape(), Utils.ToMatrix(transform.position, transform.rotation), m_mass);
        }
    }

   /* void SetMaterial(NewtonMaterial material)
    {
        m_collision.SetMaterial(material);
    }*/

    [Header("rigid body data")]
    public float m_mass = 0.0f;
    public Vector3 m_centerOfMass = new Vector3 (0.0f, 0.0f, 0.0f);
    public float m_Ixx = 0.0f;
    public float m_Iyy = 0.0f;
    public float m_Izz = 0.0f;
    public bool m_CalculateInertia = true;
    public bool m_isScene = false;
    public bool m_showGizmo = false;
    public bool m_autoSleep = true;
    public bool m_continuousCollision = false;
    public bool m_interpolate = true;
    public float m_gizmoScale = 1.0f;
    public NewtonWorld m_world;
    public Vector3 m_forceAcc { get; set; }
    public Vector3 m_torqueAcc { get; set; }
    public NewtonUserData m_userData { get; set; }
    public float m_linearDamping = 0.1f;
    public Vector3 m_angularDamping = new Vector3(0.1f, 0.1f, 0.1f);

    internal dNewtonBody m_body = null;
    internal NewtonBodyCollision m_collision = null;
    private float[] m_positionPtr = new float[3];
    private float[] m_rotationPtr = new float[4];
    private float[] m_vec3Ptr = new float[3];
    private float[] m_comPtr = new float[3];

    private OnContactCallback m_onContactCallback;

    internal List<NewtonBodyScript> m_scripts = new List<NewtonBodyScript>();
    private bool initialized = false;
}
