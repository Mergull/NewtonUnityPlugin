﻿using System;
using UnityEngine;
using System.Runtime.InteropServices;


[AddComponentMenu("Newton Physics/Joints/Hinge")]
public class NewtonHinge: NewtonJoint
{
    public override void Create()
    {
        Matrix4x4 localMatrix = Matrix4x4.identity;
        localMatrix.SetTRS(m_posit, Quaternion.Euler(m_rotation), Vector3.one);

        IntPtr floatsPtr = Marshal.AllocHGlobal(Marshal.SizeOf(localMatrix));
        Marshal.StructureToPtr(localMatrix, floatsPtr, false);
        NewtonBody child = GetComponent<NewtonBody>();
        if (m_otherBody == null)
        {
            m_joint = new dNewtonHinge(floatsPtr, child.GetBody().GetBody());
        }
        else
        {
            m_joint = new dNewtonHinge(floatsPtr, child.GetBody().GetBody(), m_otherBody.GetBody().GetBody());
        }

        Stiffness = m_stiffness;
        EnableLimits = m_enableLimits;
    }
/*
    void SetLimits ()
    {
        dNewtonHinge hinge = (dNewtonHinge)m_joint;
        hinge.SetLimits(m_enableLimits, m_minLimit, m_maxLimit);
    }

    void SetAsSpringDamper()
    {
        dNewtonHinge hinge = (dNewtonHinge)m_joint;
        hinge.SetAsSpringDamper(m_setSpringDamper, m_springDamperForceMixing, m_springConstant, m_damperConstant);
    }
*/
    void OnDrawGizmosSelected()
    {
        Matrix4x4 bodyMatrix = Matrix4x4.identity;
        Matrix4x4 localMatrix = Matrix4x4.identity;
        bodyMatrix.SetTRS(transform.position, transform.rotation, Vector3.one);
        localMatrix.SetTRS(m_posit, Quaternion.Euler(m_rotation), Vector3.one);

        Gizmos.color = Color.red;

        Gizmos.matrix = bodyMatrix;
        Vector3 direction = localMatrix.MultiplyPoint3x4(new Vector3(m_gizmoScale, 0.0f, 0.0f));

        Gizmos.DrawLine(m_posit, direction);
        if (m_enableLimits)
        {
            // draw hinge limit
        }
    }

    public bool EnableLimits
    {
        get
        {
            return m_enableLimits;
        }
        set
        {
            m_enableLimits = value;
            if (m_joint != null)
            {
                dNewtonHinge hinge = (dNewtonHinge)m_joint;
                hinge.SetLimits(m_enableLimits, m_minLimit, m_maxLimit);
            }
        }
    }

    public float MinimumLimit
    {
        get
        {
            return m_minLimit;
        }
        set
        {
            m_minLimit = value;
            if (m_joint != null)
            {
                dNewtonHinge hinge = (dNewtonHinge)m_joint;
                hinge.SetLimits(m_enableLimits, m_minLimit, m_maxLimit);
            }
        }
    }

    public float MaximunLimit
    {
        get
        {
            return m_maxLimit;
        }
        set
        {
            m_maxLimit = value;
            if (m_joint != null)
            {
                dNewtonHinge hinge = (dNewtonHinge)m_joint;
                hinge.SetLimits(m_enableLimits, m_minLimit, m_maxLimit);
            }
        }
    }

    public Vector3 m_posit = Vector3.zero;
    public Vector3 m_rotation = Vector3.zero;
    public bool m_enableLimits = false;
    public float m_minLimit = -30.0f;
    public float m_maxLimit =  30.0f;
    public bool m_setSpringDamper = false;
    public float m_springDamperForceMixing = 0.9f;
    public float m_springConstant = 0.0f;
    public float m_damperConstant = 10.0f;
}


[AddComponentMenu("Newton Physics/Joints/HingeActuator")]
public class NewtonActuator : NewtonJoint
{
    public override void Create()
    {
        Matrix4x4 localMatrix = Matrix4x4.identity;
        localMatrix.SetTRS(m_posit, Quaternion.Euler(m_rotation), Vector3.one);
        IntPtr floatsPtr = Marshal.AllocHGlobal(Marshal.SizeOf(localMatrix));
        Marshal.StructureToPtr(localMatrix, floatsPtr, false);
        NewtonBody child = GetComponent<NewtonBody>();
        if (m_otherBody == null)
        {
            m_joint = new dNewtonHingeActuator(floatsPtr, child.GetBody().GetBody());
        }
        else
        {
            m_joint = new dNewtonHingeActuator(floatsPtr, child.GetBody().GetBody(), m_otherBody.GetBody().GetBody());
        }


    }


    void OnDrawGizmosSelected()
    {
        Matrix4x4 bodyMatrix = Matrix4x4.identity;
        Matrix4x4 localMatrix = Matrix4x4.identity;
        bodyMatrix.SetTRS(transform.position, transform.rotation, Vector3.one);
        localMatrix.SetTRS(m_posit, Quaternion.Euler(m_rotation), Vector3.one);

        Gizmos.color = Color.red;

        Gizmos.matrix = bodyMatrix;
        Vector3 direction = localMatrix.MultiplyPoint3x4(new Vector3(m_gizmoScale, 0.0f, 0.0f));

        Gizmos.DrawLine(m_posit, direction);
    }

    public Vector3 m_posit = Vector3.zero;
    public Vector3 m_rotation = Vector3.zero;
    public float m_maxTorque = 10.0f;
    public float m_angulaVelocity = 1.0f;
    public float m_minAngle = -60.0f;
    public float m_maxAngle = 60.0f;
}



