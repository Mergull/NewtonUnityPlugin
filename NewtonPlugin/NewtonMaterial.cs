/*
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

[StructLayout(LayoutKind.Sequential)]
public struct MaterialProperties
{
    public float m_restitution;
    public float m_staticFriction;
    public float m_kineticFriction;
    public bool m_collisionEnable;
}

[StructLayout(LayoutKind.Sequential)]
public struct MaterialContactInfo
{
    public Vector3 m_point;
    internal float m_pointW;
    public Vector3 m_normal;
    internal float m_normalW;
    public long m_shapeId0;
    public long m_shapeId1;
    public float m_penetration;
    //internal int m_unused;
}

[CreateAssetMenu(menuName = "Newton Material")]
public class NewtonMaterial : ScriptableObject
{
}

public delegate void UserOnMaterialInteractionCallback(ref MaterialProperties properites, MaterialContactInfo contact, float normalImpact, NewtonBody body0, NewtonBody body1);

[CreateAssetMenu(menuName = "Newton Material Interaction")]
public class NewtonMaterialInteraction : ScriptableObject
{
    public NewtonMaterial m_material_0 = null;
    public NewtonMaterial m_material_1 = null;
    public float m_restitution = 0.3f;
    public float m_staticFriction = 0.9f; 
    public float m_kineticFriction = 0.75f;
    public bool m_collisionEnabled = true;
    public UserOnMaterialInteractionCallback m_callback;
    public OnMaterialInteractionCallback m_onMaterial;

    public void OnInteraction(IntPtr properties, IntPtr body0, IntPtr body1, IntPtr contact, float normalImpact)
    {
        MaterialContactInfo contactInfo = (MaterialContactInfo)Marshal.PtrToStructure(contact, typeof(MaterialContactInfo));
        MaterialProperties properties_ = (MaterialProperties)Marshal.PtrToStructure(properties, typeof(MaterialProperties));
        //NewtonBody body0_ = (NewtonBody)Marshal.PtrToStructure(body0, typeof(NewtonBody));
        //NewtonBody body1_ = (NewtonBody)Marshal.PtrToStructure(body1, typeof(NewtonBody));
        NewtonBody body0_ = (NewtonBody)GCHandle.FromIntPtr(body0).Target;
        NewtonBody body1_ = (NewtonBody)GCHandle.FromIntPtr(body1).Target;
        //Debug.Log("On interaction");
        if (m_callback != null) m_callback(ref properties_, contactInfo, normalImpact, body0_, body1_);

        Marshal.StructureToPtr(properties_, properties, false);
    }
}


