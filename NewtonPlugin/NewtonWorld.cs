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

public delegate void OnWorldBodyTransfromUpdateCallback();
public delegate void OnWorldUpdateCallback(float timestep);

[StructLayout(LayoutKind.Sequential)]
internal struct _InternalRayHitInfo
{
    internal float intersectParam;
    internal int layermask;
    internal IntPtr body;
    internal IntPtr collider;
    internal Vector3 position;
    internal Vector3 normal;
    internal uint collisionID;
}

[StructLayout(LayoutKind.Sequential)]
internal struct _InternalConvexCastInfo
{
    internal Vector4 point;
    internal Vector4 normal;
    internal long contactID1;
    internal long contactID2;
    internal IntPtr hitBody;
    internal float penetration;
    internal float timeOfImpact;
}

public struct NewtonRayHitInfo
{
    public NewtonBody body;
    //public NewtonCollider collider;
    public Vector3 position;
    public Vector3 normal;
    public uint collisionID;

}

public struct NewtonCollideInfo
{
    public NewtonBody body;
    public uint collisionID1;
    public uint collisionID2;
    public Vector3 position;
    public Vector3 normal;
    public float penetration;
    public float timeOfImpact;
}


[DisallowMultipleComponent]
[AddComponentMenu("Newton Physics/Newton World")]
public class NewtonWorld : MonoBehaviour
{
    public dNewtonWorld GetWorld()
    {
        return m_world;
    }

    void Start()
    {
        m_onWorldCallback = new OnWorldUpdateCallback(OnWorldUpdate);
        m_onWorldBodyTransfromUpdateCallback = new OnWorldBodyTransfromUpdateCallback(OnBodyTransformUpdate);

        m_world.SetAsyncUpdate(m_asyncUpdate);
        m_world.SetFrameRate(m_updateRate);
        m_world.SetThreadsCount(m_numberOfThreads);
        m_world.SetSolverIterations(m_solverIterationsCount);
        m_world.SetMaxIterations(m_maxIterations);
        m_world.SetBroadPhase(m_broadPhaseType);
        m_world.SetGravity(m_gravity.x, m_gravity.y, m_gravity.z);
        m_world.SetSubSteps(m_subSteps);
        m_world.SetParallelSolverOnLargeIsland(m_useParallerSolver);
        m_world.SetDefaultMaterial(m_defaultRestitution, m_defaultStaticFriction, m_defaultKineticFriction, true);
        m_world.SetCallbacks(m_onWorldCallback, m_onWorldBodyTransfromUpdateCallback);

        //Load all physics plug ins and choose the best one
        m_world.SelectPlugin(IntPtr.Zero);
        if (m_useParallerSolver && (m_pluginsOptions > 0))
        {
            string path = Application.dataPath;
            m_world.LoadPlugins(path);
            int index = 1;
            for (IntPtr plugin = m_world.FirstPlugin(); plugin != IntPtr.Zero; plugin = m_world.NextPlugin(plugin))
            {
                if (index == m_pluginsOptions)
                {
                    Debug.Log("Using newton physics solver: " + m_world.GetPluginName(plugin));
                    m_world.SelectPlugin(plugin);
                }
                index++;
            }
        } else {
            m_world.UnloadPlugins();
        }

        InitScene();
    }

    void OnDestroy()
    {
        DestroyScene();
        m_onWorldCallback = null;
        m_onWorldBodyTransfromUpdateCallback = null;

    }

    internal void RegisterBody(NewtonBody nb)
    {
        if (!m_bodies.Contains(nb))     // Had to add this
        {
            m_bodies.Add(nb);
        }
    }

    internal void UnregisterBody(NewtonBody nb)
    {
        m_bodies.Remove(nb);
    }

    //private void InitPhysicsScene(GameObject root)
    //{
    //    NewtonBody bodyPhysics = root.GetComponent<NewtonBody>();
    //    if (bodyPhysics != null)
    //    {
    //        bodyPhysics.InitRigidBody();
    //    }

    //    foreach (Transform child in root.transform)
    //    {
    //        InitPhysicsScene(child.gameObject);
    //    }
    //}

    //private void InitPhysicsJoints(GameObject root)
    //{
    //    foreach (NewtonJoint joint in root.GetComponents<NewtonJoint>())
    //    {
    //        joint.Create();
    //    }

    //    foreach (Transform child in root.transform)
    //    {
    //        InitPhysicsJoints(child.gameObject);
    //    }
    //}

    private void InitScene()
    {
        /*Resources.LoadAll("Newton Materials", typeof(NewtonMaterialInteraction));

        foreach (String path in m_materialsPaths)
        {
            Resources.LoadAll(path, typeof(NewtonMaterialInteraction));
        }

        NewtonMaterialInteraction[] materialList = Resources.FindObjectsOfTypeAll<NewtonMaterialInteraction>();
        foreach (NewtonMaterialInteraction materialInteraction in materialList)
        {
            // register all material interactions.
            if (materialInteraction.m_material_0 && materialInteraction.m_material_1)
            {
                int id0 = materialInteraction.m_material_0.GetInstanceID();
                int id1 = materialInteraction.m_material_1.GetInstanceID();
                m_world.SetMaterialInteraction(id0, id1, materialInteraction.m_restitution, materialInteraction.m_staticFriction, materialInteraction.m_kineticFriction, materialInteraction.m_collisionEnabled);
            }
        }*/

        foreach (String path in m_materialsPaths)
        {
            NewtonMaterialInteraction[] materialList = Resources.LoadAll<NewtonMaterialInteraction>(path);

            foreach (NewtonMaterialInteraction materialInteraction in materialList)
            {
                // register all material interactions.
                if (materialInteraction.m_material_0 && materialInteraction.m_material_1)
                {
                    int id0 = materialInteraction.m_material_0.GetInstanceID();
                    int id1 = materialInteraction.m_material_1.GetInstanceID();

                    m_world.SetMaterialInteraction(id0, id1, materialInteraction.m_restitution, materialInteraction.m_staticFriction, materialInteraction.m_kineticFriction, materialInteraction.m_collisionEnabled);
                }
            }
        }

        //GameObject[] objectList = gameObject.scene.GetRootGameObjects();
        //foreach (GameObject rootObj in objectList)
        //{
        //    InitPhysicsScene(rootObj);
        //}
        //foreach (GameObject rootObj in objectList)
        //{
        //    InitPhysicsJoints(rootObj);
        //}
    }

    private void DestroyScene()
    {
        if (m_world != null)
        {
            foreach (NewtonBody nb in m_bodies)
            {
                nb.DestroyRigidBody();
            }

            m_world = null;
        }
    }

    void Update()
    {
        //Debug.Log("Update time :" + Time.deltaTime);
        if (m_serializeSceneOnce)
        {
            m_serializeSceneOnce = false;
            m_world.SaveSerializedScene(m_saveSceneName);
        }

        m_world.Update(Time.deltaTime);
    }

    private void OnWorldUpdate(float timestep)
    {
        foreach (NewtonBody bodyPhysics in m_bodies)
        {
            // Apply force & torque accumulators
            bodyPhysics.ApplyExternaForces();

            foreach (NewtonBodyScript script in bodyPhysics.m_scripts)
            {
                if (script.m_collisionNotification)
                {
                    for (IntPtr contact = m_world.GetFirstContactJoint(bodyPhysics.m_body); contact != IntPtr.Zero; contact = m_world.GetNextContactJoint(bodyPhysics.m_body, contact))
                    {
                        IntPtr user_data = m_world.GetBody0UserData(contact);
                        if (user_data == IntPtr.Zero) continue;
                        var body0 = (NewtonBody)GCHandle.FromIntPtr(user_data).Target;
                        user_data = m_world.GetBody1UserData(contact);
                        if (user_data == IntPtr.Zero) continue;
                        var body1 = (NewtonBody)GCHandle.FromIntPtr(user_data).Target;
                        var otherBody = bodyPhysics == body0 ? body1 : body0;
                        script.OnCollision(otherBody);

                        /*if(script.m_contactNotification)
                        {
                            for (IntPtr ct = m_world.GetFirstContact(contact); ct != IntPtr.Zero; ct = m_world.GetNextContact(contact, ct))
                            {
                                //var normImpact = dNewtonContact.GetContactNormalImpact(ct);]
                                IntPtr info = dNewtonContact.GetContactInfo(bodyPhysics.GetBody().GetBody(), ct);
                                float[] normImpact = new float[22];
                                Marshal.Copy(info, normImpact, 0, 22);
                                script.OnContact(otherBody, normImpact[18]);
                            }
                        }*/

                        script.OnPostCollision(otherBody);
                    }
                }

                // apply external force and torque if any
                if (script.m_enableForceAndTorque)
                {
                    script.OnApplyForceAndTorque(timestep);
                }
            }
        }
    }

    public int GetUpdateStepsCount()
    {
        return m_world.GetUpdateStepsCount();
    }

    private void OnBodyTransformUpdate()
    {
        foreach(NewtonBody bodyPhysics in m_bodies)
        {
            bodyPhysics.OnUpdateTranform();
        }
    }

    public bool Raycast(Vector3 origin, Vector3 direction, float distance, out NewtonRayHitInfo hitInfo, int layerMask = 0)
    {
        Vector3 startPos = origin;
        Vector3 endPos = startPos + (direction * distance);

        var hitInfoPtr = m_world.Raycast(startPos.x, startPos.y, startPos.z, endPos.x, endPos.y, endPos.z, layerMask);
        if(hitInfoPtr != IntPtr.Zero)
        {
            _InternalRayHitInfo info = (_InternalRayHitInfo)Marshal.PtrToStructure(hitInfoPtr, typeof(_InternalRayHitInfo));

            if(info.body != IntPtr.Zero)
            {
                hitInfo.body = (NewtonBody)GCHandle.FromIntPtr(info.body).Target;
            }
            else
            {
                hitInfo.body = null;
            }

            //hitInfo.collider = null;
            hitInfo.position = info.position;
            hitInfo.normal = info.normal;
            hitInfo.collisionID = info.collisionID;
            return true;
        }

        hitInfo.body = null;
        //hitInfo.collider = null;
        hitInfo.position = Vector3.zero;
        hitInfo.normal = Vector3.zero;
        hitInfo.collisionID = 0;
        return false;
    }

    public bool Collide(/*Matrix4x4 matrix,*/ NewtonBody body, out NewtonCollideInfo hitInfo, int layerMask = 0)
    {
        //dMatrix matrix = Utils.ToMatrix(body.transform.position, body.transform.rotation);
        //GCHandle mat_handle = GCHandle.Alloc(matrix);
        //dVector vec = new dVector(transform.position.x, transform.position.y, transform.position.z);
        //dQuaternion quat = new dQuaternion(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        dMatrix matrix = Utils.ToMatrix(body.transform.position, body.transform.rotation);
        var hitInfoPtr = m_world.Collide(matrix, body.m_collision.GetShape(), layerMask);// m_world.Raycast(startPos.x, startPos.y, startPos.z, endPos.x, endPos.y, endPos.z, layerMask);
        //mat_handle.Free();
        if (hitInfoPtr != IntPtr.Zero)
        {
            _InternalConvexCastInfo info = (_InternalConvexCastInfo)Marshal.PtrToStructure(hitInfoPtr, typeof(_InternalConvexCastInfo));

            if (info.hitBody != IntPtr.Zero)
            {
                hitInfo.body = (NewtonBody)GCHandle.FromIntPtr(info.hitBody).Target;
            }
            else
            {
                hitInfo.body = null;
            }

            //hitInfo.collider = null;
            hitInfo.position = info.point;
            hitInfo.normal = info.normal;
            //hitInfo.collisionID = 0;
            hitInfo.collisionID1 = 0;
            hitInfo.collisionID2 = (uint)info.contactID2;
            hitInfo.penetration = info.penetration;
            hitInfo.timeOfImpact = 0;
            return true;
        }

        hitInfo.body = null;
        //hitInfo.collider = null;
        hitInfo.position = Vector3.zero;
        hitInfo.normal = Vector3.zero;
        //hitInfo.collisionID = 0;
        hitInfo.collisionID1 = 0;
        hitInfo.collisionID2 = 0;
        hitInfo.penetration = 0;
        hitInfo.timeOfImpact = 0;
        return false;
    }

    public bool Collide(NewtonBody body1, NewtonBody body2, out NewtonCollideInfo hitInfo)
    {
        dMatrix matrix1 = Utils.ToMatrix(body1.Position, body1.Rotation);
        dMatrix matrix2 = Utils.ToMatrix(body2.Position, body2.Rotation);

        var hitInfoPtr = m_world.Collide(matrix1, matrix2, body1.m_collision.GetShape(), body2.m_collision.GetShape(), 1);// m_world.Raycast(startPos.x, startPos.y, startPos.z, endPos.x, endPos.y, endPos.z, layerMask);
        //mat_handle.Free();
        if (hitInfoPtr != IntPtr.Zero)
        {
            _InternalConvexCastInfo info = (_InternalConvexCastInfo)Marshal.PtrToStructure(hitInfoPtr, typeof(_InternalConvexCastInfo));

            hitInfo.body = body2;

            hitInfo.position = info.point;
            hitInfo.normal = info.normal;
            hitInfo.collisionID1 = (uint)info.contactID1;
            hitInfo.collisionID2 = (uint)info.contactID2;
            hitInfo.penetration = info.penetration;
            hitInfo.timeOfImpact = 0;
            return true;
        }

        hitInfo.body = null;
        hitInfo.position = Vector3.zero;
        hitInfo.normal = Vector3.zero;
        hitInfo.collisionID1 = 0;
        hitInfo.collisionID2 = 0;
        hitInfo.penetration = 0;
        hitInfo.timeOfImpact = 0;
        return false;
    }

    public bool ContinuousCollide(NewtonBody body1, NewtonBody body2, out NewtonCollideInfo hitInfo)
    {
        dMatrix matrix1 = Utils.ToMatrix(body1.Position, body1.Rotation);
        dMatrix matrix2 = Utils.ToMatrix(body2.Position, body2.Rotation);
        dVector velocity1 = new dVector(body1.Velocity.x, body1.Velocity.y, body1.Velocity.z);
        dVector velocity2 = new dVector(body2.Velocity.x, body2.Velocity.y, body2.Velocity.z);
        dVector omega1 = new dVector(body1.Omega.x, body1.Omega.y, body1.Omega.z);
        dVector omega2 = new dVector(body2.Omega.x, body2.Omega.y, body2.Omega.z);

        var hitInfoPtr = m_world.ContinuousCollide(matrix1, matrix2, velocity1, velocity2, omega1, omega2, body1.m_collision.GetShape(), body2.m_collision.GetShape(), 1);// m_world.Raycast(startPos.x, startPos.y, startPos.z, endPos.x, endPos.y, endPos.z, layerMask);
        //mat_handle.Free();
        if (hitInfoPtr != IntPtr.Zero)
        {
            _InternalConvexCastInfo info = (_InternalConvexCastInfo)Marshal.PtrToStructure(hitInfoPtr, typeof(_InternalConvexCastInfo));

            hitInfo.body = body2;

            hitInfo.position = info.point;
            hitInfo.normal = info.normal;
            hitInfo.collisionID1 = (uint)info.contactID1;
            hitInfo.collisionID2 = (uint)info.contactID2;
            hitInfo.penetration = info.penetration;
            hitInfo.timeOfImpact = info.timeOfImpact;
            return true;
        }

        hitInfo.body = null;
        hitInfo.position = Vector3.zero;
        hitInfo.normal = Vector3.zero;
        hitInfo.collisionID1 = 0;
        hitInfo.collisionID2 = 0;
        hitInfo.penetration = 0;
        hitInfo.timeOfImpact = 0;
        return false;
    }

    private dNewtonWorld m_world = new dNewtonWorld();
    public bool m_asyncUpdate = true;
    public bool m_serializeSceneOnce = false;
    public bool m_useParallerSolver = true;
    public string m_saveSceneName = "scene_01.bin";
    public int m_broadPhaseType = 0;
    public int m_numberOfThreads = 0;
    public int m_solverIterationsCount = 1;
    public int m_updateRate = 60;
    public int m_subSteps = 2;
    public int m_pluginsOptions = 0;
    public int m_maxIterations = 7;
    public string[] m_materialsPaths = new string[] { "Newton Materials" };

    public Vector3 m_gravity = new Vector3(0.0f, -9.8f, 0.0f);

    public float m_defaultRestitution = 0.4f;
    public float m_defaultStaticFriction = 0.8f;
    public float m_defaultKineticFriction = 0.6f;

    private OnWorldUpdateCallback m_onWorldCallback;
    private OnWorldBodyTransfromUpdateCallback m_onWorldBodyTransfromUpdateCallback;

    private List<NewtonBody> m_bodies = new List<NewtonBody>();
}


