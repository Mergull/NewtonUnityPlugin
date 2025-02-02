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

#include "stdafx.h"
#include "dAlloc.h"
#include "dNewtonBody.h"
#include "dNewtonWorld.h"
#include "dNewtonCollision.h"
#include "dNewtonVehicleManager.h"
#include "dVector.h"

#define D_DEFAULT_FPS 120.0f

dNewtonWorld::dNewtonWorld()
	:dAlloc()
	,m_world (NewtonCreate())
	,m_collisionCache()
	,m_materialGraph()
	,m_realTimeInMicroSeconds(0)
	,m_timeStepInMicroSeconds (0)
	,m_timeStep(0.0f)
	,m_interpotationParam(0.0f)
	,m_gravity(0.0f, 0.0f, 0.0f, 0.0f)
	,m_asyncUpdateMode(true)
	,m_onUpdateCallback(NULL)
	,m_vehicleManager(NULL)
{
	// for two way communication between low and high lever, link the world with this class for 
	NewtonWorldSetUserData(m_world, this);

	// set the simplified solver mode (faster but less accurate)
	NewtonSetSolverIterations(m_world, 1);

	// enable parallel solve on large island
	NewtonSetParallelSolverOnLargeIsland(m_world, 1);

/*
	// by default runs on four micro threads
	NewtonSetThreadsCount(m_world, 4);

	// set the collision copy constructor callback
	NewtonSDKSetCollisionConstructorDestructorCallback (m_world, OnCollisionCopyConstruct, OnCollisionDestructorCallback);

	// add a hierarchical transform manage to update local transforms
	new NewtonSDKTransformManager (this);
*/

	// use default material to implement traditional "Game style" one side material system
	int defaultMaterial = NewtonMaterialGetDefaultGroupID(m_world);
	NewtonMaterialSetCallbackUserData(m_world, defaultMaterial, defaultMaterial, this);
	NewtonMaterialSetCompoundCollisionCallback(m_world, defaultMaterial, defaultMaterial, OnSubShapeAABBOverlapTest);
	NewtonMaterialSetCollisionCallback(m_world, defaultMaterial, defaultMaterial, OnBodiesAABBOverlap, OnContactCollision);

	// set joint serialization call back
	dCustomJoint::Initalize(m_world);
	SetFrameRate(D_DEFAULT_FPS);

	// create a vehicle controller manage for all vehicles.
	int materialList[] = { defaultMaterial };
	// create a vehicle controller manager
	//m_vehicleManager = new dNewtonVehicleManager(m_world, 1, materialList);
}

dNewtonWorld::~dNewtonWorld()
{
	NewtonWaitForUpdateToFinish (m_world);

	/*if (m_vehicleManager) {
		while (m_vehicleManager->GetFirst()) {
			dCustomControllerManager<dCustomVehicleController>::dListNode* const node = m_vehicleManager->GetFirst();
			m_vehicleManager->DestroyController(&node->GetInfo());
		}
	}
	m_vehicleManager = NULL;*/

	dList<dNewtonCollision*>::dListNode* next;
	for (dList<dNewtonCollision*>::dListNode* node = m_collisionCache.GetFirst(); node; node = next) {
		next = node->GetNext();
		node->GetInfo()->DeleteShape();
	}

	if (m_world) {
		NewtonDestroy(m_world);
		m_world = NULL;
	}
}

int dNewtonWorld::GetUpdateStepsCount()
{
	return m_stepsCount;
}

dLong dNewtonWorld::GetMaterialKey(int materialID0, int materialID1) const
{
	if (materialID0 > materialID1) {
		dSwap(materialID0, materialID1);
	}
	return ((dLong)(materialID1) << 32) + (dLong)(materialID0);
}

void dNewtonWorld::SetCallbacks(OnWorldUpdateCallback forceCallback, OnWorldBodyTransfromUpdateCallback tranformCallback)
{
	m_onUpdateCallback = forceCallback;
	m_onTransformCallback = tranformCallback;
}

const dNewtonWorld::dMaterialProperties& dNewtonWorld::FindMaterial(int id0, int id1) const
{
	long long key = GetMaterialKey(id0, id1);
	dTree<dMaterialProperties, long long>::dTreeNode* const node = m_materialGraph.Find(key);
	if (node) {
		return node->GetInfo();
	}
	return m_defaultMaterial;
}

void dNewtonWorld::SetDefaultMaterial(float restitution, float staticFriction, float kineticFriction, bool collisionEnable)
{
	m_defaultMaterial.m_restitution = restitution;
	m_defaultMaterial.m_staticFriction = staticFriction;
	m_defaultMaterial.m_kineticFriction = kineticFriction;
	m_defaultMaterial.m_collisionEnable = collisionEnable;
}

void* dNewtonWorld::Raycast(float p0x, float p0y, float p0z, float p1x, float p1y, float p1z, int layerMask)
{
	dVector p0(p0x, p0y, p0z);
	dVector p1(p1x, p1y, p1z);

	hitInfo.clearData();
	hitInfo.layermask = layerMask;
	NewtonWorldRayCast(m_world, &p0.m_x, &p1.m_x, &rayFilterCallback, &hitInfo, &rayPreFilterCallback, 0);
	if (hitInfo.intersectParam < 1.0f)
	{
		return &hitInfo;
	}
	else
	{
		return nullptr;
	}
}

void* dNewtonWorld::Collide(const dMatrix matrix, const dNewtonCollision* shape, int layerMask)
{
	//NewtonWorldConvexCastReturnInfo info;
	hitInfo.clearData();
	hitInfo.layermask = layerMask;
	NewtonWorldConvexCastReturnInfo retInfo = NewtonWorldConvexCastReturnInfo();
	//dMatrix matrixx = dMatrix(rotation,position);
	//if (shape->m_shape == nullptr)return nullptr;
	if (NewtonWorldCollide(m_world, &matrix[0][0], shape->m_shape, &hitInfo, &rayPreFilterCallback, &retInfo, 1, 0))
	{
		collideInfo.point[0] = retInfo.m_point[0];
		collideInfo.point[1] = retInfo.m_point[1];
		collideInfo.point[2] = retInfo.m_point[2];
		collideInfo.normal[0] = retInfo.m_normal[0];
		collideInfo.normal[1] = retInfo.m_normal[1];
		collideInfo.normal[2] = retInfo.m_normal[2];
		collideInfo.penetration = retInfo.m_penetration;
		dNewtonBody* dBody = static_cast<dNewtonBody*>(NewtonBodyGetUserData(retInfo.m_hitBody));
		if (dBody == nullptr)collideInfo.managedBodyHandle;
		else collideInfo.managedBodyHandle = dBody->GetUserData();
		//collideInfo.body = retInfo.m_hitBody;
		collideInfo.contact_id2 = retInfo.m_contactID;
		return &collideInfo;
	}
	else return nullptr;
}


void* dNewtonWorld::Collide(const dMatrix matrix1, const dMatrix matrix2, dNewtonCollision* collision1, dNewtonCollision* collision2, int max_contacts)
{
	dVector contacts;
	dVector normal;
	dFloat penetration;
	dLong contact_id1;
	dLong contact_id2;

	if (NewtonCollisionCollide(m_world, 1, collision1->m_shape, &matrix1[0][0], collision2->m_shape, &matrix2[0][0], &contacts[0], &normal[0], &penetration, &contact_id1, &contact_id2, 0))
	{
		collideInfo.point[0] = contacts[0];
		collideInfo.point[1] = contacts[1];
		collideInfo.point[2] = contacts[2];
		collideInfo.normal[0] = normal[0];
		collideInfo.normal[1] = normal[1];
		collideInfo.normal[2] = normal[2];
		collideInfo.penetration = penetration;
		/*dNewtonBody* dBody = static_cast<dNewtonBody*>(NewtonBodyGetUserData(collideInfo.m_hitBody));
		collideInfo.managedBodyHandle = dBody->GetUserData();*/
		collideInfo.managedBodyHandle = nullptr;
		//collideInfo.body = collideInfo.m_hitBody;
		collideInfo.contact_id1 = contact_id1;
		collideInfo.contact_id2 = contact_id2;
		collideInfo.timeOfImpact = 0;
		return &collideInfo;
	}
	return nullptr;
}

void* dNewtonWorld::ContinuousCollide(const dMatrix matrix1, const dMatrix matrix2, const dVector veloctiy1, const dVector velocity2, const dVector omega1, const dVector omega2, dNewtonCollision* collision1, dNewtonCollision* collision2, int max_contacts)
{
	dFloat timeOfImpact;
	dVector contacts;
	dVector normal;
	dFloat penetration;
	dLong contact_id1;
	dLong contact_id2;

	if (NewtonCollisionCollideContinue(m_world, 1, m_timeStep, collision1->m_shape, &matrix1[0][0], &veloctiy1[0], &omega1[0], collision2->m_shape, &matrix2[0][0], &velocity2[0], &omega2[0], &timeOfImpact, &contacts[0], &normal[0], &penetration, &contact_id1, &contact_id2, 0))
	{
		collideInfo.point[0] = contacts[0];
		collideInfo.point[1] = contacts[1];
		collideInfo.point[2] = contacts[2];
		collideInfo.normal[0] = normal[0];
		collideInfo.normal[1] = normal[1];
		collideInfo.normal[2] = normal[2];
		collideInfo.penetration = penetration;
		/*dNewtonBody* dBody = static_cast<dNewtonBody*>(NewtonBodyGetUserData(collideInfo.m_hitBody));
		collideInfo.managedBodyHandle = dBody->GetUserData();*/
		collideInfo.managedBodyHandle = nullptr;
		//collideInfo.body = collideInfo.m_hitBody;
		collideInfo.contact_id1 = contact_id1;
		collideInfo.contact_id2 = contact_id2;
		collideInfo.timeOfImpact = timeOfImpact;
		return &collideInfo;
	}
	return nullptr;
}

void* dNewtonWorld::ConvexCast(const dFloat* const matrix, const dFloat* const target, dNewtonCollision* const collision, int layerMask, int max_contacts)
{
	hitInfo.clearData();
	hitInfo.layermask = layerMask;
	//collideInfo.clearData();
	NewtonWorldConvexCastReturnInfo ret_info = NewtonWorldConvexCastReturnInfo();
	if (NewtonWorldConvexCast(m_world, matrix, target, collision->m_shape, nullptr, &hitInfo, &rayPreFilterCallback, &ret_info, 1, 0))
	{
		//collideInfo.point[0] = ret_info.m_point[0];
		return &collideInfo;
	}
	else return nullptr;
}

float dNewtonWorld::rayFilterCallback(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam)
{
	rayHitInfo* hitInfo = static_cast<rayHitInfo*>(userData);

	if (intersectParam < hitInfo->intersectParam)
	{
		if (body)
		{
			dNewtonBody* dBody = static_cast<dNewtonBody*>(NewtonBodyGetUserData(body));
			hitInfo->managedBodyHandle = dBody->GetUserData();
		}
		else {
			hitInfo->managedBodyHandle = nullptr;
		}

		hitInfo->intersectParam = intersectParam;
		hitInfo->collider = shapeHit;
		hitInfo->position[0] = hitContact[0];
		hitInfo->position[1] = hitContact[1];
		hitInfo->position[2] = hitContact[2];
		hitInfo->normal[0] = hitNormal[0];
		hitInfo->normal[1] = hitNormal[1];
		hitInfo->normal[2] = hitNormal[2];
		hitInfo->collisionID = collisionID;
	}

	return intersectParam;
}

unsigned dNewtonWorld::rayPreFilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	rayHitInfo* hitInfo = static_cast<rayHitInfo*>(userData);

	if (collision)
	{
		dNewtonCollision* dCol = static_cast<dNewtonCollision*>(NewtonCollisionGetUserData(collision));
		int layer = dCol->m_layer;
		
		if (layer & hitInfo->layermask) {
			return 0;
		}

		return 1;
	}

	return 1;
}

void dNewtonWorld::SetMaterialInteraction(int materialID0, int materialID1, float restitution, float staticFriction, float kineticFriction, bool collisionEnable)
{
	long long key = GetMaterialKey(materialID0, materialID1);
	dTree<dMaterialProperties, long long>::dTreeNode* node = m_materialGraph.Find(key);
	if (!node) {
		node = m_materialGraph.Insert(key);
	}
	dMaterialProperties& material = node->GetInfo();
	material.m_restitution = restitution;
	material.m_staticFriction = staticFriction;
	material.m_kineticFriction = kineticFriction;
	material.m_collisionEnable = collisionEnable;
	material.m_callback = nullptr;
	material.m_aabb_overlap_callback = nullptr;
}

void dNewtonWorld::SetMaterialInteractionCallback(int materialID0, int materialID1, OnMaterialInteractionCallback callback)
{
	long long key = GetMaterialKey(materialID0, materialID1);
	dTree<dMaterialProperties, long long>::dTreeNode* node = m_materialGraph.Find(key);
	if (!node) {
		node = m_materialGraph.Insert(key);
	}
	dMaterialProperties& material = node->GetInfo();
	material.m_callback = callback;
}

void dNewtonWorld::SetMaterialAABBOverlapCallback(int materialID0, int materialID1, OnMaterialAABBOverlapCallback callback)
{
	long long key = GetMaterialKey(materialID0, materialID1);
	dTree<dMaterialProperties, long long>::dTreeNode* node = m_materialGraph.Find(key);
	if (!node) {
		node = m_materialGraph.Insert(key);
	}
	dMaterialProperties& material = node->GetInfo();
	material.m_aabb_overlap_callback = callback;
}

void dNewtonWorld::SetFrameRate(dFloat frameRate)
{
	m_timeStep = 1.0f / frameRate;
	m_realTimeInMicroSeconds = 0;
	m_timeStepInMicroSeconds = (dLong)(1000000.0 / double(frameRate));
}

void dNewtonWorld::SetSubSteps(int subSteps)
{
	NewtonSetNumberOfSubsteps(m_world, dClamp(subSteps, 1, 16));
}

void dNewtonWorld::SetSolverIterations(int mode)
{
	NewtonSetSolverIterations(m_world, dClamp (mode, 1, 1000));
}

void dNewtonWorld::SetMaxIterations(int count)
{
	m_maxInterations = dClamp(count, 1, 100);
}

void dNewtonWorld::SetThreadsCount(int threads)
{
	NewtonSetThreadsCount(m_world, dClamp (threads, 0, 8));
}

void dNewtonWorld::SetBroadPhase(int broadphase)
{
	NewtonSelectBroadphaseAlgorithm(m_world, broadphase ? 0 : 1);
}

void dNewtonWorld::SetParallelSolverOnLargeIsland(bool mode)
{
	NewtonSetParallelSolverOnLargeIsland(m_world, mode ? 1 : 0);
}

dNewtonVehicleManager* dNewtonWorld::GetVehicleManager() const
{
	return m_vehicleManager;
}

const dVector& dNewtonWorld::GetGravity() const
{
	return m_gravity;
}

void dNewtonWorld::SetGravity(const dVector& gravity)
{
	m_gravity = gravity;
	m_gravity.m_w = 0.0f;
}

void dNewtonWorld::SetGravity(dFloat x, dFloat y, dFloat z)
{
	SetGravity(dVector(x, y, z, 0.0f));
}

void dNewtonWorld::SetAsyncUpdate(bool updateMode)
{
	m_asyncUpdateMode = updateMode;
}

dNewtonBody* dNewtonWorld::GetFirstBody() const
{
	NewtonBody* const body = NewtonWorldGetFirstBody(m_world);
	return body ? (dNewtonBody*)NewtonBodyGetUserData(body) : NULL;
}

dNewtonBody* dNewtonWorld::GetNextBody(dNewtonBody* const body) const
{
	NewtonBody* const nextBody = NewtonWorldGetNextBody(m_world, body->m_body);
	return nextBody ? (dNewtonBody*)NewtonBodyGetUserData(nextBody) : NULL;
}

void* dNewtonWorld::GetNextContactJoint(dNewtonBody* const body, void* const contact) const
{
	NewtonBody* const newtonBody = body->m_body;
	for (NewtonJoint* contactJoint = NewtonBodyGetNextContactJoint(newtonBody, (NewtonJoint*)contact); contactJoint; contactJoint = NewtonBodyGetNextContactJoint(newtonBody, contactJoint)) {
		if (NewtonJointIsActive(contactJoint)) {
			return contactJoint;
		}
	}
	return NULL;
}

void* dNewtonWorld::GetFirstContactJoint(dNewtonBody* const body) const
{
	NewtonBody* const newtonBody = body->m_body;
	for (NewtonJoint* contactJoint = NewtonBodyGetFirstContactJoint(body->m_body); contactJoint; contactJoint = NewtonBodyGetNextContactJoint(newtonBody, contactJoint)) {
		if (NewtonJointIsActive(contactJoint)) {
			return contactJoint;
		}
	}
	return NULL;
}

void* dNewtonWorld::GetFirstContact(void* const joint) const
{
	return NewtonContactJointGetFirstContact(static_cast<NewtonJoint*>(joint));
}

void* dNewtonWorld::GetNextContact(void* const joint, void* const contact) const
{
	return NewtonContactJointGetNextContact(static_cast<NewtonJoint*>(joint), contact);
}

dNewtonBody* dNewtonWorld::GetBody0(void* const contact) const
{
	NewtonJoint* const contactJoint = (NewtonJoint*)contact;
	NewtonBody* const body = NewtonJointGetBody0(contactJoint);
	return (dNewtonBody*)NewtonBodyGetUserData(body);
}

dNewtonBody* dNewtonWorld::GetBody1(void* const contact) const
{
	NewtonJoint* const contactJoint = (NewtonJoint*)contact;
	NewtonBody* const body = NewtonJointGetBody1(contactJoint);
	return (dNewtonBody*)NewtonBodyGetUserData(body);
}

void* dNewtonWorld::GetBody0UserData(void* const contact) const
{
	NewtonJoint* const contactJoint = (NewtonJoint*)contact;
	NewtonBody* const body = NewtonJointGetBody0(contactJoint);
	if (body == nullptr)return nullptr;
	dNewtonBody* const dBody = (dNewtonBody*)NewtonBodyGetUserData(body);
	if (dBody == nullptr)return nullptr;
	return dBody->GetUserData();
}

void* dNewtonWorld::GetBody1UserData(void* const contact) const
{
	NewtonJoint* const contactJoint = (NewtonJoint*)contact;
	NewtonBody* const body = NewtonJointGetBody1(contactJoint);
	if (body == nullptr)return nullptr;
	dNewtonBody* const dBody = (dNewtonBody*)NewtonBodyGetUserData(body);
	if (dBody == nullptr)return nullptr;
	return dBody->GetUserData();
}

//int dNewtonWorld::OnSubShapeAABBOverlapTest(const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
int dNewtonWorld::OnSubShapeAABBOverlapTest(const NewtonJoint* const contactJoint, dFloat timestep, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}

int dNewtonWorld::OnBodiesAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody* const bodyPtr0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const bodyPtr1 = NewtonJointGetBody1(contactJoint);
	//dNewtonWorld* const world = (dNewtonWorld*)NewtonMaterialGetMaterialPairUserData(material);
	dNewtonWorld* const world = (dNewtonWorld*)NewtonWorldGetUserData(NewtonBodyGetWorld(bodyPtr0));
	NewtonCollision* const newtonCollision0 = (NewtonCollision*)NewtonBodyGetCollision(bodyPtr0);
	NewtonCollision* const newtonCollision1 = (NewtonCollision*)NewtonBodyGetCollision(bodyPtr1);
	dNewtonCollision* const collision0 = (dNewtonCollision*)NewtonCollisionGetUserData(newtonCollision0);
	dNewtonCollision* const collision1 = (dNewtonCollision*)NewtonCollisionGetUserData(newtonCollision1);
	const dMaterialProperties materialProp = world->FindMaterial(collision0->m_materialID, collision1->m_materialID);
	
	if (materialProp.m_aabb_overlap_callback)
	{
		dNewtonBody* dbody0 = (dNewtonBody*)NewtonBodyGetUserData(bodyPtr0);
		dNewtonBody* dbody1 = (dNewtonBody*)NewtonBodyGetUserData(bodyPtr1);
		if (!dbody0 || !dbody1)return 0;
		void* userData0 = dbody0->GetUserData();
		void* userData1 = dbody1->GetUserData();
		if (!userData0 || !userData1)return 0;
		return materialProp.m_aabb_overlap_callback(userData0, userData1);
	}
	else return materialProp.m_collisionEnable ? 1 : 0;
	//return 1;
}

void dNewtonWorld::OnContactCollision(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	dNewtonBody* const dbody0 = (dNewtonBody*)NewtonBodyGetUserData(body0);
	dNewtonBody* const dbody1 = (dNewtonBody*)NewtonBodyGetUserData(body1);
	if (!dbody0 || !dbody1)return;
//	dbody0->m_onCollision(dbody1);
//	dbody1->m_onCollision(dbody0);

	dNewtonWorld* const world = (dNewtonWorld*)NewtonWorldGetUserData(NewtonBodyGetWorld(body0));

	const dMaterialProperties* lastMaterialProp = NULL;
	for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial(contact);

		float normalImpact = NewtonMaterialGetContactNormalSpeed(material);
		float penetration = NewtonMaterialGetContactPenetration(material);
		//if(dbody0 && dbody0->m_onContactCallback)dbody0->m_onContactCallback(normalImpact);
		float normal[3];
		float position[3];
		
		if (dbody0->m_onContactCallback)
		{
			NewtonMaterialGetContactPositionAndNormal(material, body0, position, normal);
			dbody0->m_onContactCallback(dbody1->GetUserData(), normal, normalImpact, penetration);
		}
		
		if (dbody1->m_onContactCallback)
		{
			NewtonMaterialGetContactPositionAndNormal(material, body1, position, normal);
			dbody1->m_onContactCallback(dbody0->GetUserData(), normal, normalImpact, penetration);
		}

		NewtonCollision* const newtonCollision0 = (NewtonCollision*)NewtonContactGetCollision0(contact);
		NewtonCollision* const newtonCollision1 = (NewtonCollision*)NewtonContactGetCollision1(contact);
		dNewtonCollision* const collision0 = (dNewtonCollision*)NewtonCollisionGetUserData(newtonCollision0);
		dNewtonCollision* const collision1 = (dNewtonCollision*)NewtonCollisionGetUserData(newtonCollision1);
		const dMaterialProperties* const currentMaterialProp = &world->FindMaterial(collision0->m_materialID, collision1->m_materialID);
		dMaterialProperties materialProp(*currentMaterialProp);
		/*if (currentMaterialProp != lastMaterialProp) {
			lastMaterialProp = currentMaterialProp;
			if(materialProp.m_callback)materialProp.m_callback();
			// do a material callback here is needed
		}*/
		if (materialProp.m_callback)
		{
			NewtonMaterialGetContactPositionAndNormal(material, body0, position, normal);
			NewtonUserContactPoint info;
			info.m_point[0] = position[0];
			info.m_point[1] = position[1];
			info.m_point[2] = position[2];
			info.m_normal[0] = normal[0];
			info.m_normal[1] = normal[1];
			info.m_normal[2] = normal[2];
			info.m_penetration = penetration;
			info.m_shapeId0 = (dLong)NewtonContactGetCollisionID0(contact);
			info.m_shapeId1 = (dLong)NewtonContactGetCollisionID1(contact);
			//info.m_hitBody = dbody0;
			materialProp.m_callback(&materialProp, dbody0->GetUserData(), dbody1->GetUserData(), &info, normalImpact);
		}

		NewtonMaterialSetContactElasticity(material, materialProp.m_restitution);
		NewtonMaterialSetContactFrictionCoef(material, materialProp.m_staticFriction, materialProp.m_kineticFriction, 0);
		NewtonMaterialSetContactFrictionCoef(material, materialProp.m_staticFriction, materialProp.m_kineticFriction, 1);
	}
}

void dNewtonWorld::Update(dFloat timestepInSeconds)
{
	dLong timestepMicroSeconds = dClamp((dLong)(double(timestepInSeconds) * 1000000.0f), (dLong)0, m_timeStepInMicroSeconds * m_maxInterations);
	m_realTimeInMicroSeconds += timestepMicroSeconds;
	m_stepsCount = (int)(m_realTimeInMicroSeconds / m_timeStepInMicroSeconds);
	if (m_stepsCount > m_maxInterations)m_stepsCount = m_maxInterations;

	for (int doUpate = m_maxInterations; m_realTimeInMicroSeconds >= m_timeStepInMicroSeconds; doUpate--) {
		if (doUpate) {
			//UpdateWorld(forceCallback);
			UpdateWorld();
		}
		m_realTimeInMicroSeconds -= m_timeStepInMicroSeconds;
		dAssert(m_realTimeInMicroSeconds >= 0);
	}
	dAssert(m_realTimeInMicroSeconds >= 0);
	dAssert(m_realTimeInMicroSeconds < m_timeStepInMicroSeconds);

	// call every frame update
	m_interpotationParam = dFloat(double(m_realTimeInMicroSeconds) / double(m_timeStepInMicroSeconds));
	m_onTransformCallback();
}

void dNewtonWorld::SaveSerializedScene(const char* const sceneName)
{
	NewtonSerializeToFile(m_world, sceneName, NULL, NULL);
}

void dNewtonWorld::LoadPlugins(const char* const pluginPath)
{
	char plugInPath[2048];
	sprintf(plugInPath, "%s/Plugins/newtonPlugins/Release", pluginPath);
	NewtonLoadPlugins(m_world, plugInPath);
}

void dNewtonWorld::UnloadPlugins()
{
	NewtonUnloadPlugins(m_world);
}

void* dNewtonWorld::FirstPlugin() const
{
	return NewtonGetFirstPlugin(m_world);
}

void* dNewtonWorld::NextPlugin(const void* const plugin) const
{
	return NewtonGetNextPlugin(m_world, plugin);
}

const char* dNewtonWorld::GetPluginName(const void* const plugin) const
{
	return NewtonGetPluginString(m_world, plugin);
}

void dNewtonWorld::SelectPlugin(const void* const plugin) const
{
	NewtonSelectPlugin(m_world, plugin);
}

void dNewtonWorld::UpdateWorld()
{
	for (NewtonBody* bodyPtr = NewtonWorldGetFirstBody(m_world); bodyPtr; bodyPtr = NewtonWorldGetNextBody(m_world, bodyPtr)) {
		dNewtonBody* const body = (dNewtonBody*)NewtonBodyGetUserData(bodyPtr);
		if(body)body->InitForceAccumulators();
	}

	// every rigid body update
	m_onUpdateCallback(m_timeStep);

	if (m_asyncUpdateMode) 
	{
		NewtonWaitForUpdateToFinish(m_world);
		NewtonUpdateAsync(m_world, m_timeStep);
	} else {
		NewtonUpdate(m_world, m_timeStep);
	}
}


