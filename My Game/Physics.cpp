
#include "Physics.h"
#include <windows.h>

btBroadphaseInterface* CPhysics::broadphase = nullptr;
btDefaultCollisionConfiguration* CPhysics::collisionConfiguration = nullptr;
btCollisionDispatcher* CPhysics::dispatcher = nullptr;
btSequentialImpulseConstraintSolver* CPhysics::solver = nullptr;
btDiscreteDynamicsWorld* CPhysics::dynamicsWorld = nullptr;

void CPhysics::InitializeBullet() {
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btDiscreteDynamicsWorld(
		dispatcher, broadphase, solver, collisionConfiguration
	);

	dynamicsWorld->setGravity(btVector3(0, -9.8f, 0));

	//add borders
	float scale = 30.0f;
	float halfWidth = 1024.0f / scale / 2.0f;
	float halfHeight = 768.0f / scale / 2.0f;

	// after you compute scale, halfWidth, halfHeight...
// thickness of the wall (in Bullet units)
	const float wallThickness = 1.0f; // 1 unit ? 30 px; adjust if you want thinner

	// helper to create a static box wall at a given center transform
	auto AddStaticBox = [&](const btVector3& center, const btVector3& halfExtents) {
		btCollisionShape* shape = new btBoxShape(halfExtents);

		btTransform t;
		t.setIdentity();
		t.setOrigin(center);

		btDefaultMotionState* motion = new btDefaultMotionState(t);
		btRigidBody::btRigidBodyConstructionInfo info(0.0f, motion, shape);
		btRigidBody* body = new btRigidBody(info);
		body->setRestitution(1.0f);
		body->setDamping(0.0f, 0.0f);
		dynamicsWorld->addRigidBody(body);
		// (optionally store shape & motion pointers to delete at shutdown)
		};

	// compute extents in Bullet units
	float hw = halfWidth;   // ~17.07 units
	float hh = halfHeight;  // ~12.80 units

	// floor: centered at (0, -hh - thickness/2) so top surface sits at -hh
	AddStaticBox(btVector3(0.0f, -hh - wallThickness * 0.5f, 0.0f),
		btVector3(hw + wallThickness, wallThickness * 0.5f, 10.0f));

	// ceiling: centered at (0, +hh + thickness/2) so bottom surface sits at +hh
	AddStaticBox(btVector3(0.0f, +hh + wallThickness * 0.5f, 0.0f),
		btVector3(hw + wallThickness, wallThickness * 0.5f, 10.0f));

	// left wall: centered at (-hw - thickness/2, 0)
	AddStaticBox(btVector3(-hw - wallThickness * 0.5f, 0.0f, 0.0f),
		btVector3(wallThickness * 0.5f, hh + wallThickness, 10.0f));

	// right wall: centered at (+hw + thickness/2, 0)
	AddStaticBox(btVector3(+hw + wallThickness * 0.5f, 0.0f, 0.0f),
		btVector3(wallThickness * 0.5f, hh + wallThickness, 10.0f));
}

void CPhysics::UpdateBullet(float dt) {
	if (dynamicsWorld)
		dynamicsWorld->stepSimulation(dt, 10, 1.0f / 240.0f);
}

void CPhysics::ShutDownBullet() {
	//remove rigid bodies from world
	if (dynamicsWorld) {
		for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
			dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}
	}

	//delete world and supporting systems
	delete dynamicsWorld;			dynamicsWorld = nullptr;
	delete solver;					solver = nullptr;
	delete dispatcher;				dispatcher = nullptr;
	delete collisionConfiguration;	collisionConfiguration = nullptr;
	delete broadphase;				broadphase = nullptr;
}