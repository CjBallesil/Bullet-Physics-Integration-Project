
#include "Physics.h"

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
}

void CPhysics::UpdateBullet(float dt) {
	if (dynamicsWorld) {
		dynamicsWorld->stepSimulation(dt);
	}
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