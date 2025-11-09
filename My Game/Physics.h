#pragma once

#include <btBulletDynamicsCommon.h>

class CPhysics {
	public:
		static btBroadphaseInterface* broadphase;
		static btDefaultCollisionConfiguration* collisionConfiguration;
		static btCollisionDispatcher* dispatcher;
		static btSequentialImpulseConstraintSolver* solver;
		static btDiscreteDynamicsWorld* dynamicsWorld;

		void InitializeBullet();
		void ShutDownBullet();
		void UpdateBullet(float dt);
};