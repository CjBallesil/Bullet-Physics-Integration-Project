#pragma once

#include "Physics.h"
#include "ComponentIncludes.h"

class CBall {
public:
	btCollisionShape* shape = nullptr;
	btDefaultMotionState* motionState = nullptr;
	btRigidBody* body = nullptr;
	Vector4 m_vTint; //RGBA color

	void Create(btDiscreteDynamicsWorld* world);
	void Update(float dt);
	void Render();
	void Destroy(btDiscreteDynamicsWorld* world);
	float RandFloat(float min, float max);
private:
	float posY = 0.0f;
	float posX = 0.0f;
};