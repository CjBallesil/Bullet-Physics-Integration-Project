#include "Ball.h"
#include "SpriteRenderer.h"
#include "Common.h"
#include <random>
#include "ComponentIncludes.h"
#include <btBulletDynamicsCommon.h>

void CBall::Create(btDiscreteDynamicsWorld* world) {
	//generate numbers
	btVector3& pos = {RandFloat(-10.0f, 10.0f), RandFloat(10.0f, 30.0f), 0};
	float radius = RandFloat(1.0f, 5.0f);
	float mass = RandFloat(1.0f, 10.0f);
	m_vTint = { RandFloat(0.2f, 1.0f), RandFloat(0.2f, 1.0f), RandFloat(0.2f, 1.0f), RandFloat(0.2f, 1.0f) };


	//create shape
	shape = new btSphereShape(radius);

	//set up motion state and transform
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(pos);

	//set mass and inertia
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(mass, inertia);

	//create rigid body
	motionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
	body = new btRigidBody(rbInfo);

	//add to bullet world
	world->addRigidBody(body);
}

void CBall::Update(float dt) {
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	btVector3 pos = trans.getOrigin();

	//store positions for rendering
	posX = pos.x();
	posY = pos.y();
}

void CBall::Render() {
	float radius = static_cast<btSphereShape*>(shape)->getRadius();

	//draw
	LSpriteDesc2D desc;
	desc.m_nCurrentFrame = 0;
	desc.m_vPos = { posX, posY };
	desc.m_fXScale = radius * 2.0f;
	desc.m_fYScale = radius * 2.0f;
	desc.m_fRoll = 0.0f;
	desc.m_fAlpha = 1.0f;
	desc.m_f4Tint = m_vTint;

	m_pRenderer->Draw(&desc);
}

void CBall::Destroy(btDiscreteDynamicsWorld* world) {
	world->removeRigidBody(body);
	delete body->getMotionState();
	delete body;
	delete shape;

	body = nullptr;
	motionState = nullptr;
	shape = nullptr;
}

float CBall::RandFloat(float min, float max) {
	return min + static_cast<float>(rand()) / RAND_MAX * (max - min);
}