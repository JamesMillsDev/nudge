#include "Nudge/PhysicsWorld.hpp"

#include "Nudge/Rigidbody.hpp"

namespace Nudge
{
	void PhysicsWorld::Tick(const float dt)
	{
		for(Rigidbody* body : m_bodies)
		{
			body->ApplyForces();
		}

		for(Rigidbody* body : m_bodies)
		{
			body->Tick(dt);
		}
		
		for(Rigidbody* body : m_bodies)
		{
			body->SolveConstraints(m_colliders);
		}
	}

	void PhysicsWorld::Render()
	{
	}

	void PhysicsWorld::Add(Rigidbody* body)
	{
		m_bodies.emplace_back(body);
	}

	void PhysicsWorld::Add(Collider* collider)
	{
		m_colliders.emplace_back(collider);
	}

	void PhysicsWorld::ClearBodies()
	{
		m_bodies.clear();
	}

	void PhysicsWorld::ClearColliders()
	{
		m_colliders.clear();
	}
}
