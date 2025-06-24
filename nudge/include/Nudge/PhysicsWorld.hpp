#pragma once
#include <vector>

using std::vector;

namespace Nudge
{
	class Collider;
	class Rigidbody;

    class PhysicsWorld
    {
    public:
        void Tick(float dt);
        void Render();

        void Add(Rigidbody* body);
        void Add(Collider* collider);

        void ClearBodies();
        void ClearColliders();

    protected:
        vector<Rigidbody*> m_bodies;
        vector<Collider*> m_colliders;

    };
}
