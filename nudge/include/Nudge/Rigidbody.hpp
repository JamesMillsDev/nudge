#pragma once
#include <vector>

using std::vector;

namespace Nudge
{
    class Collider;

    class Rigidbody
    {
    public:
        Rigidbody();
        virtual ~Rigidbody();

    public:
        virtual void Tick(float dt);
        virtual void Render();

        virtual void ApplyForces();
        virtual void SolveConstraints(const vector<Collider*>& colliders);

    };
}