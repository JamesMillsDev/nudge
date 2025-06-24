#pragma once

#include "Nudge/Colliders/Collider.hpp"
#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
    class BoxCollider;
    class CollisionManifold;

    class SphereCollider final : public Collider
    {
    public:
        SphereCollider();

    public:
        void SetOrigin(const Vector3& origin) const;
        void SetRadius(float radius) const;

        Vector3 Origin() const;
        float Radius() const;

        CollisionManifold FindCollisionFeatures(const SphereCollider* other) const;
        CollisionManifold FindCollisionFeatures(const BoxCollider* other) const;

    };
}
