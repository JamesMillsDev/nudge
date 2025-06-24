#pragma once

#include "Nudge/Colliders/Collider.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
    class BoxCollider final : public Collider
    {
    public:
        BoxCollider();

    public:
        void SetOrigin(const Vector3& origin) const;
        void SetExtents(const Vector3& extents) const;
        void SetOrientation(const Vector3& axis, float angle) const;

        Vector3 Origin() const;
        Vector3 Extents() const;
        Matrix3 Orientation() const;

    };
}
