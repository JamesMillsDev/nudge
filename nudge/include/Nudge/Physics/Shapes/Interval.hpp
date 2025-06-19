#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
    class Aabb;
    class Obb;

    class Interval
    {
    public:
        static Interval Get(const Aabb& aabb, const Vector3& axis);
        static Interval Get(const Obb& obb, const Vector3& axis);

        static bool OverlapOnAxis(const Aabb& aabb, const Obb& obb, const Vector3& axis);
        static bool AabbObb(const Aabb& aabb, const Obb& obb);

        static bool OverlapOnAxis(const Obb& a, const Obb& b, const Vector3& axis);
        static bool ObbObb(const Obb& a, const Obb& b);

    public:
        float min;
        float max;

    };
}