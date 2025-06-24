#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
    class Shape
    {
    public:
        virtual ~Shape() = default;

    public:
        virtual bool Contains(const Vector3& point) const = 0;
        virtual Vector3 ClosestPoint(const Vector3& point) const = 0;

    };
}