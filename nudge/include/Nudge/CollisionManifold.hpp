#pragma once

#include <vector>

#include "Nudge/Maths/Vector3.hpp"

using std::vector;

namespace Nudge
{
    class CollisionManifold
    {
    public:
        bool colliding;
        Vector3 normal;
        float depth;
        vector<Vector3> contacts;

    public:
        void Reset();

    };
}