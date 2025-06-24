#include "Nudge/CollisionManifold.hpp"

#include <limits>

using std::numeric_limits;

namespace Nudge
{
	void CollisionManifold::Reset()
	{
		colliding = false;
		normal = { 0.f, 0.f, 1.f };
		depth = numeric_limits<float>::max();
		contacts.clear();
	}
}
