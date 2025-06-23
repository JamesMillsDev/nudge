#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	struct RaycastHit
	{
	public:
		Vector3 point;
		Vector3 normal;
		float distance;
		bool didHit;

	};
}
