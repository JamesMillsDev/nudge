#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	/**
	 * @brief Structure containing information about a raycast intersection
	 */
	struct RaycastHit
	{
	public:
		/**
		 * @brief The world space point where the ray intersected the object
		 */
		Vector3 point;
		
		/**
		 * @brief The surface normal at the intersection point (unit vector pointing away from the surface)
		 */
		Vector3 normal;
		
		/**
		 * @brief The distance from the ray origin to the intersection point
		 */
		float distance;
		
		/**
		 * @brief Flag indicating whether the raycast hit anything
		 */
		bool didHit;
	};
}
