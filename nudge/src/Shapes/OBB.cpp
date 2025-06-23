#include "Nudge/Shapes/OBB.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/Interval.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

namespace Nudge
{
	Obb::Obb()
		: Obb(Vector3{ 0.f }, Vector3{ 0.f })
	{
	}

	Obb::Obb(const Vector3& origin, const Vector3& extents)
		: Obb{ origin, extents, Matrix3{ } }
	{
	}

	Obb::Obb(const Vector3& origin, const Vector3& extents, const Matrix3& orientation)
		: origin{ origin }, extents{ extents }, orientation{ orientation }
	{
	}

	bool Obb::Contains(const Vector3& point) const
	{
		// Calculate vector from OBB center to the test point
		const Vector3 direction = point - origin;

		// Test containment along each local axis of the OBB
		for (int i = 0; i < 3; ++i)
		{
			// Get the i-th local axis of the OBB
			Vector3 axis = orientation.GetColumn(i);
			// Project the direction vector onto this axis
			const float distance = Vector3::Dot(direction, axis);

			// Check if the projection exceeds the extent in either direction
			if (distance > extents[i] || distance < -extents[i])
			{
				return false;  // Point is outside the OBB on this axis
			}
		}

		// Point is within bounds on all axes
		return true;
	}

	Vector3 Obb::ClosestPoint(const Vector3& point) const
	{
		// Start with the OBB center
		Vector3 result = origin;

		// Calculate vector from OBB center to the test point
		const Vector3 direction = point - origin;

		// Project onto each local axis and clamp to extents
		for (int i = 0; i < 3; ++i)
		{
			// Get the i-th local axis of the OBB
			Vector3 axis = orientation.GetColumn(i);
			// Project the direction vector onto this axis
			float distance = Vector3::Dot(direction, axis);

			// Clamp the distance to the valid range [-extents[i], extents[i]]
			distance = MathF::Max(distance, -extents[i]);
			distance = MathF::Min(distance, extents[i]);

			// Add the clamped contribution along this axis
			result += axis * distance;
		}

		return result;
	}

	bool Obb::Intersects(const Aabb& other) const
	{
		// Delegate to AABB's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Obb::Intersects(const Obb& other) const
	{
		// Delegate to specialized OBB-OBB intersection algorithm
		return Interval::ObbObb(*this, other);
	}

	bool Obb::Intersects(const Plane& other) const
	{
		// Calculate the projection length of the OBB onto the plane normal
		// Sum the absolute projections of each local axis scaled by its extent
		const float pLen = extents.x * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(0))) +
		             extents.y * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(1))) +
		             extents.z * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(2)));

		// Calculate the distance from OBB center to the plane
		const float dist = Vector3::Dot(other.normal, origin);

		// Check if the distance is within the projection length
		return MathF::Abs(dist) <= pLen;
	}

	bool Obb::Intersects(const Sphere& other) const
	{
		// Delegate to the sphere's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}

	bool Obb::Intersects(const Triangle& other) const
	{
		// Delegate to the triangle's intersection method (avoids code duplication)
		return other.Intersects(*this);
	}
}