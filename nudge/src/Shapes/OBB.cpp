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
		: Obb(Vector3{ 0.f }, Vector3{ 1.f })
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
		const Vector3 direction = point - origin;

		for (int i = 0; i < 3; ++i)
		{
			Vector3 axis = orientation.GetColumn(i);
			const float distance = Vector3::Dot(direction, axis);

			if (distance > extents[i] || distance < -extents[i])
			{
				return false;
			}
		}

		return true;
	}

	Vector3 Obb::ClosestPoint(const Vector3& point) const
	{
		Vector3 result = origin;

		const Vector3 direction = point - origin;

		for (int i = 0; i < 3; ++i)
		{
			Vector3 axis = orientation.GetColumn(i);
			float distance = Vector3::Dot(direction, axis);

			distance = MathF::Max(distance, -extents[i]);
			distance = MathF::Min(distance, extents[i]);

			result += axis * distance;
		}

		return result;
	}

	bool Obb::Intersects(const Aabb& other) const
	{
		return other.Intersects(*this);
	}

	bool Obb::Intersects(const Obb& other) const
	{
		return Interval::ObbObb(*this, other);
	}

	bool Obb::Intersects(const Plane& other) const
	{
		const float pLen = extents.x * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(0))) +
		             extents.y * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(1))) +
		             extents.z * MathF::Abs(Vector3::Dot(other.normal, orientation.GetColumn(2)));

		const float dist = Vector3::Dot(other.normal, origin);

		return MathF::Abs(dist) <= pLen;
	}

	bool Obb::Intersects(const Sphere& other) const
	{
		return other.Intersects(*this);
	}

	bool Obb::Intersects(const Triangle& other) const
	{
		return other.Intersects(*this);
	}
}
