#include "Nudge/Physics/Shapes/Ray.hpp"

#include "Nudge/Maths/MathF.hpp"

namespace Nudge
{
	Ray Ray::FromPoints(const Vector3& from, const Vector3& to)
	{
		return { from, (to - from).Normalized() };
	}

	Ray::Ray()
		: Ray(Vector3{ 0.f }, Vector3{ 0.f, 0.f, 1.f })
	{
	}

	Ray::Ray(const Vector3& origin, const Vector3& direction)
		: origin{ origin }, direction{ direction }
	{
	}

	void Ray::Normalize()
	{
		direction.Normalize();
	}

	bool Ray::Contains(const Vector3& point) const
	{
		if (point == origin)
		{
			return true;
		}

		const Vector3 norm = (point - origin).Normalized();
		return MathF::IsNearZero(Vector3::Dot(norm, direction));
	}

	Vector3 Ray::ClosestPoint(const Vector3& point) const
	{
		const float t = MathF::Max(Vector3::Dot(point - origin, direction), 0.f);

		return origin + direction * t;
	}
}
