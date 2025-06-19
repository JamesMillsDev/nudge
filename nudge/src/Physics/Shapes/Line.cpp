#include "Nudge/Physics/Shapes/Line.hpp"

#include "Nudge/Maths/MathF.hpp"

namespace Nudge
{
	Line::Line()
		: Line(Vector3{ 0.f }, Vector3{ 0.f, 1.f, 0.f })
	{
	}

	Line::Line(const Vector3& start, const Vector3& end)
		: start{ start }, end{ end }
	{
	}

	float Line::Length() const
	{
		return MathF::Sqrt(LengthSqr());
	}

	float Line::LengthSqr() const
	{
		return (start - end).MagnitudeSqr();
	}

	bool Line::Contains(const Vector3& point) const
	{
		const Vector3 closest = ClosestPoint(point);

		return MathF::IsNearZero((closest - point).MagnitudeSqr());
	}

	Vector3 Line::ClosestPoint(const Vector3& point) const
	{
		const Vector3 lVec = end - start;
		const float t = MathF::Clamp01(Vector3::Dot(point - start, lVec) / Vector3::Dot(lVec, lVec));

		return start + lVec * t;
	}
}
