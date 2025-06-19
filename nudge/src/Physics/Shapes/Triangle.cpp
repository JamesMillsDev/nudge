#include "Nudge/Physics/Shapes/Triangle.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/Interval.hpp"
#include "Nudge/Physics/Shapes/Line.hpp"
#include "Nudge/Physics/Shapes/Plane.hpp"
#include "Nudge/Physics/Shapes/Sphere.hpp"

namespace Nudge
{
	Triangle::Triangle()
		: Triangle{ Vector3{ -1.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, Vector3{ 1.f, 0.f, 0.f } }
	{
	}

	Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
		: a{ a }, b{ b }, c{ c }
	{
	}

	bool Triangle::Contains(const Vector3& point) const
	{
		const Vector3 triA = a - point;
		const Vector3 triB = b - point;
		const Vector3 triC = c - point;

		const Vector3 normPbc = Vector3::Cross(triB, triC);
		const Vector3 normPca = Vector3::Cross(triC, triA);
		const Vector3 normPab = Vector3::Cross(triA, triB);

		if (Vector3::Dot(normPbc, normPca) < 0.f || Vector3::Dot(normPbc, normPab) < 0.f)
		{
			return false;
		}

		return true;
	}

	Vector3 Triangle::ClosestPoint(const Vector3& point) const
	{
		const Plane plane = Plane::From(*this);

		if (const Vector3 closest = plane.ClosestPoint(point); Contains(closest))
		{
			return closest;
		}

		const Vector3 c1 = Line{ a, b }.ClosestPoint(point);
		const Vector3 c2 = Line{ b, c }.ClosestPoint(point);
		const Vector3 c3 = Line{ c, a }.ClosestPoint(point);

		const float magSqr1 = (point - c1).MagnitudeSqr();
		const float magSqr2 = (point - c2).MagnitudeSqr();
		const float magSqr3 = (point - c3).MagnitudeSqr();

		if (magSqr1 < magSqr2 && magSqr1 < magSqr3)
		{
			return c1;
		}

		return magSqr2 < magSqr1 && magSqr2 < magSqr3 ? c2 : c3;
	}

	Vector3 Triangle::Barycentric(const Vector3& point) const
	{
		const Vector3 ap = point - a;
		const Vector3 bp = point - a;
		const Vector3 cp = point - a;

		const Vector3 ab = b - a;
		const Vector3 ac = c - a;
		const Vector3 bc = c - b;
		const Vector3 cb = b - c;
		const Vector3 ca = a - c;

		Vector3 v = ab - Vector3::Project(ab, ac);
		const float av = 1.f - Vector3::Dot(v, ap) / Vector3::Dot(v, ab);

		v = bc - Vector3::Project(bc, ac);
		const float bv = 1.f - Vector3::Dot(v, bp) / Vector3::Dot(v, bc);

		v = ca - Vector3::Project(ca, ab);
		const float cv = 1.f - Vector3::Dot(v, cp) / Vector3::Dot(v, ca);

		return { av, bv, cv };
	}

	bool Triangle::Intersects(const Aabb& other) const
	{
		return Interval::TriangleAabb(*this, other);
	}

	bool Triangle::Intersects(const Obb& other) const
	{
		return Interval::TriangleObb(*this, other);
	}

	bool Triangle::Intersects(const Plane& other) const
	{
		const float side1 = Plane::PlaneEquation(a, other);
		const float side2 = Plane::PlaneEquation(b, other);
		const float side3 = Plane::PlaneEquation(c, other);

		if (MathF::IsNearZero(side1) && MathF::IsNearZero(side2) && MathF::IsNearZero(side3))
		{
			return true;
		}

		if ((side1 > 0.f && side2 > 0.f && side3 > 0.f) ||
			(side1 < 0.f && side2 < 0.f && side3 < 0.f))
		{
			return false;
		}

		return true;
	}

	bool Triangle::Intersects(const Sphere& other) const
	{
		const Vector3 closest = ClosestPoint(other.origin);

		return (closest - other.origin).MagnitudeSqr() < MathF::Squared(other.radius);
	}

	bool Triangle::Intersects(const Triangle& other) const
	{
		return Interval::TriangleTriangle(*this, other);
	}
}
