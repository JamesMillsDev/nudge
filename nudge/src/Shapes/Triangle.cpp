#include "Nudge/Shapes/Triangle.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/Interval.hpp"
#include "Nudge/Shapes/Line.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"

namespace Nudge
{
	Triangle::Triangle()
		: Triangle{ Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 0.f, 0.f } }
	{
	}

	Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
		: a{ a }, b{ b }, c{ c }
	{
	}

	Triangle::Triangle(const Triangle& other)
		: a{ other.a }, b{ other.b }, c{ other.c }
	{

	}

	bool Triangle::Contains(const Vector3& point) const
	{
		// Calculate vectors from each vertex to the test point
		const Vector3 triA = a - point;
		const Vector3 triB = b - point;
		const Vector3 triC = c - point;

		// Handle degenerate case: zero-area triangle (all vertices coincident)
		if(MathF::Compare(Vector3::DistanceSqr(a, b), 0.f, MathF::epsilon) &&
			MathF::Compare(Vector3::DistanceSqr(b, c), 0.f, MathF::epsilon) &&
			!triA.IsNearZero(.00001f) && !triB.IsNearZero(.00001f) && !triC.IsNearZero(.00001f))
		{
			// Zero area triangle with point outside (all tri's aligned non zero)
			return false;
		}

		// Calculate cross products between consecutive edge vectors from the point
		const Vector3 normPbc = Vector3::Cross(triB, triC);
		const Vector3 normPca = Vector3::Cross(triC, triA);
		const Vector3 normPab = Vector3::Cross(triA, triB);

		// Check if all cross products point in the same direction (same side test)
		// If dot products are negative, point is outside the triangle
		if (Vector3::Dot(normPbc, normPca) < 0.f)
		{
			return false;
		}

		if(Vector3::Dot(normPbc, normPab) < 0.f)
		{
			return false;
		}

		// Point is inside if all same-side tests pass
		return true;
	}

	Vector3 Triangle::ClosestPoint(const Vector3& point) const
	{
		// First check if the closest point is on the triangle face
		const Plane plane = Plane::From(*this);

		// Project point onto triangle's plane and check if it's inside the triangle
		if (const Vector3 closest = plane.ClosestPoint(point); Contains(closest))
		{
			return closest;  // Point projects inside triangle, use plane projection
		}

		// Point projects outside triangle, find closest point on each edge
		const Vector3 c1 = Line{ a, b }.ClosestPoint(point);
		const Vector3 c2 = Line{ b, c }.ClosestPoint(point);
		const Vector3 c3 = Line{ c, a }.ClosestPoint(point);

		// Calculate squared distances to avoid expensive square root operations
		const float magSqr1 = (point - c1).MagnitudeSqr();
		const float magSqr2 = (point - c2).MagnitudeSqr();
		const float magSqr3 = (point - c3).MagnitudeSqr();

		// Return the closest point among the three edge projections
		if (magSqr1 < magSqr2 && magSqr1 < magSqr3)
		{
			return c1;
		}

		return magSqr2 < magSqr1 && magSqr2 < magSqr3 ? c2 : c3;
	}

	Vector3 Triangle::Barycentric(const Vector3& point) const
	{
		// Calculate vectors from each vertex to the point
		const Vector3 ap = point - a;
		const Vector3 bp = point - b;
		const Vector3 cp = point - c;

		// Calculate triangle edge vectors
		const Vector3 ab = b - a;
		const Vector3 ac = c - a;
		const Vector3 bc = c - b;
		const Vector3 cb = b - c;
		const Vector3 ca = a - c;

		// Calculate barycentric coordinate for vertex A
		// Project edge AB onto the perpendicular to edge CB
		Vector3 v = ab - Vector3::Project(ab, cb);
		const float av = 1.f - Vector3::Dot(v, ap) / Vector3::Dot(v, ab);

		// Calculate barycentric coordinate for vertex B
		// Project edge BC onto the perpendicular to edge AC
		v = bc - Vector3::Project(bc, ac);
		const float bv = 1.f - Vector3::Dot(v, bp) / Vector3::Dot(v, bc);

		// Calculate barycentric coordinate for vertex C
		// Project edge CA onto the perpendicular to edge AB
		v = ca - Vector3::Project(ca, ab);
		const float cv = 1.f - Vector3::Dot(v, cp) / Vector3::Dot(v, ca);

		// Return barycentric coordinates (should sum to 1.0 for points on triangle)
		return { av, bv, cv };
	}

	bool Triangle::Intersects(const Aabb& other) const
	{
		// Delegate to specialized triangle-AABB intersection algorithm
		return Interval::TriangleAabb(*this, other);
	}

	bool Triangle::Intersects(const Obb& other) const
	{
		// Delegate to specialized triangle-OBB intersection algorithm
		return Interval::TriangleObb(*this, other);
	}

	bool Triangle::Intersects(const Plane& other) const
	{
		// Evaluate plane equation for each vertex to get signed distances
		const float side1 = Plane::PlaneEquation(a, other);
		const float side2 = Plane::PlaneEquation(b, other);
		const float side3 = Plane::PlaneEquation(c, other);

		// Triangle lies completely on the plane if all vertices have zero distance
		if (MathF::IsNearZero(side1) && MathF::IsNearZero(side2) && MathF::IsNearZero(side3))
		{
			return true;
		}

		// Triangle doesn't intersect if all vertices are on the same side of the plane
		if ((side1 > 0.f && side2 > 0.f && side3 > 0.f) ||
			(side1 < 0.f && side2 < 0.f && side3 < 0.f))
		{
			return false;
		}

		// Triangle intersects plane if vertices are on different sides
		return true;
	}

	bool Triangle::Intersects(const Sphere& other) const
	{
		// Find closest point on triangle to sphere center
		const Vector3 closest = ClosestPoint(other.origin);

		// Triangle intersects sphere if closest point is within sphere radius
		return (closest - other.origin).MagnitudeSqr() <= MathF::Squared(other.radius);
	}

	bool Triangle::Intersects(const Triangle& other) const
	{
		// Delegate to specialized triangle-triangle intersection algorithm
		return Interval::TriangleTriangle(*this, other);
	}

	Triangle& Triangle::operator=(const Triangle& rhs)
	{
		// Copy each vertex from the right-hand side triangle
		a = rhs.a;
		b = rhs.b;
		c = rhs.c;

		// Return reference to this triangle for chaining
		return *this;
	}
}