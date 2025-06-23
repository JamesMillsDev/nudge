#include "Nudge/Shapes/Line.hpp"

#include "Nudge/Ray.hpp"
#include "Nudge/RaycastHit.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"

namespace Nudge
{
	/**
	 * @brief Default constructor - creates a unit line segment along Y-axis
	 *
	 * Creates a line from (0,0,0) to (0,1,0), providing a valid non-degenerate
	 * line segment for default initialization.
	 */
	Line::Line()
		: Line(Vector3{ 0.f }, Vector3{ 0.f, 1.f, 0.f })
	{
	}

	/**
	 * @brief Constructs a line segment between two specified points
	 * @param start Starting endpoint of the line segment
	 * @param end Ending endpoint of the line segment
	 */
	Line::Line(const Vector3& start, const Vector3& end)
		: start{ start }, end{ end }
	{
	}

	/**
	 * @brief Calculates the length of the line segment
	 * @return Euclidean distance between start and end points
	 */
	float Line::Length() const
	{
		return MathF::Sqrt(LengthSqr());
	}

	/**
	 * @brief Calculates the squared length of the line segment
	 * @return Squared Euclidean distance between start and end points
	 *
	 * More efficient than Length() when comparing distances or when
	 * the actual distance value isn't needed.
	 */
	float Line::LengthSqr() const
	{
		return (start - end).MagnitudeSqr();
	}

	/**
	 * @brief Tests if a point lies on the line segment
	 * @param point Point to test for containment
	 * @return True if the point lies on the line segment (within floating-point tolerance)
	 *
	 * Uses the closest point method to determine if the input point is effectively
	 * on the line segment by checking if the distance is near zero.
	 */
	bool Line::Contains(const Vector3& point) const
	{
		const Vector3 closest = ClosestPoint(point);

		// Check if the closest point on the line is essentially the same as the input point
		return MathF::IsNearZero((closest - point).MagnitudeSqr());
	}

	/**
	 * @brief Finds the closest point on the line segment to a given point
	 * @param point Reference point to find closest approach to
	 * @return Point on the line segment that is closest to the input point
	 *
	 * Uses parametric line equation P(t) = start + t*(end-start) where t is clamped to [0,1]
	 * to ensure the result lies within the line segment bounds.
	 */
	Vector3 Line::ClosestPoint(const Vector3& point) const
	{
		const Vector3 lVec = end - start; // Line direction vector
		const float t = MathF::Clamp01( // Clamp parameter to [0,1] for segment bounds
		                               Vector3::Dot(point - start, lVec) / // Project point onto line direction
		                               Vector3::Dot(lVec, lVec) // Normalize by line length squared
		                              );

		return start + lVec * t; // Return parameterized point on segment
	}

	bool Line::Test(const Aabb& other) const
	{
		const Ray ray = { start, (end - start).Normalized() };
		RaycastHit hit;

		if (!ray.CastAgainst(other, &hit))
		{
			return false;
		}

		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Obb& other) const
	{
		const Ray ray = { start, (end - start).Normalized() };
		RaycastHit hit;

		if (!ray.CastAgainst(other, &hit))
		{
			return false;
		}

		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Plane& other) const
	{
		const Ray ray = { start, (end - start).Normalized() };
		RaycastHit hit;

		if (!ray.CastAgainst(other, &hit))
		{
			return false;
		}

		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Sphere& other) const
	{
		const Ray ray = { start, (end - start).Normalized() };
		RaycastHit hit;

		if (!ray.CastAgainst(other, &hit))
		{
			return false;
		}

		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}

	bool Line::Test(const Triangle& other) const
	{
		const Ray ray = { start, (end - start).Normalized() };
		RaycastHit hit;

		if (!ray.CastAgainst(other, &hit))
		{
			return false;
		}

		const float t = hit.distance;
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}
}
