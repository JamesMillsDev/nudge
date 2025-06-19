#include "Nudge/Physics/Shapes/Line.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/Plane.hpp"
#include "Nudge/Physics/Shapes/Ray.hpp"
#include "Nudge/Physics/Shapes/Sphere.hpp"

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

	/**
	 * @brief Tests if the line segment intersects with an Axis-Aligned Bounding Box
	 * @param other AABB to test intersection against
	 * @return True if the line segment intersects the AABB
	 */
	bool Line::Test(const Aabb& other) const
	{
		// Create ray from line start in line direction
		const Ray ray = { start, (end - start).Normalized() };
		const float t = ray.CastAgainst(other); // Get intersection distance
		const float tSqr = MathF::Squared(t);

		return t >= 0.f && tSqr <= LengthSqr(); // Check if intersection is within segment bounds
	}

	/**
	 * @brief Tests if the line segment intersects with an Oriented Bounding Box
	 * @param other OBB to test intersection against
	 * @return True if the line segment intersects the OBB
	 */
	bool Line::Test(const Obb& other) const
	{
		// Create ray from line start in line direction
		const Ray ray = { start, (end - start).Normalized() };
		const float t = ray.CastAgainst(other); // Get intersection distance
		const float tSqr = MathF::Squared(t);

		return t >= 0.f && tSqr <= LengthSqr(); // Check if intersection is within segment bounds
	}

	/**
	 * @brief Tests if the line segment intersects with a plane
	 * @param other Plane to test intersection against
	 * @return True if the line segment crosses the plane
	 *
	 * Uses parametric line-plane intersection. Returns false if line is parallel to plane.
	 */
	bool Line::Test(const Plane& other) const
	{
		const Vector3 ab = end - start; // Line direction vector

		const float nA = Vector3::Dot(other.normal, start); // Distance from plane to line start
		const float nAB = Vector3::Dot(other.normal, ab);   // Projection of line direction onto plane normal

		// Check if line is parallel to plane (no intersection)
		if (MathF::IsNearZero(nAB))
		{
			return false;
		}

		// Calculate intersection parameter: solve dot(start + t*ab, normal) = distance
		const float t = (other.distance - nA) / nAB;

		// Intersection occurs within line segment if t <= [0,1]
		return t >= 0.f && t <= 1.f;
	}

	/**
	 * @brief Tests if the line segment intersects with a sphere
	 * @param other Sphere to test intersection against
	 * @return True if the line segment intersects or passes through the sphere
	 *
	 * Finds the closest point on the line segment to the sphere center and checks
	 * if that distance is within the sphere radius.
	 */
	bool Line::Test(const Sphere& other) const
	{
		const Vector3 closest = ClosestPoint(other.origin);         // Closest point on line to sphere center
		const float radii = MathF::Squared(other.radius);           // Sphere radius squared
		const float dist = (other.origin - closest).MagnitudeSqr(); // Distance squared from sphere center to line

		// Intersection occurs if closest distance is within sphere radius
		return dist <= radii;
	}

	/**
	 * @brief Tests if the line segment intersects with a triangle
	 * @param other Triangle to test intersection against
	 * @return True if the line segment intersects the triangle
	 *
	 * Algorithm:
	 * 1. Convert line segment to ray starting at line's start point
	 * 2. Perform ray-triangle intersection to get hit distance
	 * 3. Check if intersection point lies within the line segment bounds
	 */
	bool Line::Test(const Triangle& other) const
	{
		// Create ray from line segment's start point in the direction toward end point
		const Ray ray =
		{
			start,                      // Ray origin at line segment start
			(end - start).Normalized()  // Normalized direction vector toward end point
		};

		// Perform ray-triangle intersection test
		// Returns distance along ray to intersection point, or -1 if no intersection
		const float t = ray.CastAgainst(other);

		// Check if intersection exists and lies within line segment bounds
		return t >= 0.f && MathF::Squared(t) <= LengthSqr();
	}
}
