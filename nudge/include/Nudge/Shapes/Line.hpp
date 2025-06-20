#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;
	class Sphere;
	class Triangle;

	/**
	 * @brief Represents a line segment in 3D space defined by two endpoints
	 *
	 * A line segment is a finite portion of a line bounded by two distinct points.
	 * Unlike a Ray which extends infinitely in one direction, a Line has a definite
	 * start and end point. This class provides functionality for:
	 * - Line segment construction and properties (length calculation)
	 * - Geometric queries (point containment, closest point calculation)
	 * - Intersection testing with various geometric primitives
	 */
	class Line
	{
	public:
		Vector3 start;  ///< Starting endpoint of the line segment
		Vector3 end;    ///< Ending endpoint of the line segment

	public:
		/**
		 * @brief Default constructor - creates a degenerate line at origin
		 *
		 * Both start and end points will be initialized to (0,0,0),
		 * creating a zero-length line segment.
		 */
		Line();

		/**
		 * @brief Constructs a line segment between two specified points
		 * @param start Starting endpoint of the line segment
		 * @param end Ending endpoint of the line segment
		 */
		Line(const Vector3& start, const Vector3& end);

	public:
		/**
		 * @brief Calculates the length of the line segment
		 * @return Euclidean distance between start and end points
		 */
		float Length() const;

		/**
		 * @brief Calculates the squared length of the line segment
		 * @return Squared Euclidean distance between start and end points
		 *
		 * This method is more efficient than Length() when you only need
		 * to compare lengths or when the actual distance value isn't required.
		 */
		float LengthSqr() const;

	public:
		/**
		 * @brief Tests if a point lies on the line segment
		 * @param point Point to test for containment
		 * @return True if the point lies on the line segment (within floating-point tolerance)
		 *
		 * The point must lie between the start and end points to be considered
		 * contained within the line segment.
		 */
		bool Contains(const Vector3& point) const;

		/**
		 * @brief Finds the closest point on the line segment to a given point
		 * @param point Reference point to find closest approach to
		 * @return Point on the line segment that is closest to the input point
		 *
		 * The returned point will always lie between the start and end points
		 * (inclusive) of the line segment.
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Tests if the line segment intersects with an Axis-Aligned Bounding Box
		 * @param other AABB to test intersection against
		 * @return True if the line segment intersects or is contained within the AABB
		 */
		bool Test(const Aabb& other) const;

		/**
		 * @brief Tests if the line segment intersects with an Oriented Bounding Box
		 * @param other OBB to test intersection against
		 * @return True if the line segment intersects or is contained within the OBB
		 */
		bool Test(const Obb& other) const;

		/**
		 * @brief Tests if the line segment intersects with a plane
		 * @param other Plane to test intersection against
		 * @return True if the line segment crosses or touches the plane
		 */
		bool Test(const Plane& other) const;

		/**
		 * @brief Tests if the line segment intersects with a sphere
		 * @param other Sphere to test intersection against
		 * @return True if the line segment intersects or is contained within the sphere
		 */
		bool Test(const Sphere& other) const;

		/**
		 * @brief Tests if the line segment intersects with a triangle
		 * @param other Triangle to test intersection against
		 * @return True if the line segment intersects the triangle
		 *
		 * This method determines whether the finite line segment crosses, touches, or
		 * passes through the triangle in 3D space. The test involves:
		 * 1. Converting the line segment to a ray representation
		 * 2. Performing ray-triangle intersection using plane intersection + barycentric coordinates
		 * 3. Validating that the intersection point lies within the line segment bounds
		 *
		 * The intersection is considered valid if:
		 * - The ray intersects the triangle's plane
		 * - The intersection point lies within the triangle's boundaries
		 * - The intersection occurs between the line segment's start and end points
		 *
		 * @note This test is more computationally expensive than primitive shape tests
		 *       due to the underlying barycentric coordinate calculations
		 * @note Unlike ray-triangle intersection, this method respects the finite nature
		 *       of the line segment and will return false for intersections beyond the endpoints
		 * @see Ray::CastAgainst(const Triangle&) for the underlying intersection algorithm
		 * @see Test(const Plane&) for simpler plane intersection testing
		 */
		bool Test(const Triangle& other) const;

	};
}