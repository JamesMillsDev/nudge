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
	 * @brief Represents a ray in 3D space with an origin point and direction vector
	 *
	 * A ray is a half-line that starts at an origin point and extends infinitely
	 * in a specified direction. This class provides functionality for:
	 * - Ray construction and manipulation
	 * - Geometric queries (point containment, closest point calculation)
	 * - Intersection testing with various geometric primitives
	 */
	class Ray
	{
	public:
		/**
		 * @brief Factory method to create a ray from two points
		 * @param from Starting point of the ray (origin)
		 * @param to Target point to aim the ray towards
		 * @return Ray with origin at 'from' and normalized direction towards 'to'
		 */
		static Ray FromPoints(const Vector3& from, const Vector3& to);

	public:
		Vector3 origin;     ///< Starting point of the ray in 3D space
		Vector3 direction;  ///< Direction vector of the ray (should be normalized for most operations)

	public:
		/**
		 * @brief Default constructor - creates ray at origin pointing along positive Z-axis
		 */
		Ray();

		/**
		 * @brief Constructs a ray with specified origin and direction
		 * @param origin Starting point of the ray
		 * @param direction Direction vector of the ray (typically normalized)
		 */
		Ray(const Vector3& origin, const Vector3& direction);

	public:
		/**
		 * @brief Normalizes the ray's direction vector to unit length
		 *
		 * This ensures consistent behavior for geometric operations that assume
		 * a normalized direction vector.
		 */
		void Normalize();

		/**
		 * @brief Tests if a point lies on the ray
		 * @param point Point to test for containment
		 * @return True if the point lies on the ray (within floating-point tolerance)
		 */
		bool Contains(const Vector3& point) const;

		/**
		 * @brief Finds the closest point on the ray to a given point
		 * @param point Reference point to find closest approach to
		 * @return Point on the ray that is closest to the input point
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Performs ray-AABB intersection test
		 * @param other Axis-Aligned Bounding Box to test intersection against
		 * @return Distance along ray to intersection point, or -1 if no intersection
		 */
		float CastAgainst(const Aabb& other) const;

		/**
		 * @brief Performs ray-OBB intersection test
		 * @param other Oriented Bounding Box to test intersection against
		 * @return Distance along ray to intersection point, or -1 if no intersection
		 */
		float CastAgainst(const Obb& other) const;

		/**
		 * @brief Performs ray-plane intersection test
		 * @param other Plane to test intersection against
		 * @return Distance along ray to intersection point, or -1 if no intersection
		 */
		float CastAgainst(const Plane& other) const;

		/**
		 * @brief Performs ray-sphere intersection test
		 * @param other Sphere to test intersection against
		 * @return Distance along ray to intersection point, or -1 if no intersection
		 */
		float CastAgainst(const Sphere& other) const;

		/**
		 * @brief Performs ray-triangle intersection test using plane intersection and barycentric coordinates
		 * @param tri Triangle to test intersection against
		 * @return Distance along ray to intersection point, or -1 if no intersection
		 *
		 * This method uses a two-phase approach:
		 * 1. First tests ray intersection with the triangle's containing plane
		 * 2. Then validates that the intersection point lies within the triangle bounds
		 *    using barycentric coordinate testing
		 *
		 * The returned distance can be used to calculate the exact intersection point:
		 * intersectionPoint = ray.origin + ray.direction * returnedDistance
		 *
		 * @note This is more expensive than basic primitive tests due to the barycentric
		 *       coordinate calculation, but provides precise triangle intersection detection
		 * @see CastAgainst(const Plane&) for the underlying plane intersection logic
		 */
		float CastAgainst(const Triangle& tri) const;
	};
}