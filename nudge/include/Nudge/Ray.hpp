#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Mesh;
	class Obb;
	class Plane;
	class Sphere;
	class Triangle;

	struct RaycastHit;

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

		bool CastAgainst(const Aabb& other, RaycastHit* hit = nullptr) const;

		float CastAgainst(const Mesh& other) const;

		bool CastAgainst(const Obb& other, RaycastHit* hit = nullptr) const;

		bool CastAgainst(const Plane& other, RaycastHit* hit = nullptr) const;

		bool CastAgainst(const Sphere& other, RaycastHit* hit = nullptr) const;

		bool CastAgainst(const Triangle& tri, RaycastHit* hit = nullptr) const;

	private:
		static void ResetHit(RaycastHit* hit);

	};
}