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
	 * @brief 3D ray class for raycasting and geometric queries
	 */
	class Ray
	{
	public:
		/**
		 * @brief Creates a ray from two points
		 * @param from The starting point of the ray
		 * @param to The target point to aim the ray towards
		 * @return A new ray starting at 'from' and pointing towards 'to'
		 */
		static Ray FromPoints(const Vector3& from, const Vector3& to);

	public:
		/**
		 * @brief Starting point of the ray
		 */
		Vector3 origin;
		
		/**
		 * @brief Direction vector of the ray
		 */
		Vector3 direction;

	public:
		/**
		 * @brief Default constructor - creates a ray at origin pointing along positive X axis
		 */
		Ray();

		/**
		 * @brief Constructor to create ray with specified origin and direction
		 * @param origin The starting point of the ray
		 * @param direction The direction vector of the ray
		 */
		Ray(const Vector3& origin, const Vector3& direction);

	public:
		/**
		 * @brief Normalizes the direction vector to unit length
		 */
		void Normalize();

		/**
		 * @brief Checks if a point lies exactly on the ray
		 * @param point The point to test
		 * @return True if the point is on the ray (within floating point tolerance)
		 */
		bool Contains(const Vector3& point) const;

		/**
		 * @brief Returns the closest point on the ray to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on the ray (may be behind the origin if point is behind ray start)
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Performs raycast against an Axis-Aligned Bounding Box
		 * @param other The AABB to cast against
		 * @param hit Optional pointer to store hit information
		 * @return True if the ray intersects the AABB
		 */
		bool CastAgainst(const Aabb& other, RaycastHit* hit = nullptr) const;

		/**
		 * @brief Performs raycast against a mesh
		 * @param other The mesh to cast against
		 * @return The distance to the closest intersection, or negative value if no hit
		 */
		float CastAgainst(const Mesh& other);

		/**
		 * @brief Performs raycast against an Oriented Bounding Box
		 * @param other The OBB to cast against
		 * @param hit Optional pointer to store hit information
		 * @return True if the ray intersects the OBB
		 */
		bool CastAgainst(const Obb& other, RaycastHit* hit = nullptr) const;

		/**
		 * @brief Performs raycast against a plane
		 * @param other The plane to cast against
		 * @param hit Optional pointer to store hit information
		 * @return True if the ray intersects the plane
		 */
		bool CastAgainst(const Plane& other, RaycastHit* hit = nullptr) const;

		/**
		 * @brief Performs raycast against a sphere
		 * @param other The sphere to cast against
		 * @param hit Optional pointer to store hit information
		 * @return True if the ray intersects the sphere
		 */
		bool CastAgainst(const Sphere& other, RaycastHit* hit = nullptr) const;

		/**
		 * @brief Performs raycast against a triangle
		 * @param tri The triangle to cast against
		 * @param hit Optional pointer to store hit information
		 * @return True if the ray intersects the triangle
		 */
		bool CastAgainst(const Triangle& tri, RaycastHit* hit = nullptr) const;

	private:
		/**
		 * @brief Resets a RaycastHit structure to default values
		 * @param hit Pointer to the RaycastHit structure to reset
		 */
		static void ResetHit(RaycastHit* hit);
	};
}