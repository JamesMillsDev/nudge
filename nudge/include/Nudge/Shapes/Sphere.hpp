#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;
	class Triangle;

	/**
	 * @brief 3D sphere class for collision detection and geometric calculations
	 */
	class Sphere
	{
	public:
		/**
		 * @brief Center point of the sphere
		 */
		Vector3 origin;
		
		/**
		 * @brief Radius of the sphere (distance from center to surface)
		 */
		float radius;

	public:
		/**
		 * @brief Default constructor - creates a sphere at origin with zero radius
		 */
		Sphere();
		
		/**
		 * @brief Constructor to create sphere with specified center and radius
		 * @param origin The center point of the sphere
		 * @param radius The radius of the sphere
		 */
		Sphere(const Vector3& origin, float radius);

	public:
		/**
		 * @brief Checks if a point is contained within the sphere
		 * @param point The point to test for containment
		 * @return True if the point is inside or on the surface of the sphere
		 */
		bool Contains(const Vector3& point) const;
		
		/**
		 * @brief Returns the closest point on the sphere surface to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on the sphere surface (or the point itself if inside)
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Checks if this sphere intersects with another sphere
		 * @param other The other sphere to test intersection with
		 * @return True if the spheres intersect or touch
		 */
		bool Intersects(const Sphere& other) const;
		
		/**
		 * @brief Checks if this sphere intersects with an Axis-Aligned Bounding Box
		 * @param other The AABB to test intersection with
		 * @return True if the sphere and AABB intersect
		 */
		bool Intersects(const Aabb& other) const;
		
		/**
		 * @brief Checks if this sphere intersects with an Oriented Bounding Box
		 * @param other The OBB to test intersection with
		 * @return True if the sphere and OBB intersect
		 */
		bool Intersects(const Obb& other) const;
		
		/**
		 * @brief Checks if this sphere intersects with a plane
		 * @param other The plane to test intersection with
		 * @return True if the sphere intersects or touches the plane
		 */
		bool Intersects(const Plane& other) const;
		
		/**
		 * @brief Checks if this sphere intersects with a triangle
		 * @param other The triangle to test intersection with
		 * @return True if the sphere and triangle intersect
		 */
		bool Intersects(const Triangle& other) const;
	};
}