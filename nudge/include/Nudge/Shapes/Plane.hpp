#pragma once

#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/Shape.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Sphere;
	class Triangle;

	/**
	 * @brief 3D plane class for geometric calculations and collision detection
	 */
	class Plane : public Shape
	{
	public:
		/**
		 * @brief Creates a plane from a triangle using its vertices
		 * @param tri The triangle to create the plane from
		 * @return A new plane that contains the triangle
		 */
		static Plane From(const Triangle& tri);

		/**
		 * @brief Evaluates the plane equation for a given point
		 * @param point The point to evaluate
		 * @param plane The plane to use in the equation
		 * @return The signed distance from the point to the plane (positive = front side, negative = back side, zero = on plane)
		 */
		static float PlaneEquation(const Vector3& point, const Plane& plane);

	public:
		/**
		 * @brief Unit normal vector of the plane pointing away from the origin
		 */
		Vector3 normal;
		
		/**
		 * @brief Signed distance from the origin to the plane along the normal direction
		 */
		float distance;

	public:
		/**
		 * @brief Default constructor - creates a plane at origin with up normal (0,1,0)
		 */
		Plane();
		
		/**
		 * @brief Constructor to create plane with specified normal and distance
		 * @param normal The unit normal vector of the plane
		 * @param distance The signed distance from origin to the plane
		 */
		Plane(const Vector3& normal, float distance);

	public:
		/**
		 * @brief Checks if a point lies exactly on the plane
		 * @param point The point to test
		 * @return True if the point is on the plane (within floating point tolerance)
		 */
		bool Contains(const Vector3& point) const override;
		
		/**
		 * @brief Returns the closest point on the plane to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on the plane (orthogonal projection)
		 */
		Vector3 ClosestPoint(const Vector3& point) const override;

		/**
		 * @brief Checks if this plane intersects with an Axis-Aligned Bounding Box
		 * @param other The AABB to test intersection with
		 * @return True if the plane intersects or touches the AABB
		 */
		bool Intersects(const Aabb& other) const;
		
		/**
		 * @brief Checks if this plane intersects with an Oriented Bounding Box
		 * @param other The OBB to test intersection with
		 * @return True if the plane intersects or touches the OBB
		 */
		bool Intersects(const Obb& other) const;
		
		/**
		 * @brief Checks if this plane intersects with another plane
		 * @param other The other plane to test intersection with
		 * @return True if the planes intersect (not parallel)
		 */
		bool Intersects(const Plane& other) const;
		
		/**
		 * @brief Checks if this plane intersects with a sphere
		 * @param other The sphere to test intersection with
		 * @return True if the plane intersects or touches the sphere
		 */
		bool Intersects(const Sphere& other) const;
		
		/**
		 * @brief Checks if this plane intersects with a triangle
		 * @param other The triangle to test intersection with
		 * @return True if the plane intersects or touches the triangle
		 */
		bool Intersects(const Triangle& other) const;
	};
}