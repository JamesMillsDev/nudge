#pragma once

#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/Shape.hpp"

namespace Nudge
{
	class Obb;
	class Plane;
	class Sphere;
	class Triangle;

	/**
	 * @brief Axis-Aligned Bounding Box (AABB) class for 3D collision detection and spatial queries
	 */
	class Aabb : public Shape
	{
	public:
		/**
		 * @brief Creates an AABB from minimum and maximum corner points
		 * @param min The minimum corner point of the bounding box
		 * @param max The maximum corner point of the bounding box
		 * @return A new AABB constructed from the given min/max points
		 */
		static Aabb FromMinMax(const Vector3& min, const Vector3& max);

	public:
		/**
		 * @brief Center point of the AABB
		 */
		Vector3 origin;
		
		/**
		 * @brief Half-widths from origin to each face (always positive)
		 */
		Vector3 extents;

	public:
		/**
		 * @brief Default constructor - creates an AABB at origin with zero extents
		 */
		Aabb();
		
		/**
		 * @brief Constructor to create AABB with specified origin and extents
		 * @param origin The center point of the AABB
		 * @param extents The half-widths from origin to each face
		 */
		Aabb(const Vector3& origin, const Vector3& extents);

	public:
		/**
		 * @brief Returns the minimum corner point of the AABB
		 * @return The minimum corner point (origin - extents)
		 */
		Vector3 Min() const;
		
		/**
		 * @brief Returns the maximum corner point of the AABB
		 * @return The maximum corner point (origin + extents)
		 */
		Vector3 Max() const;

		/**
		 * @brief Checks if a point is contained within the AABB
		 * @param point The point to test for containment
		 * @return True if the point is inside or on the boundary of the AABB
		 */
		bool Contains(const Vector3& point) const override;
		
		/**
		 * @brief Returns the closest point on or inside the AABB to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on or inside the AABB
		 */
		Vector3 ClosestPoint(const Vector3& point) const override;

		/**
		 * @brief Checks if this AABB intersects with another AABB
		 * @param other The other AABB to test intersection with
		 * @return True if the AABBs intersect or touch
		 */
		bool Intersects(const Aabb& other) const;
		
		/**
		 * @brief Checks if this AABB intersects with an Oriented Bounding Box
		 * @param other The OBB to test intersection with
		 * @return True if the AABB and OBB intersect
		 */
		bool Intersects(const Obb& other) const;
		
		/**
		 * @brief Checks if this AABB intersects with a plane
		 * @param other The plane to test intersection with
		 * @return True if the AABB intersects the plane
		 */
		bool Intersects(const Plane& other) const;
		
		/**
		 * @brief Checks if this AABB intersects with a sphere
		 * @param other The sphere to test intersection with
		 * @return True if the AABB and sphere intersect
		 */
		bool Intersects(const Sphere& other) const;
		
		/**
		 * @brief Checks if this AABB intersects with a triangle
		 * @param other The triangle to test intersection with
		 * @return True if the AABB and triangle intersect
		 */
		bool Intersects(const Triangle& other) const;
	};
}