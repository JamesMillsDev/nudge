#pragma once

#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Plane;
	class Sphere;
	class Triangle;

	/**
	 * @brief Oriented Bounding Box (OBB) class for 3D collision detection with arbitrary rotation
	 */
	class Obb
	{
	public:
		/**
		 * @brief Center point of the OBB
		 */
		Vector3 origin;
		
		/**
		 * @brief Half-widths from origin to each face along local axes (always positive)
		 */
		Vector3 extents;
		
		/**
		 * @brief 3x3 rotation matrix defining the orientation of the OBB's local axes
		 */
		Matrix3 orientation;

	public:
		/**
		 * @brief Default constructor - creates an OBB at origin with zero extents and identity orientation
		 */
		Obb();
		
		/**
		 * @brief Constructor to create axis-aligned OBB with specified origin and extents
		 * @param origin The center point of the OBB
		 * @param extents The half-widths from origin to each face
		 */
		Obb(const Vector3& origin, const Vector3& extents);
		
		/**
		 * @brief Constructor to create OBB with specified origin, extents, and orientation
		 * @param origin The center point of the OBB
		 * @param extents The half-widths from origin to each face along local axes
		 * @param orientation The 3x3 rotation matrix defining the OBB's orientation
		 */
		Obb(const Vector3& origin, const Vector3& extents, const Matrix3& orientation);

	public:
		/**
		 * @brief Checks if a point is contained within the OBB
		 * @param point The point to test for containment
		 * @return True if the point is inside or on the boundary of the OBB
		 */
		bool Contains(const Vector3& point) const;
		
		/**
		 * @brief Returns the closest point on or inside the OBB to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on or inside the OBB
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Checks if this OBB intersects with an Axis-Aligned Bounding Box
		 * @param other The AABB to test intersection with
		 * @return True if the OBB and AABB intersect
		 */
		bool Intersects(const Aabb& other) const;
		
		/**
		 * @brief Checks if this OBB intersects with another Oriented Bounding Box
		 * @param other The other OBB to test intersection with
		 * @return True if the OBBs intersect or touch
		 */
		bool Intersects(const Obb& other) const;
		
		/**
		 * @brief Checks if this OBB intersects with a plane
		 * @param other The plane to test intersection with
		 * @return True if the OBB intersects the plane
		 */
		bool Intersects(const Plane& other) const;
		
		/**
		 * @brief Checks if this OBB intersects with a sphere
		 * @param other The sphere to test intersection with
		 * @return True if the OBB and sphere intersect
		 */
		bool Intersects(const Sphere& other) const;
		
		/**
		 * @brief Checks if this OBB intersects with a triangle
		 * @param other The triangle to test intersection with
		 * @return True if the OBB and triangle intersect
		 */
		bool Intersects(const Triangle& other) const;
	};
}
