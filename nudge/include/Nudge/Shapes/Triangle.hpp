#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;
	class Sphere;

	/**
	 * @brief 3D triangle class for geometric calculations and collision detection
	 */
	class Triangle
	{
	public:
		union
		{
			struct
			{
				/**
				 * @brief First vertex of the triangle
				 */
				Vector3 a;
				
				/**
				 * @brief Second vertex of the triangle
				 */
				Vector3 b;
				
				/**
				 * @brief Third vertex of the triangle
				 */
				Vector3 c;
			};

			/**
			 * @brief Array access to the three vertices
			 */
			Vector3 points[3];
			
			/**
			 * @brief Raw float array access to all 9 coordinate values (x,y,z for each vertex)
			 */
			float values[9];
		};

	public:
		/**
		 * @brief Default constructor - creates a triangle with all vertices at origin
		 */
		Triangle();

		/**
		 * @brief Constructor to create triangle from three vertices
		 * @param a The first vertex of the triangle
		 * @param b The second vertex of the triangle
		 * @param c The third vertex of the triangle
		 */
		Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

		/**
		 * @brief Copy constructor
		 * @param other The triangle to copy from
		 */
		Triangle(const Triangle& other);

	public:
		/**
		 * @brief Checks if a point is contained within the triangle
		 * @param point The point to test for containment
		 * @return True if the point is inside the triangle (coplanar and within bounds)
		 */
		bool Contains(const Vector3& point) const;

		/**
		 * @brief Returns the closest point on the triangle to the given point
		 * @param point The point to find the closest point to
		 * @return The closest point on the triangle surface (vertex, edge, or face)
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Calculates the barycentric coordinates of a point relative to the triangle
		 * @param point The point to calculate barycentric coordinates for
		 * @return Vector3 containing the barycentric coordinates (u, v, w) where point = u*a + v*b + w*c
		 */
		Vector3 Barycentric(const Vector3& point) const;

		/**
		 * @brief Checks if this triangle intersects with an Axis-Aligned Bounding Box
		 * @param other The AABB to test intersection with
		 * @return True if the triangle and AABB intersect
		 */
		bool Intersects(const Aabb& other) const;

		/**
		 * @brief Checks if this triangle intersects with an Oriented Bounding Box
		 * @param other The OBB to test intersection with
		 * @return True if the triangle and OBB intersect
		 */
		bool Intersects(const Obb& other) const;

		/**
		 * @brief Checks if this triangle intersects with a plane
		 * @param other The plane to test intersection with
		 * @return True if the triangle intersects or touches the plane
		 */
		bool Intersects(const Plane& other) const;

		/**
		 * @brief Checks if this triangle intersects with a sphere
		 * @param other The sphere to test intersection with
		 * @return True if the triangle and sphere intersect
		 */
		bool Intersects(const Sphere& other) const;

		/**
		 * @brief Checks if this triangle intersects with another triangle
		 * @param other The other triangle to test intersection with
		 * @return True if the triangles intersect or touch
		 */
		bool Intersects(const Triangle& other) const;

	public:
		/**
		 * @brief Assignment operator
		 * @param rhs The triangle to assign from
		 * @return Reference to this triangle after assignment
		 */
		Triangle& operator=(const Triangle& rhs);
	};
}