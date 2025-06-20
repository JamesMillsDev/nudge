#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Plane;
	class Sphere;

	/**
	 * @brief Represents a triangle in 3D space defined by three vertices
	 *
	 * A triangle is a fundamental geometric primitive consisting of three vertices
	 * that form a planar surface. This class provides functionality for:
	 * - Triangle construction and vertex access
	 * - Geometric queries (point containment, closest point, barycentric coordinates)
	 * - Intersection testing with various geometric primitives
	 *
	 * The class uses a union to provide multiple ways of accessing vertex data:
	 * - Named vertices (a, b, c)
	 * - Array access (points[0], points[1], points[2])
	 * - Raw float array (values[0-8] for x,y,z components)
	 */
	class Triangle
	{
	public:
		/**
		 * @brief Union providing multiple access patterns for triangle vertices
		 *
		 * This design allows flexible access to vertex data depending on usage context:
		 * - Semantic access: tri.a, tri.b, tri.c
		 * - Indexed access: tri.points[i]
		 * - Component access: tri.values[i] for raw float manipulation
		 */
		union
		{
			struct
			{
				Vector3 a; ///< First vertex of the triangle
				Vector3 b; ///< Second vertex of the triangle
				Vector3 c; ///< Third vertex of the triangle
			};

			Vector3 points[3]; ///< Array access to vertices [a, b, c]
			float values[9];   ///< Raw float access to all components [ax,ay,az, bx,by,bz, cx,cy,cz]
		};

	public:
		/**
		 * @brief Default constructor - creates a degenerate triangle at origin
		 *
		 * All three vertices will be initialized to (0,0,0), creating a
		 * zero-area triangle that may require special handling in geometric operations.
		 */
		Triangle();

		/**
		 * @brief Constructs a triangle from three specified vertices
		 * @param a First vertex of the triangle
		 * @param b Second vertex of the triangle
		 * @param c Third vertex of the triangle
		 *
		 * Vertex order determines the triangle's winding and normal direction
		 * using the right-hand rule: normal = normalize(cross(b-a, c-a))
		 */
		Triangle(const Vector3& a, const Vector3& b, const Vector3& c);

	public:
		/**
		 * @brief Tests if a point lies within the triangle
		 * @param point Point to test for containment
		 * @return True if the point lies inside or on the triangle boundary
		 *
		 * Uses barycentric coordinates to determine if the point lies within
		 * the triangle's area. The point must be coplanar with the triangle.
		 */
		bool Contains(const Vector3& point) const;

		/**
		 * @brief Finds the closest point on the triangle to a given point
		 * @param point Reference point to find closest approach to
		 * @return Point on the triangle (vertex, edge, or face) closest to input point
		 *
		 * The closest point may lie on a vertex, edge, or within the triangle face
		 * depending on the input point's position relative to the triangle.
		 */
		Vector3 ClosestPoint(const Vector3& point) const;

		/**
		 * @brief Calculates barycentric coordinates of a point relative to the triangle
		 * @param point Point to convert to barycentric coordinates
		 * @return Vector3 containing barycentric coordinates (u, v, w) where:
		 *         - point = u*a + v*b + w*c
		 *         - u + v + w = 1.0 (for points on triangle plane)
		 *         - All coordinates >= 0 if point is inside triangle
		 *
		 * Barycentric coordinates provide a natural way to express points relative
		 * to triangle vertices and are essential for triangle intersection testing.
		 */
		Vector3 Barycentric(const Vector3& point) const;

		/**
		 * @brief Tests if the triangle intersects with an Axis-Aligned Bounding Box
		 * @param other AABB to test intersection against
		 * @return True if the triangle intersects, touches, or is contained within the AABB
		 */
		bool Intersects(const Aabb& other) const;

		/**
		 * @brief Tests if the triangle intersects with an Oriented Bounding Box
		 * @param other OBB to test intersection against
		 * @return True if the triangle intersects, touches, or is contained within the OBB
		 */
		bool Intersects(const Obb& other) const;

		/**
		 * @brief Tests if the triangle intersects with a plane
		 * @param other Plane to test intersection against
		 * @return True if the triangle crosses, touches, or lies on the plane
		 *
		 * A triangle intersects a plane if:
		 * - Any vertex lies on the plane, or
		 * - Vertices are on opposite sides of the plane (triangle crosses plane)
		 */
		bool Intersects(const Plane& other) const;

		/**
		 * @brief Tests if the triangle intersects with a sphere
		 * @param other Sphere to test intersection against
		 * @return True if the triangle intersects or is contained within the sphere
		 *
		 * Tests intersection by finding the closest point on the triangle to the
		 * sphere center and checking if it's within the sphere radius.
		 */
		bool Intersects(const Sphere& other) const;

		/**
		 * @brief Tests if this triangle intersects with another triangle
		 * @param other Triangle to test intersection against
		 * @return True if the triangles intersect, touch, or overlap
		 *
		 * Triangle-triangle intersection is complex and may involve:
		 * - Edge-edge intersections
		 * - Vertex-triangle containment
		 * - Coplanar triangle overlap testing
		 */
		bool Intersects(const Triangle& other) const;
	};
}