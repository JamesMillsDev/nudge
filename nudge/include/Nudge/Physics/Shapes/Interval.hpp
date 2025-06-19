#pragma once

#include "Nudge/Maths/Vector3.hpp"

namespace Nudge
{
	class Aabb;
	class Obb;
	class Triangle;

	/**
	 * @brief Utility class for Separating Axis Theorem (SAT) collision detection
	 *
	 * The Interval class represents a 1D projection of a geometric shape onto an axis.
	 * It provides methods to:
	 * - Project various shapes (AABB, OBB, Triangle) onto arbitrary axes
	 * - Test for overlap between projected intervals on specific axes
	 * - Perform complete SAT-based intersection tests between shape pairs
	 *
	 * SAT Theory:
	 * Two convex shapes are NOT intersecting if there exists a separating axis
	 * where their projections do not overlap. If no separating axis is found
	 * after testing all potential axes, the shapes must be intersecting.
	 */
	class Interval
	{
	public:
		/**
		 * @brief Projects an AABB onto a given axis
		 * @param aabb Axis-Aligned Bounding Box to project
		 * @param axis Direction vector to project onto (should be normalized)
		 * @return Interval representing the min/max projection values on the axis
		 *
		 * Projects all 8 corners of the AABB onto the axis and finds the
		 * minimum and maximum projection values.
		 */
		static Interval Get(const Aabb& aabb, const Vector3& axis);

		/**
		 * @brief Projects an OBB onto a given axis
		 * @param obb Oriented Bounding Box to project
		 * @param axis Direction vector to project onto (should be normalized)
		 * @return Interval representing the min/max projection values on the axis
		 *
		 * Uses the OBB's center and extents to efficiently calculate the
		 * projection interval without explicitly computing all 8 corners.
		 */
		static Interval Get(const Obb& obb, const Vector3& axis);

		/**
		 * @brief Projects a triangle onto a given axis
		 * @param tri Triangle to project
		 * @param axis Direction vector to project onto (should be normalized)
		 * @return Interval representing the min/max projection values on the axis
		 *
		 * Projects all three vertices of the triangle onto the axis and finds
		 * the minimum and maximum projection values.
		 */
		static Interval Get(const Triangle& tri, const Vector3& axis);

		/**
		 * @brief Tests if AABB and OBB projections overlap on a specific axis
		 * @param aabb Axis-Aligned Bounding Box
		 * @param obb Oriented Bounding Box
		 * @param axis Separating axis candidate to test
		 * @return True if projections overlap (no separation on this axis)
		 *
		 * Helper function for AABB-OBB SAT testing. If this returns false,
		 * the axis is a separating axis and the shapes do not intersect.
		 */
		static bool OverlapOnAxis(const Aabb& aabb, const Obb& obb, const Vector3& axis);

		/**
		 * @brief Complete SAT-based intersection test between AABB and OBB
		 * @param aabb Axis-Aligned Bounding Box
		 * @param obb Oriented Bounding Box
		 * @return True if the shapes intersect
		 *
		 * Tests all potential separating axes:
		 * - 3 AABB face normals (world X, Y, Z axes)
		 * - 3 OBB face normals
		 * - 9 cross products of AABB and OBB edge directions
		 */
		static bool AabbObb(const Aabb& aabb, const Obb& obb);

		/**
		 * @brief Tests if two OBB projections overlap on a specific axis
		 * @param a First Oriented Bounding Box
		 * @param b Second Oriented Bounding Box
		 * @param axis Separating axis candidate to test
		 * @return True if projections overlap (no separation on this axis)
		 */
		static bool OverlapOnAxis(const Obb& a, const Obb& b, const Vector3& axis);

		/**
		 * @brief Complete SAT-based intersection test between two OBBs
		 * @param a First Oriented Bounding Box
		 * @param b Second Oriented Bounding Box
		 * @return True if the OBBs intersect
		 *
		 * Tests all potential separating axes:
		 * - 3 face normals from OBB A
		 * - 3 face normals from OBB B
		 * - 9 cross products of edge directions from both OBBs
		 */
		static bool ObbObb(const Obb& a, const Obb& b);

		/**
		 * @brief Tests if triangle and AABB projections overlap on a specific axis
		 * @param tri Triangle
		 * @param aabb Axis-Aligned Bounding Box
		 * @param axis Separating axis candidate to test
		 * @return True if projections overlap (no separation on this axis)
		 */
		static bool OverlapOnAxis(const Triangle& tri, const Aabb& aabb, const Vector3& axis);

		/**
		 * @brief Complete SAT-based intersection test between triangle and AABB
		 * @param tri Triangle
		 * @param aabb Axis-Aligned Bounding Box
		 * @return True if the triangle and AABB intersect
		 *
		 * Tests potential separating axes including:
		 * - Triangle face normal
		 * - AABB face normals (world X, Y, Z axes)
		 * - Cross products of triangle edges with AABB edge directions
		 */
		static bool TriangleAabb(const Triangle& tri, const Aabb& aabb);

		/**
		 * @brief Tests if triangle and OBB projections overlap on a specific axis
		 * @param tri Triangle
		 * @param obb Oriented Bounding Box
		 * @param axis Separating axis candidate to test
		 * @return True if projections overlap (no separation on this axis)
		 */
		static bool OverlapOnAxis(const Triangle& tri, const Obb& obb, const Vector3& axis);

		/**
		 * @brief Complete SAT-based intersection test between triangle and OBB
		 * @param tri Triangle
		 * @param obb Oriented Bounding Box
		 * @return True if the triangle and OBB intersect
		 *
		 * Tests potential separating axes including:
		 * - Triangle face normal
		 * - OBB face normals
		 * - Cross products of triangle edges with OBB edge directions
		 */
		static bool TriangleObb(const Triangle& tri, const Obb& obb);

		/**
		 * @brief Tests if two triangle projections overlap on a specific axis
		 * @param t1 First triangle
		 * @param t2 Second triangle
		 * @param axis Separating axis candidate to test
		 * @return True if projections overlap (no separation on this axis)
		 */
		static bool OverlapOnAxis(const Triangle& t1, const Triangle& t2, const Vector3& axis);

		/**
		 * @brief Complete SAT-based intersection test between two triangles
		 * @param t1 First triangle
		 * @param t2 Second triangle
		 * @return True if the triangles intersect
		 *
		 * Tests potential separating axes including:
		 * - Face normals of both triangles
		 * - Cross products of edge pairs from both triangles
		 * This is one of the most complex SAT tests due to the number of potential axes.
		 */
		static bool TriangleTriangle(const Triangle& t1, const Triangle& t2);

		/**
		 * @brief Utility function to compute cross product of two edges defined by points
		 * @param a Start point of first edge
		 * @param b End point of first edge
		 * @param c Start point of second edge
		 * @param d End point of second edge
		 * @return Cross product of edges (b-a) x (d-c)
		 *
		 * Helper function for generating separating axis candidates from edge pairs.
		 * The resulting vector is perpendicular to both edges and can serve as
		 * a potential separating axis in SAT testing.
		 */
		static Vector3 CrossEdge(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);

	public:
		float min;  ///< Minimum projection value on the axis
		float max;  ///< Maximum projection value on the axis
	};
}