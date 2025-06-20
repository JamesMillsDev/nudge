#include "Nudge/Shapes/Interval.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Triangle.hpp"

#include <algorithm>

namespace Nudge
{
	/**
	 * @brief Projects an AABB onto a given axis by computing all 8 corner projections
	 * @param aabb Axis-Aligned Bounding Box to project
	 * @param axis Direction vector to project onto
	 * @return Interval containing min/max projection values
	 */
	Interval Interval::Get(const Aabb& aabb, const Vector3& axis)
	{
		Vector3 min = aabb.Min();
		Vector3 max = aabb.Max();

		// Generate all 8 vertices of the AABB by combining min/max components
		const Vector3 vertex[8] =
		{
			{ min.x, max.y, max.z },  // Bottom-front-left to top-back-right combinations
			{ min.x, max.y, min.z },
			{ min.x, min.y, max.z },
			{ min.x, min.y, min.z },
			{ max.x, max.y, max.z },
			{ max.x, max.y, min.z },
			{ max.x, min.y, max.z },
			{ max.x, min.y, min.z }
		};

		Interval result;
		// Initialize with first vertex projection
		result.min = result.max = Vector3::Dot(axis, vertex[0]);

		// Find min/max projections across all vertices
		for (int i = 1; i < 8; ++i)
		{
			float projection = Vector3::Dot(axis, vertex[i]);

			result.min = projection < result.min ? projection : result.min;
			result.max = projection > result.max ? projection : result.max;
		}

		return result;
	}

	/**
	 * @brief Projects an OBB onto a given axis by computing all 8 corner projections
	 * @param obb Oriented Bounding Box to project
	 * @param axis Direction vector to project onto
	 * @return Interval containing min/max projection values
	 */
	Interval Interval::Get(const Obb& obb, const Vector3& axis)
	{
		Vector3 vertex[8];

		Vector3 origin = obb.origin;
		Vector3 extents = obb.extents;

		// Extract OBB's local coordinate system axes
		Vector3 axes[] = // OBB Axis 
		{
			obb.orientation.GetColumn(0),  // Local X-axis
			obb.orientation.GetColumn(1),  // Local Y-axis
			obb.orientation.GetColumn(2)   // Local Z-axis
		};

		// Generate all 8 vertices by combining +/- extents along each local axis
		vertex[0] = origin + axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
		vertex[1] = origin - axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
		vertex[2] = origin + axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];
		vertex[3] = origin + axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
		vertex[4] = origin - axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
		vertex[5] = origin + axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
		vertex[6] = origin - axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
		vertex[7] = origin - axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];

		Interval result;
		// Initialize with first vertex projection
		result.min = result.max = Vector3::Dot(axis, vertex[0]);

		// Find min/max projections across all vertices
		for (int i = 1; i < 8; ++i)
		{
			float projection = Vector3::Dot(axis, vertex[i]);

			result.min = projection < result.min ? projection : result.min;
			result.max = projection > result.max ? projection : result.max;
		}

		return result;
	}

	/**
	 * @brief Projects a triangle onto a given axis by projecting all 3 vertices
	 * @param tri Triangle to project
	 * @param axis Direction vector to project onto
	 * @return Interval containing min/max projection values
	 */
	Interval Interval::Get(const Triangle& tri, const Vector3& axis)
	{
		Interval result;

		// Initialize with first vertex projection
		result.min = Vector3::Dot(axis, tri.points[0]);
		result.max = result.min;

		// Find min/max projections across remaining vertices
		for (int i = 1; i < 3; ++i)
		{
			const float value = Vector3::Dot(axis, tri.points[i]);
			result.min = MathF::Min(result.min, value);
			result.max = MathF::Max(result.max, value);
		}

		return result;
	}

	/**
	 * @brief Tests if AABB and OBB projections overlap on a specific axis
	 * @param aabb Axis-Aligned Bounding Box
	 * @param obb Oriented Bounding Box
	 * @param axis Separating axis candidate to test
	 * @return True if projections overlap (no separation on this axis)
	 */
	bool Interval::OverlapOnAxis(const Aabb& aabb, const Obb& obb, const Vector3& axis)
	{
		// Use structured binding to extract interval bounds
		const auto [aMin, aMax] = Get(aabb, axis);
		const auto [bMin, bMax] = Get(obb, axis);

		// Intervals overlap if neither is completely before the other
		return bMin <= aMax && aMin <= bMax;
	}

	/**
	 * @brief Complete SAT-based intersection test between AABB and OBB
	 * @param aabb Axis-Aligned Bounding Box
	 * @param obb Oriented Bounding Box
	 * @return True if the shapes intersect
	 *
	 * Tests 15 potential separating axes:
	 * - 3 AABB face normals (world axes)
	 * - 3 OBB face normals
	 * - 9 cross products between AABB and OBB edge directions
	 */
	bool Interval::AabbObb(const Aabb& aabb, const Obb& obb)
	{
		Vector3 test[15] =
		{
			{ 1.f, 0.f, 0.f },            // AABB axis 1 (world X)
			{ 0.f, 1.f, 0.f },            // AABB axis 2 (world Y)
			{ 0.f, 0.f, 1.f },            // AABB axis 3 (world Z)
			obb.orientation.GetColumn(0), // OBB axis 1
			obb.orientation.GetColumn(1), // OBB axis 2
			obb.orientation.GetColumn(2)  // OBB axis 3
		};

		// Generate 9 cross product axes: AABB_edge x OBB_edge
		for (int i = 0; i < 3; ++i)
		{
			test[6 + i * 3 + 0] = Vector3::Cross(test[i], test[3]);  // Fixed indexing
			test[6 + i * 3 + 1] = Vector3::Cross(test[i], test[4]);
			test[6 + i * 3 + 2] = Vector3::Cross(test[i], test[5]);
		}

		// If all axes show overlap, shapes intersect
		return std::ranges::all_of(test, [&](const Vector3& axis)
			{
				return OverlapOnAxis(aabb, obb, axis);
			});
	}

	/**
	 * @brief Tests if two OBB projections overlap on a specific axis
	 */
	bool Interval::OverlapOnAxis(const Obb& a, const Obb& b, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(a, axis);
		const auto [bMin, bMax] = Get(b, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	/**
	 * @brief Complete SAT-based intersection test between two OBBs
	 * @param a First Oriented Bounding Box
	 * @param b Second Oriented Bounding Box
	 * @return True if the OBBs intersect
	 *
	 * Tests 15 potential separating axes:
	 * - 3 face normals from OBB A
	 * - 3 face normals from OBB B
	 * - 9 cross products between edge directions from both OBBs
	 */
	bool Interval::ObbObb(const Obb& a, const Obb& b)
	{
		Vector3 test[15] =
		{
			a.orientation.GetColumn(0),  // OBB A face normals
			a.orientation.GetColumn(1),
			a.orientation.GetColumn(2),
			b.orientation.GetColumn(0),  // OBB B face normals
			b.orientation.GetColumn(1),
			b.orientation.GetColumn(2),
		};

		// Generate 9 cross product axes: A_edge x B_edge
		for (int i = 0; i < 3; ++i)
		{
			test[6 + i * 3 + 0] = Vector3::Cross(test[i], test[3]);
			test[6 + i * 3 + 1] = Vector3::Cross(test[i], test[4]);
			test[6 + i * 3 + 2] = Vector3::Cross(test[i], test[5]);
		}

		return std::ranges::all_of(test, [&](const Vector3& axis)
			{
				return OverlapOnAxis(a, b, axis);
			});
	}

	/**
	 * @brief Tests if triangle and AABB projections overlap on a specific axis
	 */
	bool Interval::OverlapOnAxis(const Triangle& tri, const Aabb& aabb, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(aabb, axis);
		const auto [bMin, bMax] = Get(tri, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	/**
	 * @brief Complete SAT-based intersection test between triangle and AABB
	 * @param tri Triangle
	 * @param aabb Axis-Aligned Bounding Box
	 * @return True if the triangle and AABB intersect
	 *
	 * Tests 13 potential separating axes:
	 * - 3 AABB face normals (world axes)
	 * - 1 triangle face normal
	 * - 9 cross products between triangle edges and AABB edge directions
	 */
	bool Interval::TriangleAabb(const Triangle& tri, const Aabb& aabb)
	{
		// Calculate triangle edge vectors
		const Vector3 f0 = tri.b - tri.a;  // Edge AB
		const Vector3 f1 = tri.c - tri.b;  // Edge BC
		const Vector3 f2 = tri.a - tri.c;  // Edge CA

		// AABB edge directions (world axes)
		const Vector3 u0 = { 1.f, 0.f, 0.f };
		const Vector3 u1 = { 0.f, 1.f, 0.f };
		const Vector3 u2 = { 0.f, 0.f, 1.f };

		Vector3 test[13] =
		{
			u0,                         // AABB face normal X
			u1,                         // AABB face normal Y
			u2,                         // AABB face normal Z
			Vector3::Cross(f0, f1),     // Triangle face normal
			Vector3::Cross(u0, f0), Vector3::Cross(u0, f1), Vector3::Cross(u0, f2),  // X-axis x triangle edges
			Vector3::Cross(u1, f0), Vector3::Cross(u1, f1), Vector3::Cross(u1, f2),  // Y-axis x triangle edges
			Vector3::Cross(u2, f0), Vector3::Cross(u2, f1), Vector3::Cross(u2, f2)   // Z-axis x triangle edges
		};

		return std::ranges::all_of(test, [&](const Vector3& axis)
			{
				return OverlapOnAxis(tri, aabb, axis);
			});
	}

	/**
	 * @brief Tests if triangle and OBB projections overlap on a specific axis
	 */
	bool Interval::OverlapOnAxis(const Triangle& tri, const Obb& obb, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(obb, axis);
		const auto [bMin, bMax] = Get(tri, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	/**
	 * @brief Complete SAT-based intersection test between triangle and OBB
	 * Similar to TriangleAabb but uses OBB's local axes instead of world axes
	 */
	bool Interval::TriangleObb(const Triangle& tri, const Obb& obb)
	{
		// Calculate triangle edge vectors
		const Vector3 f0 = tri.b - tri.a;
		const Vector3 f1 = tri.c - tri.b;
		const Vector3 f2 = tri.a - tri.c;

		// OBB edge directions (local axes)
		const Vector3 u0 = obb.orientation.GetColumn(0);
		const Vector3 u1 = obb.orientation.GetColumn(1);
		const Vector3 u2 = obb.orientation.GetColumn(2);

		Vector3 test[13] =
		{
			u0,                         // OBB face normal X
			u1,                         // OBB face normal Y
			u2,                         // OBB face normal Z
			Vector3::Cross(f0, f1),     // Triangle face normal
			Vector3::Cross(u0, f0), Vector3::Cross(u0, f1), Vector3::Cross(u0, f2),
			Vector3::Cross(u1, f0), Vector3::Cross(u1, f1), Vector3::Cross(u1, f2),
			Vector3::Cross(u2, f0), Vector3::Cross(u2, f1), Vector3::Cross(u2, f2)
		};

		return std::ranges::all_of(test, [&](const Vector3& axis)
			{
				return OverlapOnAxis(tri, obb, axis);
			});
	}

	/**
	 * @brief Tests if two triangle projections overlap on a specific axis
	 */
	bool Interval::OverlapOnAxis(const Triangle& t1, const Triangle& t2, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(t1, axis);
		const auto [bMin, bMax] = Get(t2, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	/**
	 * @brief Complete SAT-based intersection test between two triangles
	 * @param t1 First triangle
	 * @param t2 Second triangle
	 * @return True if the triangles intersect
	 *
	 * Tests 11 potential separating axes:
	 * - 2 triangle face normals
	 * - 9 cross products between edge pairs from both triangle
	 */
	bool Interval::TriangleTriangle(const Triangle& t1, const Triangle& t2)
	{
		Vector3 test[] =
		{
			// Triangle face normals
			CrossEdge(t1.a, t1.b, t1.b, t1.c),  // Triangle 1 normal
			CrossEdge(t2.a, t2.b, t2.b, t2.c),  // Triangle 2 normal

			// Cross products of all edge pairs between triangles
			CrossEdge(t2.a, t2.b, t1.a, t1.b),  // T2.AB x T1.AB
			CrossEdge(t2.a, t2.b, t1.b, t1.c),  // T2.AB x T1.BC
			CrossEdge(t2.a, t2.b, t1.c, t1.a),  // T2.AB x T1.CA
			CrossEdge(t2.b, t2.c, t1.a, t1.b),  // T2.BC x T1.AB
			CrossEdge(t2.b, t2.c, t1.b, t1.c),  // T2.BC x T1.BC
			CrossEdge(t2.b, t2.c, t1.c, t1.a),  // T2.BC x T1.CA
			CrossEdge(t2.c, t2.a, t1.a, t1.b),  // T2.CA x T1.AB
			CrossEdge(t2.c, t2.a, t1.b, t1.c),  // T2.CA x T1.BC
			CrossEdge(t2.c, t2.a, t1.c, t1.a)   // T2.CA x T1.CA
		};

		return std::ranges::all_of(test, [&](const Vector3& axis)
			{
				// Skip degenerate axes (parallel edges produce zero cross product)
				return OverlapOnAxis(t1, t2, axis) || MathF::IsNearZero(axis.MagnitudeSqr());
			});
	}

	/**
	 * @brief Utility function to compute cross product of two edges with fallback handling
	 * @param a Start point of first edge
	 * @param b End point of first edge
	 * @param c Start point of second edge
	 * @param d End point of second edge
	 * @return Cross product of edges (a-b) x (c-d), with fallback for parallel edges
	 *
	 * Handles degenerate cases where edges are parallel by computing an alternative axis
	 */
	Vector3 Interval::CrossEdge(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
	{
		const Vector3 ab = a - b;  // First edge vector
		const Vector3 cd = c - d;  // Second edge vector
		Vector3 result = Vector3::Cross(ab, cd);

		// If edges are not parallel, return their cross product
		if (!MathF::IsNearZero(result.MagnitudeSqr()))
		{
			return result;
		}

		// Fallback for parallel edges: create perpendicular axis
		const Vector3 axis = Vector3::Cross(ab, c - a);
		result = Vector3::Cross(ab, axis);

		if (!MathF::IsNearZero(result.MagnitudeSqr()))
		{
			return result;
		}

		// Return zero vector for completely degenerate cases
		return { };
	}
}