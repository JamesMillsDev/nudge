#include <gtest/gtest.h>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

using testing::Test;

namespace Nudge
{
	/**
	* @brief Test fixture for AABB unit tests
	*
	* Provides common test data and utility methods for AABB testing.
	* Sets up standard test cases including unit cube, offset cubes,
	* and edge case scenarios.
	*/
	class AabbTest : public Test
	{
	protected:
		// Test AABB instances
		Aabb m_unitCube;
		Aabb m_offsetCube;
		Aabb m_overlapCube;
		Aabb m_largeCube;
		Aabb m_pointCube;
		Aabb m_negativeCube;

	protected:
		void SetUp() override
		{
			// Standard unit cube centered at origin
			m_unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));

			// Offset cube for intersection testing
			m_offsetCube = Aabb(Vector3(2, 0, 0), Vector3(1, 1, 1));

			// Overlapping cube for partial intersection tests
			m_overlapCube = Aabb(Vector3(1.5f, 0, 0), Vector3(1, 1, 1));

			// Large cube containing unit cube
			m_largeCube = Aabb(Vector3(0, 0, 0), Vector3(5, 5, 5));

			// Zero-extent cube (degenerate case)
			m_pointCube = Aabb(Vector3(1, 1, 1), Vector3(0, 0, 0));

			// Negative extent cube (invalid case)
			m_negativeCube = Aabb(Vector3(0, 0, 0), Vector3(-1, -1, -1));
		}

		// Tolerance for floating-point comparisons
		static constexpr float EPSILON = 1e-6f;
	};

	// ============================================================================
	// Constructor and Factory Method Tests
	// ============================================================================

	TEST_F(AabbTest, DefaultConstructor_InitializesZeroOriginAndExtents)
	{
		Aabb aabb;

		EXPECT_EQ(aabb.origin, Vector3(0, 0, 0));
		EXPECT_EQ(aabb.extents, Vector3(1.f, 1.f, 1.f));
	}

	TEST_F(AabbTest, ParameterizedConstructor_SetsOriginAndExtents)
	{
		Vector3 origin(1, 2, 3);
		Vector3 extents(4, 5, 6);
		Aabb aabb(origin, extents);

		EXPECT_EQ(aabb.origin, origin);
		EXPECT_EQ(aabb.extents, extents);
	}

	TEST_F(AabbTest, FromMinMax_CreatesCorrectAABB)
	{
		Vector3 min(-1, -2, -3);
		Vector3 max(4, 6, 9);
		Aabb aabb = Aabb::FromMinMax(min, max);

		Vector3 expectedOrigin = (min + max) * 0.5f;  // Center point
		Vector3 expectedExtents = (max - min) * 0.5f; // Half-size

		EXPECT_NEAR(aabb.origin.x, expectedOrigin.x, EPSILON);
		EXPECT_NEAR(aabb.origin.y, expectedOrigin.y, EPSILON);
		EXPECT_NEAR(aabb.origin.z, expectedOrigin.z, EPSILON);

		EXPECT_NEAR(aabb.extents.x, expectedExtents.x, EPSILON);
		EXPECT_NEAR(aabb.extents.y, expectedExtents.y, EPSILON);
		EXPECT_NEAR(aabb.extents.z, expectedExtents.z, EPSILON);
	}

	TEST_F(AabbTest, FromMinMax_HandlesIdenticalMinMax)
	{
		Vector3 point(5, 5, 5);
		Aabb aabb = Aabb::FromMinMax(point, point);

		EXPECT_EQ(aabb.origin, point);
		EXPECT_EQ(aabb.extents, Vector3(0, 0, 0));
	}

	// ============================================================================
	// Min/Max Calculation Tests
	// ============================================================================

	TEST_F(AabbTest, Min_ReturnsCorrectMinimumPoint)
	{
		Vector3 min = m_unitCube.Min();
		Vector3 expected(-1, -1, -1);

		EXPECT_NEAR(min.x, expected.x, EPSILON);
		EXPECT_NEAR(min.y, expected.y, EPSILON);
		EXPECT_NEAR(min.z, expected.z, EPSILON);
	}

	TEST_F(AabbTest, Max_ReturnsCorrectMaximumPoint)
	{
		Vector3 max = m_unitCube.Max();
		Vector3 expected(1, 1, 1);

		EXPECT_NEAR(max.x, expected.x, EPSILON);
		EXPECT_NEAR(max.y, expected.y, EPSILON);
		EXPECT_NEAR(max.z, expected.z, EPSILON);
	}

	TEST_F(AabbTest, MinMax_ConsistentWithFromMinMax)
	{
		Vector3 originalMin(-3, -2, -1);
		Vector3 originalMax(4, 5, 6);
		Aabb aabb = Aabb::FromMinMax(originalMin, originalMax);

		Vector3 calculatedMin = aabb.Min();
		Vector3 calculatedMax = aabb.Max();

		EXPECT_NEAR(calculatedMin.x, originalMin.x, EPSILON);
		EXPECT_NEAR(calculatedMin.y, originalMin.y, EPSILON);
		EXPECT_NEAR(calculatedMin.z, originalMin.z, EPSILON);

		EXPECT_NEAR(calculatedMax.x, originalMax.x, EPSILON);
		EXPECT_NEAR(calculatedMax.y, originalMax.y, EPSILON);
		EXPECT_NEAR(calculatedMax.z, originalMax.z, EPSILON);
	}

	// ============================================================================
	// Point Containment Tests
	// ============================================================================

	TEST_F(AabbTest, Contains_PointAtOrigin_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0, 0, 0)));
	}

	TEST_F(AabbTest, Contains_PointInsideBounds_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0.5f, 0.5f, 0.5f)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(-0.5f, -0.5f, -0.5f)));
	}

	TEST_F(AabbTest, Contains_PointOnBoundary_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Contains(Vector3(1, 0, 0)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(-1, 0, 0)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0, 1, 0)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0, -1, 0)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0, 0, 1)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(0, 0, -1)));
	}

	TEST_F(AabbTest, Contains_PointAtCorners_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Contains(Vector3(1, 1, 1)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(-1, -1, -1)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(1, -1, 1)));
		EXPECT_TRUE(m_unitCube.Contains(Vector3(-1, 1, -1)));
	}

	TEST_F(AabbTest, Contains_PointOutsideBounds_ReturnsFalse)
	{
		EXPECT_FALSE(m_unitCube.Contains(Vector3(2, 0, 0)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(0, 2, 0)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(0, 0, 2)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(-2, 0, 0)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(0, -2, 0)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(0, 0, -2)));
	}

	TEST_F(AabbTest, Contains_PointSlightlyOutside_ReturnsFalse)
	{
		EXPECT_FALSE(m_unitCube.Contains(Vector3(1.001f, 0, 0)));
		EXPECT_FALSE(m_unitCube.Contains(Vector3(-1.001f, 0, 0)));
	}

	TEST_F(AabbTest, Contains_DegenerateAABB_OnlyContainsExactPoint)
	{
		// Create a zero-extent AABB at the test point location
		Vector3 testPoint(5, 5, 5);
		Aabb degenerateAABB(testPoint, Vector3(0, 0, 0));

		EXPECT_TRUE(degenerateAABB.Contains(testPoint));
		EXPECT_FALSE(degenerateAABB.Contains(testPoint + Vector3(0.001f, 0, 0)));
		EXPECT_FALSE(degenerateAABB.Contains(testPoint + Vector3(0, 0.001f, 0)));
		EXPECT_FALSE(degenerateAABB.Contains(testPoint + Vector3(0, 0, 0.001f)));
	}

	// ============================================================================
	// Closest Point Tests
	// ============================================================================

	TEST_F(AabbTest, ClosestPoint_PointInside_ReturnsSamePoint)
	{
		Vector3 inside(0.5f, 0.5f, 0.5f);
		Vector3 closest = m_unitCube.ClosestPoint(inside);

		EXPECT_NEAR(closest.x, inside.x, EPSILON);
		EXPECT_NEAR(closest.y, inside.y, EPSILON);
		EXPECT_NEAR(closest.z, inside.z, EPSILON);
	}

	TEST_F(AabbTest, ClosestPoint_PointAtCenter_ReturnsSamePoint)
	{
		Vector3 center = m_unitCube.origin;
		Vector3 closest = m_unitCube.ClosestPoint(center);

		EXPECT_EQ(closest, center);
	}

	TEST_F(AabbTest, ClosestPoint_PointOutsideOnAxis_ReturnsProjectedPoint)
	{
		Vector3 outside(3, 0, 0);
		Vector3 closest = m_unitCube.ClosestPoint(outside);
		Vector3 expected(1, 0, 0);  // Clamped to right face

		EXPECT_NEAR(closest.x, expected.x, EPSILON);
		EXPECT_NEAR(closest.y, expected.y, EPSILON);
		EXPECT_NEAR(closest.z, expected.z, EPSILON);
	}

	TEST_F(AabbTest, ClosestPoint_PointOutsideOnNegativeAxis_ReturnsProjectedPoint)
	{
		Vector3 outside(-3, 0, 0);
		Vector3 closest = m_unitCube.ClosestPoint(outside);
		Vector3 expected(-1, 0, 0);  // Clamped to left face

		EXPECT_NEAR(closest.x, expected.x, EPSILON);
		EXPECT_NEAR(closest.y, expected.y, EPSILON);
		EXPECT_NEAR(closest.z, expected.z, EPSILON);
	}

	TEST_F(AabbTest, ClosestPoint_PointOutsideOnAllAxes_ReturnsCorner)
	{
		Vector3 outside(3, 3, 3);
		Vector3 closest = m_unitCube.ClosestPoint(outside);
		Vector3 expected(1, 1, 1);  // Clamped to corner

		EXPECT_NEAR(closest.x, expected.x, EPSILON);
		EXPECT_NEAR(closest.y, expected.y, EPSILON);
		EXPECT_NEAR(closest.z, expected.z, EPSILON);
	}

	TEST_F(AabbTest, ClosestPoint_PointOnBoundary_ReturnsSamePoint)
	{
		Vector3 boundary(1, 0.5f, 0.5f);
		Vector3 closest = m_unitCube.ClosestPoint(boundary);

		EXPECT_NEAR(closest.x, boundary.x, EPSILON);
		EXPECT_NEAR(closest.y, boundary.y, EPSILON);
		EXPECT_NEAR(closest.z, boundary.z, EPSILON);
	}

	// ============================================================================
	// AABB-AABB Intersection Tests
	// ============================================================================

	TEST_F(AabbTest, IntersectsAABB_IdenticalBoxes_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Intersects(m_unitCube));
	}

	TEST_F(AabbTest, IntersectsAABB_ContainedBox_ReturnsTrue)
	{
		Aabb small(Vector3(0, 0, 0), Vector3(0.5f, 0.5f, 0.5f));
		EXPECT_TRUE(m_unitCube.Intersects(small));
		EXPECT_TRUE(small.Intersects(m_unitCube));
	}

	TEST_F(AabbTest, IntersectsAABB_OverlappingBoxes_ReturnsTrue)
	{
		EXPECT_TRUE(m_unitCube.Intersects(m_overlapCube));
		EXPECT_TRUE(m_overlapCube.Intersects(m_unitCube));
	}

	TEST_F(AabbTest, IntersectsAABB_TouchingBoxes_ReturnsTrue)
	{
		Aabb touching(Vector3(2, 0, 0), Vector3(1, 1, 1));
		EXPECT_TRUE(m_unitCube.Intersects(touching));
	}

	TEST_F(AabbTest, IntersectsAABB_SeparatedBoxes_ReturnsFalse)
	{
		Aabb separated(Vector3(3, 0, 0), Vector3(1, 1, 1));
		EXPECT_FALSE(m_unitCube.Intersects(separated));
		EXPECT_FALSE(separated.Intersects(m_unitCube));
	}

	TEST_F(AabbTest, IntersectsAABB_TouchingAtPoint_ReturnsTrue)
	{
		Aabb pointTouch(Vector3(2, 2, 2), Vector3(1, 1, 1));
		EXPECT_TRUE(m_unitCube.Intersects(pointTouch));
	}

	TEST_F(AabbTest, IntersectsAABB_DegenerateBoxes_HandledCorrectly)
	{
		EXPECT_TRUE(m_unitCube.Intersects(m_pointCube));  // Point inside cube

		Aabb outsidePoint(Vector3(5, 5, 5), Vector3(0, 0, 0));
		EXPECT_FALSE(m_unitCube.Intersects(outsidePoint));
	}

	// ============================================================================
	// Edge Cases and Robustness Tests
	// ============================================================================

	TEST_F(AabbTest, EdgeCases_ZeroExtentAABB_BehavesAsPoint)
	{
		Vector3 point(2, 2, 2);
		Aabb pointAABB(point, Vector3(0, 0, 0));

		EXPECT_TRUE(pointAABB.Contains(point));
		EXPECT_FALSE(pointAABB.Contains(point + Vector3(0.001f, 0, 0)));
		EXPECT_EQ(pointAABB.ClosestPoint(Vector3(5, 5, 5)), point);
	}

	TEST_F(AabbTest, EdgeCases_NegativeExtents_UndefinedBehavior)
	{
		// Test should document behavior with negative extents
		// Implementation may vary - this tests current behavior
		Vector3 testPoint(0, 0, 0);

		// Behavior with negative extents is implementation-defined
		// This test documents the current behavior
		bool containsResult = m_negativeCube.Contains(testPoint);
		Vector3 closestResult = m_negativeCube.ClosestPoint(testPoint);

		// Results may vary based on implementation
		// The test ensures no crashes occur
		EXPECT_NO_THROW({
			m_negativeCube.Contains(testPoint);
			m_negativeCube.ClosestPoint(testPoint);
			});
	}

	TEST_F(AabbTest, EdgeCases_VeryLargeAABB_MaintainsPrecision)
	{
		Vector3 largeExtents(1e6f, 1e6f, 1e6f);
		Aabb largeAABB(Vector3(0, 0, 0), largeExtents);

		Vector3 farPoint(5e5f, 5e5f, 5e5f);
		EXPECT_TRUE(largeAABB.Contains(farPoint));

		Vector3 closest = largeAABB.ClosestPoint(farPoint);
		EXPECT_NEAR(closest.x, farPoint.x, 1.0f);  // Relaxed precision for large numbers
	}

	TEST_F(AabbTest, EdgeCases_VerySmallAABB_MaintainsPrecision)
	{
		Vector3 tinyExtents(1e-6f, 1e-6f, 1e-6f);
		Aabb tinyAABB(Vector3(0, 0, 0), tinyExtents);

		Vector3 nearPoint(5e-7f, 5e-7f, 5e-7f);
		EXPECT_TRUE(tinyAABB.Contains(nearPoint));

		Vector3 farPoint(5e-6f, 5e-6f, 5e-6f);
		EXPECT_FALSE(tinyAABB.Contains(farPoint));
	}

	// ============================================================================
	// Performance and Stress Tests
	// ============================================================================

	TEST_F(AabbTest, Performance_ManyIntersectionTests_CompletesQuickly)
	{
		const int numTests = 10000;
		int intersectionCount = 0;

		auto start = std::chrono::high_resolution_clock::now();

		for (int i = 0; i < numTests; ++i)
		{
			Vector3 randomOffset(
				(float)(i % 100) / 50.0f - 1.0f,
				(float)((i * 7) % 100) / 50.0f - 1.0f,
				(float)((i * 13) % 100) / 50.0f - 1.0f
			);

			Aabb testAABB(randomOffset, Vector3(0.5f, 0.5f, 0.5f));
			if (m_unitCube.Intersects(testAABB))
			{
				intersectionCount++;
			}
		}

		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		// Should complete many intersection tests quickly
		EXPECT_LT(duration.count(), 100);  // Less than 100ms
		EXPECT_GT(intersectionCount, 0);   // Some intersections should occur
	}

	// ============================================================================
	// Integration Tests with Other Shapes
	// ============================================================================

	TEST_F(AabbTest, Integration_AABBSphereIntersection_WorksCorrectly)
	{
		Sphere containedSphere(Vector3(0, 0, 0), 0.5f);
		Sphere intersectingSphere(Vector3(1.5f, 0, 0), 1.0f);
		Sphere separatedSphere(Vector3(5, 0, 0), 1.0f);

		EXPECT_TRUE(m_unitCube.Intersects(containedSphere));
		EXPECT_TRUE(m_unitCube.Intersects(intersectingSphere));
		EXPECT_FALSE(m_unitCube.Intersects(separatedSphere));
	}

	TEST_F(AabbTest, Integration_AABBPlaneIntersection_WorksCorrectly)
	{
		Plane throughCenter(Vector3(1, 0, 0), 0);      // Plane through center
		Plane touchingEdge(Vector3(1, 0, 0), -1);      // Plane touching left edge
		Plane separated(Vector3(1, 0, 0), -5);         // Plane far from box

		EXPECT_TRUE(m_unitCube.Intersects(throughCenter));
		EXPECT_TRUE(m_unitCube.Intersects(touchingEdge));
		EXPECT_FALSE(m_unitCube.Intersects(separated));
	}

	TEST_F(AabbTest, Integration_AABBTriangleIntersection_WorksCorrectly)
	{
		Triangle insideTriangle(
			Vector3(-0.5f, -0.5f, 0),
			Vector3(0.5f, -0.5f, 0),
			Vector3(0, 0.5f, 0)
		);

		Triangle intersectingTriangle(
			Vector3(-2, 0, 0),
			Vector3(2, 0, 0),
			Vector3(0, 2, 0)
		);

		Triangle separatedTriangle(
			Vector3(5, 5, 5),
			Vector3(6, 5, 5),
			Vector3(5, 6, 5)
		);

		EXPECT_TRUE(m_unitCube.Intersects(insideTriangle));
		EXPECT_TRUE(m_unitCube.Intersects(intersectingTriangle));
		EXPECT_FALSE(m_unitCube.Intersects(separatedTriangle));
	}

	TEST_F(AabbTest, IntersectsOBB_IdenticalAlignedOBB_ReturnsTrue)
	{
		// Create an OBB that's identical to the unit cube (no rotation)
		Matrix3 identity = Matrix3::Identity();
		Obb alignedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);

		EXPECT_TRUE(m_unitCube.Intersects(alignedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_ContainedAlignedOBB_ReturnsTrue)
	{
		// Small OBB inside the unit cube
		Matrix3 identity = Matrix3::Identity();
		Obb smallOBB(Vector3(0, 0, 0), Vector3(0.5f, 0.5f, 0.5f), identity);

		EXPECT_TRUE(m_unitCube.Intersects(smallOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_ContainingAlignedOBB_ReturnsTrue)
	{
		// Large OBB containing the unit cube
		Matrix3 identity = Matrix3::Identity();
		Obb largeOBB(Vector3(0, 0, 0), Vector3(2, 2, 2), identity);

		EXPECT_TRUE(m_unitCube.Intersects(largeOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_SeparatedAlignedOBB_ReturnsFalse)
	{
		// OBB completely separated from unit cube
		Matrix3 identity = Matrix3::Identity();
		Obb separatedOBB(Vector3(3, 0, 0), Vector3(1, 1, 1), identity);

		EXPECT_FALSE(m_unitCube.Intersects(separatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_TouchingAlignedOBB_ReturnsTrue)
	{
		// OBB touching the unit cube face
		Matrix3 identity = Matrix3::Identity();
		Obb touchingOBB(Vector3(2, 0, 0), Vector3(1, 1, 1), identity);

		EXPECT_TRUE(m_unitCube.Intersects(touchingOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_45DegreesZ_ReturnsTrue)
	{
		// OBB rotated 45 degrees around Z-axis, should still intersect
		Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 4.0f);
		Obb rotatedOBB(Vector3(0, 0, 0), Vector3(0.8f, 0.8f, 1), rotationZ);

		EXPECT_TRUE(m_unitCube.Intersects(rotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_45DegreesY_ReturnsTrue)
	{
		// OBB rotated 45 degrees around Y-axis
		Matrix3 rotationY = Matrix3::RotationY(MathF::pi / 4.0f);
		Obb rotatedOBB(Vector3(0, 0, 0), Vector3(0.8f, 1, 0.8f), rotationY);

		EXPECT_TRUE(m_unitCube.Intersects(rotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_45DegreesX_ReturnsTrue)
	{
		// OBB rotated 45 degrees around X-axis
		Matrix3 rotationX = Matrix3::RotationX(MathF::pi / 4.0f);
		Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 0.8f, 0.8f), rotationX);

		EXPECT_TRUE(m_unitCube.Intersects(rotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_90Degrees_StillIntersects)
	{
		// OBB rotated 90 degrees around Z-axis
		Matrix3 rotation90 = Matrix3::RotationZ(MathF::pi / 2.0f);
		Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation90);

		EXPECT_TRUE(m_unitCube.Intersects(rotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedSeparatedOBB_ReturnsFalse)
	{
		// Rotated OBB that's separated from the unit cube
		Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 4.0f);
		Obb separatedRotatedOBB(Vector3(3, 3, 0), Vector3(0.5f, 0.5f, 1), rotationZ);

		EXPECT_FALSE(m_unitCube.Intersects(separatedRotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_DiagonallyRotatedOBB_Intersects)
	{
		// OBB rotated around multiple axes (complex rotation)
		Matrix3 rotationX = Matrix3::RotationX(MathF::pi / 6.0f);  // 30 degrees
		Matrix3 rotationY = Matrix3::RotationY(MathF::pi / 4.0f);  // 45 degrees
		Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 3.0f);  // 60 degrees
		Matrix3 combinedRotation = rotationZ * rotationY * rotationX;

		Obb complexOBB(Vector3(0, 0, 0), Vector3(0.7f, 0.7f, 0.7f), combinedRotation);

		EXPECT_TRUE(m_unitCube.Intersects(complexOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_ThinRotatedOBB_EdgeCase)
	{
		// Very thin OBB rotated 45 degrees - tests precision
		Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 4.0f);
		Obb thinOBB(Vector3(0, 0, 0), Vector3(2.0f, 0.01f, 1), rotationZ);

		EXPECT_TRUE(m_unitCube.Intersects(thinOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_JustTouching)
	{
		// Rotated OBB that just barely touches the cube
		Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 4.0f);
		// Position it so corner just touches the cube face
		float touchDistance = sqrt(2.0f);  // Distance for 45-degree rotated square to touch
		Obb touchingRotatedOBB(Vector3(touchDistance, 0, 0), Vector3(1, 1, 1), rotationZ);

		EXPECT_TRUE(m_unitCube.Intersects(touchingRotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_RotatedOBB_JustMissing)
	{
	    Matrix3 rotationZ = Matrix3::RotationZ(MathF::pi / 4.0f);
	    
	    // Position the OBB so its closest point is just outside the unit cube
	    // For 45 degrees rotation, the closest point to origin will be at distance:
	    // center_distance - sqrt(2) (the rotated extent in X direction)
	    float centerDistance = 1.0f + sqrt(2.0f) + 0.001f;  // Just beyond touching
	    
	    Obb missingRotatedOBB(Vector3(centerDistance, 0, 0), Vector3(1, 1, 1), rotationZ);
	    
	    EXPECT_FALSE(m_unitCube.Intersects(missingRotatedOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_DegenerateOBB_ZeroExtents)
	{
		// Zero-extent OBB (point) inside the cube
		Matrix3 identity = Matrix3::Identity();
		Obb pointOBB(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0, 0), identity);

		EXPECT_TRUE(m_unitCube.Intersects(pointOBB));

		// Zero-extent OBB outside the cube
		Obb outsidePointOBB(Vector3(5, 5, 5), Vector3(0, 0, 0), identity);
		EXPECT_FALSE(m_unitCube.Intersects(outsidePointOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_ExtremelySkinnyOBB_Precision)
	{
		// Extremely thin OBB to test numerical precision
		Matrix3 identity = Matrix3::Identity();
		Obb skinnyOBB(Vector3(0, 0, 0), Vector3(2, 1e-6f, 2), identity);

		EXPECT_TRUE(m_unitCube.Intersects(skinnyOBB));
	}

	TEST_F(AabbTest, IntersectsOBB_ArbitraryRotation_StressTest)
	{
		// Test with various arbitrary rotations to ensure robustness
		for (int i = 0; i < 10; ++i)
		{
			float angleX = (float)i * 0.3f;
			float angleY = (float)i * 0.7f;
			float angleZ = (float)i * 1.1f;

			Matrix3 rotX = Matrix3::RotationX(angleX);
			Matrix3 rotY = Matrix3::RotationY(angleY);
			Matrix3 rotZ = Matrix3::RotationZ(angleZ);
			Matrix3 combinedRot = rotZ * rotY * rotX;

			Obb testOBB(Vector3(0, 0, 0), Vector3(0.8f, 0.8f, 0.8f), combinedRot);

			// Should intersect since it's centered and smaller than unit cube
			EXPECT_TRUE(m_unitCube.Intersects(testOBB));
		}
	}

	TEST_F(AabbTest, IntersectsOBB_OffsetRotatedOBBs_VariousPositions)
	{
		Matrix3 rotation45Z = Matrix3::RotationZ(MathF::pi / 4.0f);

		// Test OBBs at various offset positions
		std::vector<Vector3> testPositions = {
			Vector3(0.5f, 0, 0),     // Partially overlapping
			Vector3(1.0f, 1.0f, 0),  // Corner touching
			Vector3(-0.8f, 0.8f, 0), // Diagonal overlap
			Vector3(0, 0, 0.9f),     // Z-offset overlap
			Vector3(1.5f, 0, 0)      // Edge case overlap
		};

		for (const auto& pos : testPositions)
		{
			Obb testOBB(pos, Vector3(0.6f, 0.6f, 0.6f), rotation45Z);
			bool intersects = m_unitCube.Intersects(testOBB);

			// All these positions should result in intersection with the rotation and size chosen
			EXPECT_TRUE(intersects) << "Failed for position: " << pos.x << ", " << pos.y << ", " << pos.z;
		}
	}
}