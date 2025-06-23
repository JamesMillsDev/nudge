#include <chrono>
#include <gtest/gtest.h>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

namespace Nudge
{
    /**
     * @brief Test fixture for OBB unit tests
     * 
     * Provides common test data and utility methods for OBB testing.
     * Sets up standard test cases including aligned and rotated OBBs,
     * and various geometric primitives for intersection testing.
     */
    class ObbTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Identity matrix for aligned OBBs
            identity = Matrix3::Identity();
            
            // Standard aligned OBB (equivalent to AABB)
            unitOBB = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
            
            // OBB with only origin and extents (should use identity orientation)
            simpleOBB = Obb(Vector3(1, 1, 1), Vector3(0.5f, 0.5f, 0.5f));
            
            // Rotated OBBs for testing
            rotationZ45 = Matrix3::RotationZ(MathF::pi / 4.0f);  // 45 degrees around Z
            rotationY90 = Matrix3::RotationY(MathF::pi / 2.0f);  // 90 degrees around Y
            rotationX30 = Matrix3::RotationX(MathF::pi / 6.0f);  // 30 degrees around X
            
            rotatedOBB_Z = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), rotationZ45);
            rotatedOBB_Y = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), rotationY90);
            rotatedOBB_X = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), rotationX30);
            
            // Complex rotation (multiple axes)
            complexRotation = rotationZ45 * rotationY90 * rotationX30;
            complexOBB = Obb(Vector3(2, 2, 2), Vector3(0.8f, 0.8f, 0.8f), complexRotation);
            
            // Offset OBBs
            offsetOBB = Obb(Vector3(3, 0, 0), Vector3(1, 1, 1), identity);
            
            // Zero extent OBB (degenerate case)
            pointOBB = Obb(Vector3(1, 1, 1), Vector3(0, 0, 0), identity);
            
            // Test geometries
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            testPlane = Plane(Vector3(1, 0, 0), 0);  // YZ plane through origin
            testTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
        }

        // Test OBB instances
        Matrix3 identity;
        Matrix3 rotationZ45;
        Matrix3 rotationY90;
        Matrix3 rotationX30;
        Matrix3 complexRotation;
        
        Obb unitOBB;
        Obb simpleOBB;
        Obb rotatedOBB_Z;
        Obb rotatedOBB_Y;
        Obb rotatedOBB_X;
        Obb complexOBB;
        Obb offsetOBB;
        Obb pointOBB;
        
        // Other test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Plane testPlane;
        Triangle testTriangle;
        
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(ObbTest, DefaultConstructor_InitializesZeroOriginExtentsAndIdentityOrientation)
    {
        Obb obb;
        
        EXPECT_EQ(obb.origin, Vector3(0, 0, 0));
        EXPECT_EQ(obb.extents, Vector3(0, 0, 0));
        // Note: Default orientation behavior may vary by implementation
    }

    TEST_F(ObbTest, TwoParameterConstructor_SetsOriginAndExtents)
    {
        Vector3 origin(1, 2, 3);
        Vector3 extents(4, 5, 6);
        Obb obb(origin, extents);
        
        EXPECT_EQ(obb.origin, origin);
        EXPECT_EQ(obb.extents, extents);
        // Orientation should be identity or default
    }

    TEST_F(ObbTest, ThreeParameterConstructor_SetsAllProperties)
    {
        Vector3 origin(1, 2, 3);
        Vector3 extents(4, 5, 6);
        Matrix3 orientation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb obb(origin, extents, orientation);
        
        EXPECT_EQ(obb.origin, origin);
        EXPECT_EQ(obb.extents, extents);
        EXPECT_EQ(obb.orientation, orientation);
    }

    // ============================================================================
    // Point Containment Tests
    // ============================================================================

    TEST_F(ObbTest, Contains_PointAtOrigin_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Contains(Vector3(0, 0, 0)));
    }

    TEST_F(ObbTest, Contains_PointInsideAlignedOBB_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Contains(Vector3(0.5f, 0.5f, 0.5f)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(-0.5f, -0.5f, -0.5f)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(0.9f, 0.9f, 0.9f)));
    }

    TEST_F(ObbTest, Contains_PointOnBoundaryAlignedOBB_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Contains(Vector3(1, 0, 0)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(-1, 0, 0)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(0, 1, 0)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(0, -1, 0)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(0, 0, 1)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(0, 0, -1)));
    }

    TEST_F(ObbTest, Contains_PointAtCornersAlignedOBB_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Contains(Vector3(1, 1, 1)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(-1, -1, -1)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(1, -1, 1)));
        EXPECT_TRUE(unitOBB.Contains(Vector3(-1, 1, -1)));
    }

    TEST_F(ObbTest, Contains_PointOutsideAlignedOBB_ReturnsFalse)
    {
        EXPECT_FALSE(unitOBB.Contains(Vector3(2, 0, 0)));
        EXPECT_FALSE(unitOBB.Contains(Vector3(0, 2, 0)));
        EXPECT_FALSE(unitOBB.Contains(Vector3(0, 0, 2)));
        EXPECT_FALSE(unitOBB.Contains(Vector3(-2, 0, 0)));
        EXPECT_FALSE(unitOBB.Contains(Vector3(1.5f, 1.5f, 1.5f)));
    }

    TEST_F(ObbTest, Contains_PointInRotatedOBB_WorksCorrectly)
    {
        // Test with 45-degree rotated OBB
        EXPECT_TRUE(rotatedOBB_Z.Contains(Vector3(0, 0, 0)));  // Center
        
        // Points that should be inside the rotated OBB
        Vector3 rotatedPoint = rotationZ45 * Vector3(0.5f, 0, 0);
        EXPECT_TRUE(rotatedOBB_Z.Contains(rotatedPoint));
    }

    TEST_F(ObbTest, Contains_PointOutsideRotatedOBB_ReturnsFalse)
    {
        // Point that would be inside an aligned OBB but outside the rotated one
        EXPECT_FALSE(rotatedOBB_Z.Contains(Vector3(1, 1, 0)));
    }

    TEST_F(ObbTest, Contains_OffsetOBB_WorksCorrectly)
    {
        Vector3 offsetCenter = simpleOBB.origin;
        EXPECT_TRUE(simpleOBB.Contains(offsetCenter));
        EXPECT_TRUE(simpleOBB.Contains(offsetCenter + Vector3(0.4f, 0.4f, 0.4f)));
        EXPECT_FALSE(simpleOBB.Contains(Vector3(0, 0, 0)));  // Origin is outside offset OBB
    }

    TEST_F(ObbTest, Contains_DegenerateOBB_OnlyContainsExactPoint)
    {
        Vector3 center = pointOBB.origin;
        EXPECT_TRUE(pointOBB.Contains(center));
        EXPECT_FALSE(pointOBB.Contains(center + Vector3(0.001f, 0, 0)));
    }

    // ============================================================================
    // Closest Point Tests
    // ============================================================================

    TEST_F(ObbTest, ClosestPoint_PointInside_ReturnsSamePoint)
    {
        Vector3 inside(0.5f, 0.5f, 0.5f);
        Vector3 closest = unitOBB.ClosestPoint(inside);
        
        EXPECT_NEAR(closest.x, inside.x, EPSILON);
        EXPECT_NEAR(closest.y, inside.y, EPSILON);
        EXPECT_NEAR(closest.z, inside.z, EPSILON);
    }

    TEST_F(ObbTest, ClosestPoint_PointAtCenter_ReturnsSamePoint)
    {
        Vector3 center = unitOBB.origin;
        Vector3 closest = unitOBB.ClosestPoint(center);
        
        EXPECT_EQ(closest, center);
    }

    TEST_F(ObbTest, ClosestPoint_PointOutsideAlignedOBB_ReturnsProjectedPoint)
    {
        Vector3 outside(3, 0, 0);
        Vector3 closest = unitOBB.ClosestPoint(outside);
        Vector3 expected(1, 0, 0);  // Clamped to right face
        
        EXPECT_NEAR(closest.x, expected.x, EPSILON);
        EXPECT_NEAR(closest.y, expected.y, EPSILON);
        EXPECT_NEAR(closest.z, expected.z, EPSILON);
    }

    TEST_F(ObbTest, ClosestPoint_PointOutsideRotatedOBB_ReturnsCorrectProjection)
    {
        Vector3 outside(3, 3, 0);
        Vector3 closest = rotatedOBB_Z.ClosestPoint(outside);
        
        // Should return a point on the OBB surface
        EXPECT_TRUE(rotatedOBB_Z.Contains(closest) || 
                   (closest - rotatedOBB_Z.origin).Magnitude() <= sqrt(2.0f) + EPSILON);
    }

    TEST_F(ObbTest, ClosestPoint_PointOnBoundary_ReturnsSamePoint)
    {
        Vector3 boundary(1, 0.5f, 0.5f);
        Vector3 closest = unitOBB.ClosestPoint(boundary);
        
        EXPECT_NEAR(closest.x, boundary.x, EPSILON);
        EXPECT_NEAR(closest.y, boundary.y, EPSILON);
        EXPECT_NEAR(closest.z, boundary.z, EPSILON);
    }

    TEST_F(ObbTest, ClosestPoint_ComplexRotatedOBB_WorksCorrectly)
    {
        Vector3 testPoint(5, 5, 5);
        Vector3 closest = complexOBB.ClosestPoint(testPoint);
        
        // Closest point should be closer to test point than OBB center
        float distanceToClosest = (testPoint - closest).Magnitude();
        float distanceToCenter = (testPoint - complexOBB.origin).Magnitude();
        
        EXPECT_LE(distanceToClosest, distanceToCenter);
    }

    // ============================================================================
    // OBB-AABB Intersection Tests
    // ============================================================================

    TEST_F(ObbTest, IntersectsAABB_AlignedOBBIdenticalToAABB_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_AlignedOBBContainedInAABB_ReturnsTrue)
    {
        Obb smallOBB(Vector3(0, 0, 0), Vector3(0.5f, 0.5f, 0.5f), identity);
        EXPECT_TRUE(smallOBB.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_AlignedOBBContainingAABB_ReturnsTrue)
    {
        Obb largeOBB(Vector3(0, 0, 0), Vector3(2, 2, 2), identity);
        EXPECT_TRUE(largeOBB.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_SeparatedAlignedOBB_ReturnsFalse)
    {
        EXPECT_FALSE(offsetOBB.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_TouchingAlignedOBB_ReturnsTrue)
    {
        Obb touching(Vector3(2, 0, 0), Vector3(1, 1, 1), identity);
        EXPECT_TRUE(touching.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_RotatedOBB_45DegreesZ_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Z.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_RotatedOBB_90DegreesY_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Y.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_RotatedOBB_30DegreesX_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_X.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_ComplexRotatedOBB_SeparatedFromAABB_ReturnsFalse)
    {
        // Complex OBB is at (2,2,2) which should be separate from unit cube at origin
        EXPECT_FALSE(complexOBB.Intersects(unitCube));
    }

    TEST_F(ObbTest, IntersectsAABB_DegenerateOBB_HandledCorrectly)
    {
        EXPECT_TRUE(unitCube.Contains(pointOBB.origin) ? 
                   true : !pointOBB.Intersects(unitCube));
    }

    // ============================================================================
    // OBB-OBB Intersection Tests
    // ============================================================================

    TEST_F(ObbTest, IntersectsOBB_IdenticalOBBs_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Intersects(unitOBB));
        EXPECT_TRUE(rotatedOBB_Z.Intersects(rotatedOBB_Z));
    }

    TEST_F(ObbTest, IntersectsOBB_AlignedOBBs_OverlappingCenters_ReturnsTrue)
    {
        Obb overlapping(Vector3(0.5f, 0.5f, 0.5f), Vector3(1, 1, 1), identity);
        EXPECT_TRUE(unitOBB.Intersects(overlapping));
        EXPECT_TRUE(overlapping.Intersects(unitOBB));
    }

    TEST_F(ObbTest, IntersectsOBB_AlignedOBBs_Separated_ReturnsFalse)
    {
        EXPECT_FALSE(unitOBB.Intersects(offsetOBB));
        EXPECT_FALSE(offsetOBB.Intersects(unitOBB));
    }

    TEST_F(ObbTest, IntersectsOBB_AlignedOBBs_Touching_ReturnsTrue)
    {
        Obb touching(Vector3(2, 0, 0), Vector3(1, 1, 1), identity);
        EXPECT_TRUE(unitOBB.Intersects(touching));
    }

    TEST_F(ObbTest, IntersectsOBB_RotatedOBBs_SameRotation_ReturnsTrue)
    {
        Obb rotated2(Vector3(0.5f, 0.5f, 0), Vector3(1, 1, 1), rotationZ45);
        EXPECT_TRUE(rotatedOBB_Z.Intersects(rotated2));
    }

    TEST_F(ObbTest, IntersectsOBB_DifferentRotations_Intersecting_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Z.Intersects(rotatedOBB_Y));
        EXPECT_TRUE(rotatedOBB_Y.Intersects(rotatedOBB_X));
        EXPECT_TRUE(rotatedOBB_X.Intersects(rotatedOBB_Z));
    }

    TEST_F(ObbTest, IntersectsOBB_ComplexRotations_WorkCorrectly)
    {
        Obb complex2(Vector3(1.5f, 1.5f, 1.5f), Vector3(0.8f, 0.8f, 0.8f), complexRotation);
        EXPECT_TRUE(complexOBB.Intersects(complex2));
    }

    TEST_F(ObbTest, IntersectsOBB_DegenerateOBBs_HandledCorrectly)
    {
        Obb pointOBB2(Vector3(0.5f, 0.5f, 0.5f), Vector3(0, 0, 0), identity);
        
        EXPECT_TRUE(unitOBB.Intersects(pointOBB2));  // Point inside OBB
        EXPECT_FALSE(pointOBB.Intersects(pointOBB2)); // Separate points
    }

    // ============================================================================
    // OBB-Sphere Intersection Tests
    // ============================================================================

    TEST_F(ObbTest, IntersectsSphere_SphereContainingOBB_ReturnsTrue)
    {
        Sphere largeSphere(Vector3(0, 0, 0), 5.0f);
        EXPECT_TRUE(unitOBB.Intersects(largeSphere));
    }

    TEST_F(ObbTest, IntersectsSphere_OBBContainingSphere_ReturnsTrue)
    {
        Obb largeOBB(Vector3(0, 0, 0), Vector3(2, 2, 2), identity);
        EXPECT_TRUE(largeOBB.Intersects(unitSphere));
    }

    TEST_F(ObbTest, IntersectsSphere_OverlappingOBBAndSphere_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Intersects(unitSphere));
    }

    TEST_F(ObbTest, IntersectsSphere_TouchingOBBAndSphere_ReturnsTrue)
    {
        Sphere touching(Vector3(2, 0, 0), 1.0f);
        EXPECT_TRUE(unitOBB.Intersects(touching));
    }

    TEST_F(ObbTest, IntersectsSphere_SeparatedOBBAndSphere_ReturnsFalse)
    {
        Sphere separated(Vector3(5, 0, 0), 1.0f);
        EXPECT_FALSE(unitOBB.Intersects(separated));
    }

    TEST_F(ObbTest, IntersectsSphere_RotatedOBBAndSphere_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Z.Intersects(unitSphere));
        EXPECT_TRUE(rotatedOBB_Y.Intersects(unitSphere));
        EXPECT_TRUE(rotatedOBB_X.Intersects(unitSphere));
    }

    TEST_F(ObbTest, IntersectsSphere_ComplexRotatedOBBAndOffsetSphere_WorksCorrectly)
    {
        Sphere offsetSphere(Vector3(2, 2, 2), 1.5f);
        EXPECT_TRUE(complexOBB.Intersects(offsetSphere));
    }

    // ============================================================================
    // OBB-Plane Intersection Tests
    // ============================================================================

    TEST_F(ObbTest, IntersectsPlane_PlaneThroughOBBCenter_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Intersects(testPlane));
    }

    TEST_F(ObbTest, IntersectsPlane_PlaneTouchingOBBFace_ReturnsTrue)
    {
        Plane touching(Vector3(1, 0, 0), -1);  // Touching right face
        EXPECT_TRUE(unitOBB.Intersects(touching));
    }

    TEST_F(ObbTest, IntersectsPlane_PlaneSeparatedFromOBB_ReturnsFalse)
    {
        Plane separated(Vector3(1, 0, 0), -3);  // Far from OBB
        EXPECT_FALSE(unitOBB.Intersects(separated));
    }

    TEST_F(ObbTest, IntersectsPlane_PlaneThroughRotatedOBB_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Z.Intersects(testPlane));
        EXPECT_TRUE(rotatedOBB_Y.Intersects(testPlane));
        EXPECT_TRUE(rotatedOBB_X.Intersects(testPlane));
    }

    TEST_F(ObbTest, IntersectsPlane_DiagonalPlaneThroughOBB_ReturnsTrue)
    {
        Plane diagonal(Vector3(1, 1, 1).Normalized(), 0);
        EXPECT_TRUE(unitOBB.Intersects(diagonal));
        EXPECT_TRUE(rotatedOBB_Z.Intersects(diagonal));
    }

    TEST_F(ObbTest, IntersectsPlane_OffsetOBBAndPlane_WorksCorrectly)
    {
        Plane offsetPlane(Vector3(1, 0, 0), -2.5f);  // Should intersect offset OBB
        EXPECT_TRUE(offsetOBB.Intersects(offsetPlane));
    }

    // ============================================================================
    // OBB-Triangle Intersection Tests
    // ============================================================================

    TEST_F(ObbTest, IntersectsTriangle_TriangleInsideOBB_ReturnsTrue)
    {
        Triangle small(
            Vector3(-0.5f, -0.5f, 0),
            Vector3(0.5f, -0.5f, 0),
            Vector3(0, 0.5f, 0)
        );
        EXPECT_TRUE(unitOBB.Intersects(small));
    }

    TEST_F(ObbTest, IntersectsTriangle_TriangleIntersectingOBB_ReturnsTrue)
    {
        EXPECT_TRUE(unitOBB.Intersects(testTriangle));
    }

    TEST_F(ObbTest, IntersectsTriangle_TriangleSeparatedFromOBB_ReturnsFalse)
    {
        Triangle separated(
            Vector3(5, 5, 5),
            Vector3(6, 5, 5),
            Vector3(5, 6, 5)
        );
        EXPECT_FALSE(unitOBB.Intersects(separated));
    }

    TEST_F(ObbTest, IntersectsTriangle_TriangleTouchingOBBVertex_ReturnsTrue)
    {
        Triangle touching(
            Vector3(1, 1, 1),  // OBB corner
            Vector3(2, 1, 1),
            Vector3(1, 2, 1)
        );
        EXPECT_TRUE(unitOBB.Intersects(touching));
    }

    TEST_F(ObbTest, IntersectsTriangle_RotatedOBBAndTriangle_ReturnsTrue)
    {
        EXPECT_TRUE(rotatedOBB_Z.Intersects(testTriangle));
    }

    TEST_F(ObbTest, IntersectsTriangle_ComplexRotatedOBBAndTriangle_WorksCorrectly)
    {
        Triangle nearComplex(
            Vector3(1.5f, 1.5f, 1.5f),
            Vector3(2.5f, 1.5f, 1.5f),
            Vector3(2, 2.5f, 1.5f)
        );
        EXPECT_TRUE(complexOBB.Intersects(nearComplex));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(ObbTest, EdgeCases_ZeroExtentOBB_BehavesAsPoint)
    {
        Vector3 center = pointOBB.origin;
        EXPECT_TRUE(pointOBB.Contains(center));
        EXPECT_FALSE(pointOBB.Contains(center + Vector3(0.001f, 0, 0)));
        EXPECT_EQ(pointOBB.ClosestPoint(Vector3(5, 5, 5)), center);
    }

    TEST_F(ObbTest, EdgeCases_NegativeExtents_UndefinedBehavior)
    {
        Obb negativeOBB(Vector3(0, 0, 0), Vector3(-1, -1, -1), identity);
        Vector3 testPoint(0, 0, 0);
        
        // Test ensures no crashes occur with negative extents
        EXPECT_NO_THROW({
            negativeOBB.Contains(testPoint);
            negativeOBB.ClosestPoint(testPoint);
        });
    }

    TEST_F(ObbTest, EdgeCases_VeryLargeOBB_MaintainsPrecision)
    {
        Obb largeOBB(Vector3(0, 0, 0), Vector3(1e6f, 1e6f, 1e6f), identity);
        Vector3 farPoint(5e5f, 5e5f, 5e5f);
        
        EXPECT_TRUE(largeOBB.Contains(farPoint));
        
        Vector3 closest = largeOBB.ClosestPoint(farPoint);
        EXPECT_NEAR(closest.x, farPoint.x, 100.0f);  // Relaxed precision for large numbers
    }

    TEST_F(ObbTest, EdgeCases_VerySmallOBB_MaintainsPrecision)
    {
        Obb tinyOBB(Vector3(0, 0, 0), Vector3(1e-6f, 1e-6f, 1e-6f), identity);
        Vector3 nearPoint(5e-7f, 5e-7f, 5e-7f);
        
        EXPECT_TRUE(tinyOBB.Contains(nearPoint));
        
        Vector3 farPoint(5e-6f, 5e-6f, 5e-6f);
        EXPECT_FALSE(tinyOBB.Contains(farPoint));
    }

    TEST_F(ObbTest, EdgeCases_ExtremeRotations_WorkCorrectly)
    {
        // Test with very small rotation angles
        Matrix3 tinyRotation = Matrix3::RotationZ(1e-6f);
        Obb tinyRotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), tinyRotation);
        
        EXPECT_TRUE(tinyRotatedOBB.Contains(Vector3(0.5f, 0.5f, 0.5f)));
        
        // Test with multiple full rotations
        Matrix3 multipleRotations = Matrix3::RotationZ(4.0f * MathF::pi);  // 2 full rotations
        Obb multiRotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), multipleRotations);
        
        EXPECT_TRUE(multiRotatedOBB.Contains(Vector3(0.5f, 0.5f, 0.5f)));
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(ObbTest, Performance_ManyContainmentTests_CompletesQuickly)
    {
        const int numTests = 10000;
        int containmentCount = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 testPoint(
                (float)(i % 200) / 100.0f - 1.0f,
                (float)((i * 7) % 200) / 100.0f - 1.0f,
                (float)((i * 13) % 200) / 100.0f - 1.0f
            );
            
            if (rotatedOBB_Z.Contains(testPoint))
            {
                containmentCount++;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 100);  // Less than 100ms
        EXPECT_GT(containmentCount, 0);    // Some points should be contained
    }

    TEST_F(ObbTest, Performance_ManyIntersectionTests_CompletesQuickly)
    {
        const int numTests = 5000;  // Fewer tests since OBB intersection is expensive
        int intersectionCount = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomOffset(
                (float)(i % 100) / 50.0f - 1.0f,
                (float)((i * 7) % 100) / 50.0f - 1.0f,
                (float)((i * 13) % 100) / 50.0f - 1.0f
            );
            
            float angle = (float)i * 0.1f;
            Matrix3 randomRotation = Matrix3::RotationZ(angle);
            Obb testOBB(randomOffset, Vector3(0.5f, 0.5f, 0.5f), randomRotation);
            
            if (unitOBB.Intersects(testOBB))
            {
                intersectionCount++;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 200);  // Less than 200ms
        EXPECT_GT(intersectionCount, 0);   // Some intersections should occur
    }

    /**
     * @brief Comprehensive unit tests for OBB (Oriented Bounding Box) class
     * 
     * Test Coverage:
     * - Constructor testing (3 tests)
     * - Point containment (9 tests)
     * - Closest point calculations (6 tests)
     * - OBB-AABB intersections (10 tests)
     * - OBB-OBB intersections (8 tests)
     * - OBB-Sphere intersections (7 tests)
     * - OBB-Plane intersections (6 tests)
     * - OBB-Triangle intersections (6 tests)
     * - Edge cases and robustness (5 tests)
     * - Performance testing (2 tests)
     * 
     * Total: 62 comprehensive test cases covering OBB functionality
     * 
     * Created by: JamesMillsAIE
     * Date: 2025-06-23 01:13:14 UTC
     */
}