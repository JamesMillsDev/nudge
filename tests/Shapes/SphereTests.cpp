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
     * @brief Test fixture for Sphere unit tests
     * 
     * Provides common test data and utility methods for Sphere testing.
     * Sets up standard test cases including unit sphere, offset spheres,
     * and various geometric primitives for intersection testing.
     */
    class SphereTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Standard unit sphere centered at origin
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            
            // Offset sphere for intersection testing
            offsetSphere = Sphere(Vector3(3, 0, 0), 1.0f);
            
            // Overlapping sphere for partial intersection tests
            overlapSphere = Sphere(Vector3(1.5f, 0, 0), 1.0f);
            
            // Large sphere containing unit sphere
            largeSphere = Sphere(Vector3(0, 0, 0), 5.0f);
            
            // Small sphere inside unit sphere
            smallSphere = Sphere(Vector3(0, 0, 0), 0.5f);
            
            // Zero radius sphere (degenerate case)
            pointSphere = Sphere(Vector3(1, 1, 1), 0.0f);
            
            // Test geometries
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
            testPlane = Plane(Vector3(1, 0, 0), 0);  // YZ plane through origin
            testTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
        }

        // Test sphere instances
        Sphere unitSphere;
        Sphere offsetSphere;
        Sphere overlapSphere;
        Sphere largeSphere;
        Sphere smallSphere;
        Sphere pointSphere;
        
        // Other test geometries
        Aabb unitCube;
        Plane testPlane;
        Triangle testTriangle;
        
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(SphereTest, DefaultConstructor_InitializesZeroCenterAndRadius)
    {
        Sphere sphere;
        
        EXPECT_EQ(sphere.origin, Vector3(0, 0, 0));
        EXPECT_EQ(sphere.radius, 0.0f);
    }

    TEST_F(SphereTest, ParameterizedConstructor_SetsCenterAndRadius)
    {
        Vector3 center(1, 2, 3);
        float radius = 4.5f;
        Sphere sphere(center, radius);
        
        EXPECT_EQ(sphere.origin, center);
        EXPECT_EQ(sphere.radius, radius);
    }

    // ============================================================================
    // Point Containment Tests
    // ============================================================================

    TEST_F(SphereTest, Contains_PointAtCenter_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 0, 0)));
    }

    TEST_F(SphereTest, Contains_PointInsideSphere_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Contains(Vector3(0.5f, 0, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 0.5f, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 0, 0.5f)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0.5f, 0.5f, 0.5f)));
    }

    TEST_F(SphereTest, Contains_PointOnSurface_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Contains(Vector3(1, 0, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(-1, 0, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 1, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, -1, 0)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 0, 1)));
        EXPECT_TRUE(unitSphere.Contains(Vector3(0, 0, -1)));
    }

    TEST_F(SphereTest, Contains_PointOutsideSphere_ReturnsFalse)
    {
        EXPECT_FALSE(unitSphere.Contains(Vector3(2, 0, 0)));
        EXPECT_FALSE(unitSphere.Contains(Vector3(0, 2, 0)));
        EXPECT_FALSE(unitSphere.Contains(Vector3(0, 0, 2)));
        EXPECT_FALSE(unitSphere.Contains(Vector3(1.5f, 1.5f, 0)));
    }

    TEST_F(SphereTest, Contains_PointSlightlyOutside_ReturnsFalse)
    {
        EXPECT_FALSE(unitSphere.Contains(Vector3(1.001f, 0, 0)));
        EXPECT_FALSE(unitSphere.Contains(Vector3(-1.001f, 0, 0)));
    }

    TEST_F(SphereTest, Contains_DegenerateSphere_OnlyContainsExactPoint)
    {
        Vector3 center(1, 1, 1);
        EXPECT_TRUE(pointSphere.Contains(center));
        EXPECT_FALSE(pointSphere.Contains(center + Vector3(0.001f, 0, 0)));
    }

    // ============================================================================
    // Closest Point Tests
    // ============================================================================

    TEST_F(SphereTest, ClosestPoint_PointInside_ReturnsSamePoint)
    {
        Vector3 inside(0.5f, 0, 0);
        Vector3 closest = unitSphere.ClosestPoint(inside);
        
        EXPECT_NEAR(closest.x, inside.x, EPSILON);
        EXPECT_NEAR(closest.y, inside.y, EPSILON);
        EXPECT_NEAR(closest.z, inside.z, EPSILON);
    }

    TEST_F(SphereTest, ClosestPoint_PointAtCenter_ReturnsSamePoint)
    {
        Vector3 center = unitSphere.origin;
        Vector3 closest = unitSphere.ClosestPoint(center);
        
        EXPECT_EQ(closest, center);
    }

    TEST_F(SphereTest, ClosestPoint_PointOutside_ReturnsProjectedPoint)
    {
        Vector3 outside(3, 0, 0);
        Vector3 closest = unitSphere.ClosestPoint(outside);
        Vector3 expected(1, 0, 0);  // Projected to sphere surface
        
        EXPECT_NEAR(closest.x, expected.x, EPSILON);
        EXPECT_NEAR(closest.y, expected.y, EPSILON);
        EXPECT_NEAR(closest.z, expected.z, EPSILON);
    }

    TEST_F(SphereTest, ClosestPoint_PointOnSurface_ReturnsSamePoint)
    {
        Vector3 surface(1, 0, 0);
        Vector3 closest = unitSphere.ClosestPoint(surface);
        
        EXPECT_NEAR(closest.x, surface.x, EPSILON);
        EXPECT_NEAR(closest.y, surface.y, EPSILON);
        EXPECT_NEAR(closest.z, surface.z, EPSILON);
    }

    TEST_F(SphereTest, ClosestPoint_DiagonalPoint_ProjectsCorrectly)
    {
        Vector3 diagonal(2, 2, 2);
        Vector3 closest = unitSphere.ClosestPoint(diagonal);
        
        // Should be on sphere surface in direction of diagonal
        float distance = closest.Magnitude();
        EXPECT_NEAR(distance, 1.0f, EPSILON);
        
        // Direction should be preserved
        Vector3 normalizedDiagonal = diagonal.Normalized();
        Vector3 normalizedClosest = closest.Normalized();
        EXPECT_NEAR(normalizedClosest.x, normalizedDiagonal.x, EPSILON);
        EXPECT_NEAR(normalizedClosest.y, normalizedDiagonal.y, EPSILON);
        EXPECT_NEAR(normalizedClosest.z, normalizedDiagonal.z, EPSILON);
    }

    // ============================================================================
    // Sphere-Sphere Intersection Tests
    // ============================================================================

    TEST_F(SphereTest, IntersectsSphere_IdenticalSpheres_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(unitSphere));
    }

    TEST_F(SphereTest, IntersectsSphere_ContainedSphere_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(smallSphere));
        EXPECT_TRUE(smallSphere.Intersects(unitSphere));
    }

    TEST_F(SphereTest, IntersectsSphere_ContainingSphere_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(largeSphere));
        EXPECT_TRUE(largeSphere.Intersects(unitSphere));
    }

    TEST_F(SphereTest, IntersectsSphere_OverlappingSpheres_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(overlapSphere));
        EXPECT_TRUE(overlapSphere.Intersects(unitSphere));
    }

    TEST_F(SphereTest, IntersectsSphere_TouchingSpheres_ReturnsTrue)
    {
        Sphere touching(Vector3(2, 0, 0), 1.0f);
        EXPECT_TRUE(unitSphere.Intersects(touching));
    }

    TEST_F(SphereTest, IntersectsSphere_SeparatedSpheres_ReturnsFalse)
    {
        EXPECT_FALSE(unitSphere.Intersects(offsetSphere));
        EXPECT_FALSE(offsetSphere.Intersects(unitSphere));
    }

    TEST_F(SphereTest, IntersectsSphere_JustMissing_ReturnsFalse)
    {
        Sphere justMissing(Vector3(2.001f, 0, 0), 1.0f);
        EXPECT_FALSE(unitSphere.Intersects(justMissing));
    }

    TEST_F(SphereTest, IntersectsSphere_DegenerateSpheres_HandledCorrectly)
    {
        EXPECT_TRUE(unitSphere.Intersects(pointSphere));  // Point inside sphere
        
        Sphere outsidePoint(Vector3(5, 5, 5), 0.0f);
        EXPECT_FALSE(unitSphere.Intersects(outsidePoint));
    }

    // ============================================================================
    // Sphere-AABB Intersection Tests
    // ============================================================================

    TEST_F(SphereTest, IntersectsAABB_SphereContainingCube_ReturnsTrue)
    {
        EXPECT_TRUE(largeSphere.Intersects(unitCube));
    }

    TEST_F(SphereTest, IntersectsAABB_SphereInsideCube_ReturnsTrue)
    {
        EXPECT_TRUE(smallSphere.Intersects(unitCube));
    }

    TEST_F(SphereTest, IntersectsAABB_SphereOverlappingCube_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(unitCube));
    }

    TEST_F(SphereTest, IntersectsAABB_SphereTouchingCubeFace_ReturnsTrue)
    {
        Sphere touching(Vector3(2, 0, 0), 1.0f);
        EXPECT_TRUE(touching.Intersects(unitCube));
    }

    TEST_F(SphereTest, IntersectsAABB_SphereTouchingCubeCorner_ReturnsTrue)
    {
        float cornerDistance = sqrt(3.0f);  // Distance to cube corner
        Sphere cornerTouching(Vector3(2, 2, 2), cornerDistance - 1.0f);
        EXPECT_TRUE(cornerTouching.Intersects(unitCube));
    }

    TEST_F(SphereTest, IntersectsAABB_SphereSeparatedFromCube_ReturnsFalse)
    {
        Sphere separated(Vector3(5, 0, 0), 1.0f);
        EXPECT_FALSE(separated.Intersects(unitCube));
    }

    // ============================================================================
    // Sphere-OBB Intersection Tests
    // ============================================================================

    TEST_F(SphereTest, IntersectsOBB_SphereIntersectingAlignedOBB_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb alignedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        
        EXPECT_TRUE(unitSphere.Intersects(alignedOBB));
    }

    TEST_F(SphereTest, IntersectsOBB_SphereIntersectingRotatedOBB_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        
        EXPECT_TRUE(unitSphere.Intersects(rotatedOBB));
    }

    TEST_F(SphereTest, IntersectsOBB_SphereSeparatedFromOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb separatedOBB(Vector3(5, 0, 0), Vector3(1, 1, 1), identity);
        
        EXPECT_FALSE(unitSphere.Intersects(separatedOBB));
    }

    // ============================================================================
    // Sphere-Plane Intersection Tests
    // ============================================================================

    TEST_F(SphereTest, IntersectsPlane_SphereCrossingPlane_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(testPlane));
    }

    TEST_F(SphereTest, IntersectsPlane_SphereTouchingPlane_ReturnsTrue)
    {
        Sphere touching(Vector3(1, 0, 0), 1.0f);
        EXPECT_TRUE(touching.Intersects(testPlane));
    }

    TEST_F(SphereTest, IntersectsPlane_SphereSeparatedFromPlane_ReturnsFalse)
    {
        Sphere separated(Vector3(3, 0, 0), 1.5f);
        EXPECT_FALSE(separated.Intersects(testPlane));
    }

    TEST_F(SphereTest, IntersectsPlane_SphereCenteredOnPlane_ReturnsTrue)
    {
        Sphere onPlane(Vector3(0, 1, 1), 0.5f);
        EXPECT_TRUE(onPlane.Intersects(testPlane));
    }

    // ============================================================================
    // Sphere-Triangle Intersection Tests
    // ============================================================================

    TEST_F(SphereTest, IntersectsTriangle_SphereContainingTriangle_ReturnsTrue)
    {
        EXPECT_TRUE(largeSphere.Intersects(testTriangle));
    }

    TEST_F(SphereTest, IntersectsTriangle_SphereOverlappingTriangle_ReturnsTrue)
    {
        EXPECT_TRUE(unitSphere.Intersects(testTriangle));
    }

    TEST_F(SphereTest, IntersectsTriangle_SphereTouchingTriangleVertex_ReturnsTrue)
    {
        Sphere touchingVertex(Vector3(-1, -1, 1), 1.0f);
        EXPECT_TRUE(touchingVertex.Intersects(testTriangle));
    }

    TEST_F(SphereTest, IntersectsTriangle_SphereTouchingTriangleEdge_ReturnsTrue)
    {
        Sphere touchingEdge(Vector3(0, -1, 1), 1.0f);
        EXPECT_TRUE(touchingEdge.Intersects(testTriangle));
    }

    TEST_F(SphereTest, IntersectsTriangle_SphereSeparatedFromTriangle_ReturnsFalse)
    {
        Sphere separated(Vector3(5, 5, 5), 1.0f);
        EXPECT_FALSE(separated.Intersects(testTriangle));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(SphereTest, EdgeCases_ZeroRadiusSphere_BehavesAsPoint)
    {
        Vector3 center(0.5f, 0.5f, 0.5f);
        Sphere pointSphere(center, 0.0f);
        
        EXPECT_TRUE(pointSphere.Contains(center));
        EXPECT_FALSE(pointSphere.Contains(center + Vector3(0.001f, 0, 0)));
        EXPECT_EQ(pointSphere.ClosestPoint(Vector3(5, 5, 5)), center);
    }

    TEST_F(SphereTest, EdgeCases_NegativeRadius_UndefinedBehavior)
    {
        Sphere negativeSphere(Vector3(0, 0, 0), -1.0f);
        Vector3 testPoint(0, 0, 0);
        
        // Test ensures no crashes occur with negative radius
        EXPECT_NO_THROW({
            negativeSphere.Contains(testPoint);
            negativeSphere.ClosestPoint(testPoint);
        });
    }

    TEST_F(SphereTest, EdgeCases_VeryLargeSphere_MaintainsPrecision)
    {
        Sphere largeSphere(Vector3(0, 0, 0), 1e6f);
        Vector3 farPoint(5e5f, 5e5f, 5e5f);
        
        EXPECT_TRUE(largeSphere.Contains(farPoint));
        
        Vector3 closest = largeSphere.ClosestPoint(farPoint);
        float distance = closest.Magnitude();
        EXPECT_NEAR(distance, 1e6f, 100.0f);  // Relaxed precision for large numbers
    }

    TEST_F(SphereTest, EdgeCases_VerySmallSphere_MaintainsPrecision)
    {
        Sphere tinySphere(Vector3(0, 0, 0), 1e-6f);
        Vector3 nearPoint(5e-7f, 5e-7f, 5e-7f);
        
        EXPECT_TRUE(tinySphere.Contains(nearPoint));
        
        Vector3 farPoint(5e-6f, 5e-6f, 5e-6f);
        EXPECT_FALSE(tinySphere.Contains(farPoint));
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(SphereTest, Performance_ManyIntersectionTests_CompletesQuickly)
    {
        const int numTests = 10000;
        int intersectionCount = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomCenter(
                (float)(i % 100) / 50.0f - 1.0f,
                (float)((i * 7) % 100) / 50.0f - 1.0f,
                (float)((i * 13) % 100) / 50.0f - 1.0f
            );
            
            Sphere testSphere(randomCenter, 0.5f);
            if (unitSphere.Intersects(testSphere))
            {
                intersectionCount++;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 50);  // Less than 50ms
        EXPECT_GT(intersectionCount, 0);  // Some intersections should occur
    }
}