#include <gtest/gtest.h>
#include <chrono>

#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/MathF.hpp"

namespace Nudge
{
    /**
     * @brief Test fixture for Plane unit tests
     * 
     * Provides common test data and utility methods for Plane testing.
     * Sets up standard test cases including various plane orientations
     * and target geometries for intersection testing.
     */
    class PlaneTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Standard planes for testing
            yzPlane = Plane(Vector3(1, 0, 0), 0);      // YZ plane through origin
            xzPlane = Plane(Vector3(0, 1, 0), 0);      // XZ plane through origin
            xyPlane = Plane(Vector3(0, 0, 1), 0);      // XY plane through origin
            
            // Offset planes
            offsetYZPlane = Plane(Vector3(1, 0, 0), -1);  // YZ plane at X = 1
            offsetXZPlane = Plane(Vector3(0, 1, 0), -1);  // XZ plane at Y = 1
            offsetXYPlane = Plane(Vector3(0, 0, 1), -1);  // XY plane at Z = 1
            
            // Diagonal plane
            diagonalPlane = Plane(Vector3(1, 1, 1).Normalized(), 0);
            
            // Test geometries
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            testTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
        }

        // Test plane instances
        Plane yzPlane;
        Plane xzPlane;
        Plane xyPlane;
        Plane offsetYZPlane;
        Plane offsetXZPlane;
        Plane offsetXYPlane;
        Plane diagonalPlane;
        
        // Other test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Triangle testTriangle;
        
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(PlaneTest, DefaultConstructor_InitializesZeroNormalAndDistance)
    {
        Plane plane;
        
        EXPECT_EQ(plane.normal, Vector3(0, 0, 0));
        EXPECT_EQ(plane.distance, 0.0f);
    }

    TEST_F(PlaneTest, ParameterizedConstructor_SetsNormalAndDistance)
    {
        Vector3 normal(1, 0, 0);
        float distance = 5.0f;
        Plane plane(normal, distance);
        
        EXPECT_EQ(plane.normal, normal);
        EXPECT_EQ(plane.distance, distance);
    }

    // ============================================================================
    // Factory Method Tests
    // ============================================================================

    TEST_F(PlaneTest, FromTriangle_CreatesCorrectPlane)
    {
        Plane plane = Plane::From(testTriangle);
        
        // Plane should be created from triangle
        // All three vertices should lie on the plane (distance = 0)
        EXPECT_NEAR(Plane::PlaneEquation(testTriangle.a, plane), 0.0f, EPSILON);
        EXPECT_NEAR(Plane::PlaneEquation(testTriangle.b, plane), 0.0f, EPSILON);
        EXPECT_NEAR(Plane::PlaneEquation(testTriangle.c, plane), 0.0f, EPSILON);
    }

    TEST_F(PlaneTest, FromTriangle_NormalIsNormalized)
    {
        Plane plane = Plane::From(testTriangle);
        
        // Normal should be unit length
        EXPECT_NEAR(plane.normal.Magnitude(), 1.0f, EPSILON);
    }

    // ============================================================================
    // Plane Equation Tests
    // ============================================================================

    TEST_F(PlaneTest, PlaneEquation_PointOnPlane_ReturnsZero)
    {
        EXPECT_NEAR(Plane::PlaneEquation(Vector3(0, 0, 0), yzPlane), 0.0f, EPSILON);
        EXPECT_NEAR(Plane::PlaneEquation(Vector3(0, 1, 0), yzPlane), 0.0f, EPSILON);
        EXPECT_NEAR(Plane::PlaneEquation(Vector3(0, 0, 1), yzPlane), 0.0f, EPSILON);
        EXPECT_NEAR(Plane::PlaneEquation(Vector3(0, 5, 5), yzPlane), 0.0f, EPSILON);
    }

    TEST_F(PlaneTest, PlaneEquation_PointInFrontOfPlane_ReturnsPositive)
    {
        EXPECT_GT(Plane::PlaneEquation(Vector3(1, 0, 0), yzPlane), 0.0f);
        EXPECT_GT(Plane::PlaneEquation(Vector3(5, 0, 0), yzPlane), 0.0f);
    }

    TEST_F(PlaneTest, PlaneEquation_PointBehindPlane_ReturnsNegative)
    {
        EXPECT_LT(Plane::PlaneEquation(Vector3(-1, 0, 0), yzPlane), 0.0f);
        EXPECT_LT(Plane::PlaneEquation(Vector3(-5, 0, 0), yzPlane), 0.0f);
    }

    TEST_F(PlaneTest, PlaneEquation_OffsetPlane_CalculatesCorrectly)
    {
        EXPECT_NEAR(Plane::PlaneEquation(Vector3(1, 0, 0), offsetYZPlane), 0.0f, EPSILON);
        EXPECT_GT(Plane::PlaneEquation(Vector3(2, 0, 0), offsetYZPlane), 0.0f);
        EXPECT_LT(Plane::PlaneEquation(Vector3(0, 0, 0), offsetYZPlane), 0.0f);
    }

    TEST_F(PlaneTest, PlaneEquation_DiagonalPlane_CalculatesCorrectly)
    {
        Vector3 onPlane = Vector3(1, 1, 1) * (1.0f / sqrt(3.0f));  // Point on diagonal plane
        EXPECT_NEAR(Plane::PlaneEquation(onPlane, diagonalPlane), 0.0f, EPSILON);
    }

    // ============================================================================
    // Point Containment Tests
    // ============================================================================

    TEST_F(PlaneTest, Contains_PointOnPlane_ReturnsTrue)
    {
        EXPECT_TRUE(yzPlane.Contains(Vector3(0, 0, 0)));
        EXPECT_TRUE(yzPlane.Contains(Vector3(0, 1, 1)));
    }

    TEST_F(PlaneTest, Contains_PointOffPlane_ReturnsFalse)
    {
        EXPECT_FALSE(yzPlane.Contains(Vector3(1, 0, 0)));
        EXPECT_FALSE(yzPlane.Contains(Vector3(-1, 0, 0)));
        EXPECT_FALSE(yzPlane.Contains(Vector3(5, 0, 0)));
    }

    TEST_F(PlaneTest, Contains_PointWithinTolerance_ReturnsTrue)
    {
        // Points very close to plane should be considered on plane
        EXPECT_TRUE(yzPlane.Contains(Vector3(1e-7f, 0, 0)));
        EXPECT_TRUE(yzPlane.Contains(Vector3(-1e-7f, 0, 0)));
    }

    // ============================================================================
    // Closest Point Tests
    // ============================================================================

    TEST_F(PlaneTest, ClosestPoint_PointOnPlane_ReturnsSamePoint)
    {
        Vector3 onPlane(0, 1, 1);
        Vector3 closest = yzPlane.ClosestPoint(onPlane);
        
        EXPECT_NEAR(closest.x, onPlane.x, EPSILON);
        EXPECT_NEAR(closest.y, onPlane.y, EPSILON);
        EXPECT_NEAR(closest.z, onPlane.z, EPSILON);
    }

    TEST_F(PlaneTest, ClosestPoint_PointOffPlane_ReturnsProjectedPoint)
    {
        Vector3 offPlane(5, 1, 1);
        Vector3 closest = yzPlane.ClosestPoint(offPlane);
        Vector3 expected(0, 1, 1);  // Projected onto YZ plane
        
        EXPECT_NEAR(closest.x, expected.x, EPSILON);
        EXPECT_NEAR(closest.y, expected.y, EPSILON);
        EXPECT_NEAR(closest.z, expected.z, EPSILON);
    }

    TEST_F(PlaneTest, ClosestPoint_OffsetPlane_ProjectsCorrectly)
    {
        Vector3 offPlane(5, 1, 1);
        Vector3 closest = offsetYZPlane.ClosestPoint(offPlane);
        Vector3 expected(-1, 1, 1);  // Projected onto YZ plane at X=1
        
        EXPECT_NEAR(closest.x, expected.x, EPSILON);
        EXPECT_NEAR(closest.y, expected.y, EPSILON);
        EXPECT_NEAR(closest.z, expected.z, EPSILON);
    }

    TEST_F(PlaneTest, ClosestPoint_NegativeDistance_ProjectsCorrectly)
    {
        Vector3 offPlane(-5, 1, 1);
        Vector3 closest = yzPlane.ClosestPoint(offPlane);
        Vector3 expected(0, 1, 1);  // Projected onto YZ plane
        
        EXPECT_NEAR(closest.x, expected.x, EPSILON);
        EXPECT_NEAR(closest.y, expected.y, EPSILON);
        EXPECT_NEAR(closest.z, expected.z, EPSILON);
    }

    // ============================================================================
    // Plane-AABB Intersection Tests
    // ============================================================================

    TEST_F(PlaneTest, IntersectsAABB_PlaneThroughCube_ReturnsTrue)
    {
        EXPECT_TRUE(yzPlane.Intersects(unitCube));
        EXPECT_TRUE(xzPlane.Intersects(unitCube));
        EXPECT_TRUE(xyPlane.Intersects(unitCube));
    }

    TEST_F(PlaneTest, IntersectsAABB_PlaneTouchingCubeFace_ReturnsTrue)
    {
        Plane touchingFace(Vector3(1, 0, 0), -1);  // Touching right face
        EXPECT_TRUE(touchingFace.Intersects(unitCube));
    }

    TEST_F(PlaneTest, IntersectsAABB_PlaneSeparatedFromCube_ReturnsFalse)
    {
        Plane separated(Vector3(1, 0, 0), -3);  // Far from cube
        EXPECT_FALSE(separated.Intersects(unitCube));
    }

    TEST_F(PlaneTest, IntersectsAABB_DiagonalPlaneThroughCube_ReturnsTrue)
    {
        EXPECT_TRUE(diagonalPlane.Intersects(unitCube));
    }

    TEST_F(PlaneTest, IntersectsAABB_PlaneJustMissingCube_ReturnsFalse)
    {
        Plane justMissing(Vector3(1, 0, 0), -1.001f);  // Just beyond cube
        EXPECT_FALSE(justMissing.Intersects(unitCube));
    }

    // ============================================================================
    // Plane-OBB Intersection Tests
    // ============================================================================

    TEST_F(PlaneTest, IntersectsOBB_PlaneThroughAlignedOBB_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb alignedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        
        EXPECT_TRUE(yzPlane.Intersects(alignedOBB));
        EXPECT_TRUE(xzPlane.Intersects(alignedOBB));
        EXPECT_TRUE(xyPlane.Intersects(alignedOBB));
    }

    TEST_F(PlaneTest, IntersectsOBB_PlaneThroughRotatedOBB_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        
        EXPECT_TRUE(yzPlane.Intersects(rotatedOBB));
    }

    TEST_F(PlaneTest, IntersectsOBB_PlaneSeparatedFromOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb separatedOBB(Vector3(5, 0, 0), Vector3(1, 1, 1), identity);
        Plane farPlane(Vector3(1, 0, 0), -3);
        
        EXPECT_FALSE(farPlane.Intersects(separatedOBB));
    }

    TEST_F(PlaneTest, IntersectsOBB_PlaneTouchingRotatedOBB_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        
        // Plane that should just touch the rotated OBB
        float touchDistance = sqrt(2.0f);  // Distance to corner of rotated unit square
        Plane touchingPlane(Vector3(1, 0, 0), -touchDistance);
        
        EXPECT_TRUE(touchingPlane.Intersects(rotatedOBB));
    }

    // ============================================================================
    // Plane-Sphere Intersection Tests
    // ============================================================================

    TEST_F(PlaneTest, IntersectsSphere_PlaneThroughSphere_ReturnsTrue)
    {
        EXPECT_TRUE(yzPlane.Intersects(unitSphere));
        EXPECT_TRUE(xzPlane.Intersects(unitSphere));
        EXPECT_TRUE(xyPlane.Intersects(unitSphere));
    }

    TEST_F(PlaneTest, IntersectsSphere_PlaneTouchingSphere_ReturnsTrue)
    {
        Plane touching(Vector3(1, 0, 0), -1);  // Touching sphere surface
        EXPECT_TRUE(touching.Intersects(unitSphere));
    }

    TEST_F(PlaneTest, IntersectsSphere_PlaneSeparatedFromSphere_ReturnsFalse)
    {
        Plane separated(Vector3(1, 0, 0), -2);  // Beyond sphere
        EXPECT_FALSE(separated.Intersects(unitSphere));
    }

    TEST_F(PlaneTest, IntersectsSphere_PlaneJustMissing_ReturnsFalse)
    {
        Plane justMissing(Vector3(1, 0, 0), -1.001f);
        EXPECT_FALSE(justMissing.Intersects(unitSphere));
    }

    TEST_F(PlaneTest, IntersectsSphere_OffsetSphere_WorksCorrectly)
    {
        Sphere offsetSphere(Vector3(2, 0, 0), 1.0f);
        
        EXPECT_TRUE(yzPlane.Intersects(offsetSphere));  // Sphere crosses plane
        
        Plane farPlane(Vector3(1, 0, 0), -4);
        EXPECT_FALSE(farPlane.Intersects(offsetSphere));  // Plane too far
    }

    // ============================================================================
    // Plane-Triangle Intersection Tests
    // ============================================================================

    TEST_F(PlaneTest, IntersectsTriangle_PlaneThroughTriangle_ReturnsTrue)
    {
        EXPECT_TRUE(xyPlane.Intersects(testTriangle));  // Triangle lies on XY plane
    }

    TEST_F(PlaneTest, IntersectsTriangle_PlaneCrossingTriangle_ReturnsTrue)
    {
        Plane crossing(Vector3(1, 0, 0), 0);  // YZ plane through triangle
        EXPECT_TRUE(crossing.Intersects(testTriangle));
    }

    TEST_F(PlaneTest, IntersectsTriangle_PlaneSeparatedFromTriangle_ReturnsFalse)
    {
        Plane separated(Vector3(0, 0, 1), -2);  // Below triangle
        EXPECT_FALSE(separated.Intersects(testTriangle));
    }

    TEST_F(PlaneTest, IntersectsTriangle_PlaneTouchingTriangleVertex_ReturnsTrue)
    {
        Plane touchingVertex(Vector3(0, 0, 1), 0);  // XY plane touching triangle
        EXPECT_TRUE(touchingVertex.Intersects(testTriangle));
    }

    TEST_F(PlaneTest, IntersectsTriangle_PlaneJustMissingTriangle_ReturnsFalse)
    {
        Plane justMissing(Vector3(0, 0, 1), -0.001f);  // Just below triangle
        EXPECT_FALSE(justMissing.Intersects(testTriangle));
    }

    // ============================================================================
    // Plane-Plane Intersection Tests
    // ============================================================================

    TEST_F(PlaneTest, IntersectsPlane_ParallelPlanes_ReturnsFalse)
    {
        Plane parallel1(Vector3(1, 0, 0), 0);
        Plane parallel2(Vector3(1, 0, 0), -1);
        
        EXPECT_FALSE(parallel1.Intersects(parallel2));
        EXPECT_FALSE(parallel2.Intersects(parallel1));
    }

    TEST_F(PlaneTest, IntersectsPlane_IdenticalPlanes_ReturnsTrue)
    {
        EXPECT_TRUE(yzPlane.Intersects(yzPlane));
        EXPECT_TRUE(xzPlane.Intersects(xzPlane));
    }

    TEST_F(PlaneTest, IntersectsPlane_IntersectingPlanes_ReturnsTrue)
    {
        EXPECT_TRUE(yzPlane.Intersects(xzPlane));
        EXPECT_TRUE(xzPlane.Intersects(yzPlane));
        EXPECT_TRUE(yzPlane.Intersects(xyPlane));
        EXPECT_TRUE(xyPlane.Intersects(xzPlane));
    }

    TEST_F(PlaneTest, IntersectsPlane_OppositeNormals_ReturnsTrue)
    {
        Plane opposite(Vector3(-1, 0, 0), 0);  // Opposite normal to yzPlane
        EXPECT_TRUE(yzPlane.Intersects(opposite));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(PlaneTest, EdgeCases_ZeroNormalPlane_UndefinedBehavior)
    {
        Plane zeroNormalPlane(Vector3(0, 0, 0), 0);
        Vector3 testPoint(1, 1, 1);
        
        // Test ensures no crashes occur with zero normal
        EXPECT_NO_THROW({
            Plane::PlaneEquation(testPoint, zeroNormalPlane);
            zeroNormalPlane.Contains(testPoint);
            zeroNormalPlane.ClosestPoint(testPoint);
        });
    }

    TEST_F(PlaneTest, EdgeCases_UnnormalizedNormal_WorksCorrectly)
    {
        Plane unnormalizedPlane(Vector3(2, 0, 0), 0);  // Non-unit normal
        
        // Should still work correctly (implementation may normalize internally)
        float equation = Plane::PlaneEquation(Vector3(1, 0, 0), unnormalizedPlane);
        EXPECT_GT(equation, 0.0f);  // Should be on positive side
    }

    TEST_F(PlaneTest, EdgeCases_VeryLargeDistance_MaintainsPrecision)
    {
        Plane farPlane(Vector3(1, 0, 0), 1e6f);
        Vector3 testPoint(0, 0, 0);
        
        float equation = Plane::PlaneEquation(testPoint, farPlane);
        EXPECT_LT(equation, 0.0f);  // Should be on negative side
        EXPECT_NEAR(equation, -1e6f, 1.0f);  // Relaxed precision for large numbers
    }

    TEST_F(PlaneTest, EdgeCases_DegenerateTriangle_HandledGracefully)
    {
        Triangle degenerate(
            Vector3(0, 0, 0),
            Vector3(1, 0, 0),
            Vector3(2, 0, 0)  // Collinear points
        );
        
        // Should handle degenerate case gracefully
        EXPECT_NO_THROW({
            Plane::From(degenerate);
        });
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(PlaneTest, Performance_ManyPlaneEquationCalculations_CompletesQuickly)
    {
        const int numTests = 100000;
        float totalEquation = 0.0f;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 testPoint(
                (float)(i % 100) / 50.0f - 1.0f,
                (float)((i * 7) % 100) / 50.0f - 1.0f,
                (float)((i * 13) % 100) / 50.0f - 1.0f
            );
            
            totalEquation += abs(Plane::PlaneEquation(testPoint, yzPlane));
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 100);  // Less than 100ms
        EXPECT_GT(totalEquation, 0.0f);    // Some meaningful calculations occurred
    }

    TEST_F(PlaneTest, Performance_ManyIntersectionTests_CompletesQuickly)
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
            if (yzPlane.Intersects(testSphere))
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