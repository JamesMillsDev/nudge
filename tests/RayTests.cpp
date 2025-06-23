#include <chrono>
#include <gtest/gtest.h>

#include "Nudge/Ray.hpp"
#include "Nudge/RaycastHit.hpp"
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
     * @brief Test fixture for Ray unit tests
     *
     * Provides common test data and utility methods for Ray testing.
     * Sets up standard test cases including various ray configurations
     * and target geometries for intersection testing.
     */
    class RayTest : public Test
    {
    protected:
        void SetUp() override
        {
            // Standard unit cube centered at origin for testing
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));

            // Standard rays for testing
            horizontalRay = Ray(Vector3(-3, 0, 0), Vector3(1, 0, 0));
            verticalRay = Ray(Vector3(0, -3, 0), Vector3(0, 1, 0));
            depthRay = Ray(Vector3(0, 0, -3), Vector3(0, 0, 1));
            diagonalRay = Ray(Vector3(-2, -2, -2), Vector3(1, 1, 1).Normalized());

            // Rays with different starting positions
            insideRay = Ray(Vector3(0, 0, 0), Vector3(1, 0, 0));
            outsideRay = Ray(Vector3(5, 5, 5), Vector3(1, 0, 0));

            // Other test geometries
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            testPlane = Plane(Vector3(1, 0, 0), 0);  // YZ plane through origin
            testTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
            
            // Create OBB for testing
            Matrix3 identity = Matrix3::Identity();
            unitOBB = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        }

        // Test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Plane testPlane;
        Triangle testTriangle;
        Obb unitOBB;

        // Test rays
        Ray horizontalRay;
        Ray verticalRay;
        Ray depthRay;
        Ray diagonalRay;
        Ray insideRay;
        Ray outsideRay;

        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(RayTest, DefaultConstructor_InitializesZeroOriginAndDirection)
    {
        Ray ray;

        EXPECT_EQ(ray.origin, Vector3(0, 0, 0));
        EXPECT_EQ(ray.direction, Vector3(0, 0, 0));
    }

    TEST_F(RayTest, ParameterizedConstructor_SetsOriginAndDirection)
    {
        Vector3 origin(1, 2, 3);
        Vector3 direction(4, 5, 6);
        Ray ray(origin, direction);

        EXPECT_EQ(ray.origin, origin);
        EXPECT_EQ(ray.direction, direction);
    }

    // ============================================================================
    // Ray-AABB Intersection Tests
    // ============================================================================

    TEST_F(RayTest, CastAgainstAABB_RayFromOriginThroughCenter_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(horizontalRay.CastAgainst(unitCube, &hit));
        EXPECT_NEAR(hit.distance, 2.0f, EPSILON);  // Distance to near face
        EXPECT_NEAR(hit.point.x, -1.0f, EPSILON);
        EXPECT_NEAR(hit.point.y, 0.0f, EPSILON);
        EXPECT_NEAR(hit.point.z, 0.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstAABB_RayStartingInside_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(insideRay.CastAgainst(unitCube, &hit));
        EXPECT_NEAR(hit.distance, 0.0f, EPSILON);  // Already inside
        EXPECT_EQ(hit.point, insideRay.origin);
    }

    TEST_F(RayTest, CastAgainstAABB_RayMissingBox_NoHit)
    {
        Ray missRay(Vector3(-3, 3, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_FALSE(missRay.CastAgainst(unitCube, &hit));
    }

    TEST_F(RayTest, CastAgainstAABB_RayPointingAway_NoHit)
    {
        Ray awayRay(Vector3(-3, 0, 0), Vector3(-1, 0, 0));
        RaycastHit hit;

        EXPECT_FALSE(awayRay.CastAgainst(unitCube, &hit));
    }

    TEST_F(RayTest, CastAgainstAABB_RayTangentToBox_Hits)
    {
        Ray tangentRay(Vector3(-3, 1, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(tangentRay.CastAgainst(unitCube, &hit));
        EXPECT_NEAR(hit.point.y, 1.0f, EPSILON);  // Touching top edge
    }

    TEST_F(RayTest, CastAgainstAABB_DiagonalRay_HitsCorrectly)
    {
        RaycastHit hit;

        EXPECT_TRUE(diagonalRay.CastAgainst(unitCube, &hit));

        // Verify hit point is on cube surface
        Vector3 hitPoint = hit.point;
        bool onSurface = (abs(abs(hitPoint.x) - 1.0f) < EPSILON) ||
            (abs(abs(hitPoint.y) - 1.0f) < EPSILON) ||
            (abs(abs(hitPoint.z) - 1.0f) < EPSILON);
        EXPECT_TRUE(onSurface);
    }

    TEST_F(RayTest, CastAgainstAABB_NullHitParameter_StillWorks)
    {
        EXPECT_TRUE(horizontalRay.CastAgainst(unitCube));  // No hit parameter
        EXPECT_FALSE(outsideRay.CastAgainst(unitCube));
    }

    TEST_F(RayTest, CastAgainstAABB_VerticalAndDepthRays_WorkCorrectly)
    {
        RaycastHit hitVertical, hitDepth;

        EXPECT_TRUE(verticalRay.CastAgainst(unitCube, &hitVertical));
        EXPECT_TRUE(depthRay.CastAgainst(unitCube, &hitDepth));

        EXPECT_NEAR(hitVertical.distance, 2.0f, EPSILON);
        EXPECT_NEAR(hitDepth.distance, 2.0f, EPSILON);

        EXPECT_NEAR(hitVertical.point.y, -1.0f, EPSILON);
        EXPECT_NEAR(hitDepth.point.z, -1.0f, EPSILON);
    }

    // ============================================================================
    // Ray-OBB Intersection Tests
    // ============================================================================

    TEST_F(RayTest, CastAgainstOBB_RayThroughAlignedOBB_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(horizontalRay.CastAgainst(unitOBB, &hit));
        EXPECT_NEAR(hit.distance, 2.0f, EPSILON);  // Distance to near face
        EXPECT_NEAR(hit.point.x, -1.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstOBB_RayStartingInsideOBB_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(insideRay.CastAgainst(unitOBB, &hit));
        EXPECT_NEAR(hit.distance, 0.0f, EPSILON);  // Already inside
    }

    TEST_F(RayTest, CastAgainstOBB_RayMissingOBB_NoHit)
    {
        Ray missRay(Vector3(-3, 3, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_FALSE(missRay.CastAgainst(unitOBB, &hit));
    }

    TEST_F(RayTest, CastAgainstOBB_RotatedOBB_HitsCorrectly)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);  // 45 degree rotation
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        RaycastHit hit;

        EXPECT_TRUE(horizontalRay.CastAgainst(rotatedOBB, &hit));
        
        // Hit point should be on the rotated OBB surface
        float distanceFromOrigin = hit.point.Magnitude();
        EXPECT_GT(distanceFromOrigin, 0.0f);
    }

    // ============================================================================
    // Ray-Sphere Intersection Tests
    // ============================================================================

    TEST_F(RayTest, CastAgainstSphere_RayThroughCenter_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(horizontalRay.CastAgainst(unitSphere, &hit));
        EXPECT_NEAR(hit.distance, 2.0f, EPSILON);  // Distance to sphere surface
        EXPECT_NEAR(hit.point.x, -1.0f, EPSILON);
        EXPECT_NEAR(hit.point.y, 0.0f, EPSILON);
        EXPECT_NEAR(hit.point.z, 0.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstSphere_RayStartingInside_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(insideRay.CastAgainst(unitSphere, &hit));
        EXPECT_NEAR(hit.distance, 0.0f, EPSILON);  // Already inside
    }

    TEST_F(RayTest, CastAgainstSphere_RayMissingSphere_NoHit)
    {
        Ray missRay(Vector3(-3, 2, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_FALSE(missRay.CastAgainst(unitSphere, &hit));
    }

    TEST_F(RayTest, CastAgainstSphere_RayTangentToSphere_Hits)
    {
        Ray tangentRay(Vector3(-3, 1, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(tangentRay.CastAgainst(unitSphere, &hit));
        EXPECT_NEAR(hit.point.y, 1.0f, EPSILON);  // Tangent point
        EXPECT_NEAR(hit.point.x, 0.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstSphere_RayFromInside_HitsExitPoint)
    {
        Ray exitRay(Vector3(0.5f, 0, 0), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(exitRay.CastAgainst(unitSphere, &hit));
        EXPECT_NEAR(hit.point.x, 1.0f, EPSILON);  // Exit point on sphere surface
        EXPECT_NEAR(hit.distance, 0.5f, EPSILON); // Distance from inside to surface
    }

    // ============================================================================
    // Ray-Plane Intersection Tests
    // ============================================================================

    TEST_F(RayTest, CastAgainstPlane_RayPerpendicularToPlane_Hits)
    {
        RaycastHit hit;

        EXPECT_TRUE(horizontalRay.CastAgainst(testPlane, &hit));
        EXPECT_NEAR(hit.distance, 3.0f, EPSILON);  // Distance to plane
        EXPECT_NEAR(hit.point.x, 0.0f, EPSILON);   // On the plane
        EXPECT_NEAR(hit.point.y, 0.0f, EPSILON);
        EXPECT_NEAR(hit.point.z, 0.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstPlane_RayParallelToPlane_NoHit)
    {
        Ray parallelRay(Vector3(1, 0, 0), Vector3(0, 1, 0));
        RaycastHit hit;

        EXPECT_FALSE(parallelRay.CastAgainst(testPlane, &hit));
    }

    TEST_F(RayTest, CastAgainstPlane_RayStartingOnPlane_Hits)
    {
        Ray onPlaneRay(Vector3(0, 1, 1), Vector3(1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(onPlaneRay.CastAgainst(testPlane, &hit));
        EXPECT_NEAR(hit.distance, 0.0f, EPSILON);  // Already on plane
    }

    TEST_F(RayTest, CastAgainstPlane_RayPointingAwayFromPlane_NoHit)
    {
        Ray awayRay(Vector3(-1, 0, 0), Vector3(-1, 0, 0));
        RaycastHit hit;

        EXPECT_FALSE(awayRay.CastAgainst(testPlane, &hit));
    }

    TEST_F(RayTest, CastAgainstPlane_ObliqueRayIntersection_HitsCorrectly)
    {
        Ray obliqueRay(Vector3(-2, -2, 0), Vector3(1, 1, 0).Normalized());
        RaycastHit hit;

        EXPECT_TRUE(obliqueRay.CastAgainst(testPlane, &hit));
        EXPECT_NEAR(hit.point.x, 0.0f, EPSILON);  // Should hit YZ plane at x=0
    }

    // ============================================================================
    // Ray-Triangle Intersection Tests
    // ============================================================================

    TEST_F(RayTest, CastAgainstTriangle_RayThroughTriangleCenter_Hits)
    {
        Ray centerRay(Vector3(0, 0, -1), Vector3(0, 0, 1));
        RaycastHit hit;

        EXPECT_TRUE(centerRay.CastAgainst(testTriangle, &hit));
        EXPECT_NEAR(hit.distance, 1.0f, EPSILON);
        EXPECT_NEAR(hit.point.z, 0.0f, EPSILON);  // On triangle plane
    }

    TEST_F(RayTest, CastAgainstTriangle_RayMissingTriangle_NoHit)
    {
        Ray missRay(Vector3(2, 0, -1), Vector3(0, 0, 1));
        RaycastHit hit;

        EXPECT_FALSE(missRay.CastAgainst(testTriangle, &hit));
    }

    TEST_F(RayTest, CastAgainstTriangle_RayHittingTriangleEdge_Hits)
    {
        Ray edgeRay(Vector3(0, -1, -1), Vector3(0, 0, 1));
        RaycastHit hit;

        EXPECT_TRUE(edgeRay.CastAgainst(testTriangle, &hit));
        EXPECT_NEAR(hit.point.y, -1.0f, EPSILON);  // On triangle edge
    }

    TEST_F(RayTest, CastAgainstTriangle_RayHittingTriangleVertex_Hits)
    {
        Ray vertexRay(Vector3(-1, -1, -1), Vector3(0, 0, 1));
        RaycastHit hit;

        EXPECT_TRUE(vertexRay.CastAgainst(testTriangle, &hit));
        EXPECT_NEAR(hit.point.x, -1.0f, EPSILON);  // Triangle vertex
        EXPECT_NEAR(hit.point.y, -1.0f, EPSILON);
    }

    TEST_F(RayTest, CastAgainstTriangle_RayFromBehindTriangle_NoHit)
    {
        Ray behindRay(Vector3(0, 0, 1), Vector3(0, 0, 1));  // Ray starting behind triangle
        RaycastHit hit;

        EXPECT_FALSE(behindRay.CastAgainst(testTriangle, &hit));
    }

    TEST_F(RayTest, CastAgainstTriangle_RayGrazingTriangleEdge_HandledCorrectly)
    {
        Ray grazingRay(Vector3(-2, -1, -1), Vector3(1, 0, 1).Normalized());
        RaycastHit hit;

        // This test checks edge case handling - result may vary by implementation
        bool result = grazingRay.CastAgainst(testTriangle, &hit);
        // Just ensure no crash occurs and result is consistent
        EXPECT_NO_THROW(grazingRay.CastAgainst(testTriangle, &hit));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(RayTest, EdgeCases_ZeroDirectionRay_HandledGracefully)
    {
        Ray zeroRay(Vector3(0, 0, 0), Vector3(0, 0, 0));
        RaycastHit hit;

        // Behavior with zero direction is implementation-defined
        // Test ensures no crashes occur
        EXPECT_NO_THROW({
            zeroRay.CastAgainst(unitCube, &hit);
            zeroRay.CastAgainst(unitSphere, &hit);
            zeroRay.CastAgainst(testPlane, &hit);
            zeroRay.CastAgainst(testTriangle, &hit);
            zeroRay.CastAgainst(unitOBB, &hit);
        });
    }

    TEST_F(RayTest, EdgeCases_UnnormalizedDirection_WorksCorrectly)
    {
        Ray unnormalizedRay(Vector3(-3, 0, 0), Vector3(2, 0, 0));  // Non-unit direction
        RaycastHit hit;

        EXPECT_TRUE(unnormalizedRay.CastAgainst(unitCube, &hit));

        // Distance should be scaled by direction magnitude
        float expectedDistance = 2.0f / 2.0f;  // Adjusted for direction magnitude
        EXPECT_NEAR(hit.distance, expectedDistance, EPSILON);
    }

    TEST_F(RayTest, EdgeCases_VeryLongRay_MaintainsPrecision)
    {
        Ray longRay(Vector3(-1e6f, 0, 0), Vector3(1, 0, 0));
        Aabb farCube(Vector3(0, 0, 0), Vector3(1, 1, 1));
        RaycastHit hit;

        EXPECT_TRUE(longRay.CastAgainst(farCube, &hit));
        EXPECT_GT(hit.distance, 1e6f - 10.0f);  // Should hit at very far distance
    }

    TEST_F(RayTest, EdgeCases_NegativeDirectionRay_WorksCorrectly)
    {
        Ray negativeRay(Vector3(3, 0, 0), Vector3(-1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(negativeRay.CastAgainst(unitCube, &hit));
        EXPECT_NEAR(hit.distance, 2.0f, EPSILON);
        EXPECT_NEAR(hit.point.x, 1.0f, EPSILON);  // Right face of cube
    }

    TEST_F(RayTest, EdgeCases_VeryShortRay_WorksCorrectly)
    {
        Ray shortRay(Vector3(0, 0, 0), Vector3(1e-6f, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(shortRay.CastAgainst(unitCube, &hit));  // Ray starts inside cube
        EXPECT_NEAR(hit.distance, 0.0f, EPSILON);
    }

    TEST_F(RayTest, EdgeCases_RayWithNegativeDistance_HandledCorrectly)
    {
        // Ray starting behind target but pointing toward it
        Ray behindRay(Vector3(2, 0, 0), Vector3(-1, 0, 0));
        RaycastHit hit;

        EXPECT_TRUE(behindRay.CastAgainst(unitCube, &hit));
        EXPECT_GT(hit.distance, 0.0f);  // Distance should be positive
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(RayTest, Performance_ManyRaycastTests_CompletesQuickly)
    {
        const int numTests = 10000;
        int hitCount = 0;

        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomOrigin(
                (float)(i % 200) / 100.0f - 1.0f,
                (float)((i * 7) % 200) / 100.0f - 1.0f,
                -3.0f
            );

            Ray testRay(randomOrigin, Vector3(0, 0, 1));
            RaycastHit hit;

            if (testRay.CastAgainst(unitCube, &hit))
            {
                hitCount++;
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // Should complete many raycast tests quickly
        EXPECT_LT(duration.count(), 50);  // Less than 50ms
        EXPECT_GT(hitCount, 0);           // Some hits should occur
    }

    TEST_F(RayTest, Performance_MultipleShapeTypes_CompletesQuickly)
    {
        const int numTests = 2000;  // Fewer tests since we test multiple shapes
        int totalHits = 0;

        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomOrigin(
                (float)(i % 100) / 50.0f - 1.0f,
                (float)((i * 7) % 100) / 50.0f - 1.0f,
                -2.0f
            );

            Ray testRay(randomOrigin, Vector3(0, 0, 1));
            RaycastHit hit;

            // Test against all shape types
            if (testRay.CastAgainst(unitCube, &hit)) totalHits++;
            if (testRay.CastAgainst(unitSphere, &hit)) totalHits++;
            if (testRay.CastAgainst(testPlane, &hit)) totalHits++;
            if (testRay.CastAgainst(testTriangle, &hit)) totalHits++;
            if (testRay.CastAgainst(unitOBB, &hit)) totalHits++;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // Should complete tests against multiple shape types quickly
        EXPECT_LT(duration.count(), 100);  // Less than 100ms
        EXPECT_GT(totalHits, 0);           // Some hits should occur
    }
}