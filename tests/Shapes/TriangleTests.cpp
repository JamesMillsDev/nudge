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
     * @brief Test fixture for Triangle unit tests
     * 
     * Provides common test data and utility methods for Triangle testing.
     * Sets up standard test cases including various triangle configurations
     * and target geometries for intersection testing.
     */
    class TriangleTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Standard triangle on XY plane
            standardTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
            
            // Right triangle
            rightTriangle = Triangle(
                Vector3(0, 0, 0),
                Vector3(1, 0, 0),
                Vector3(0, 1, 0)
            );
            
            // Equilateral triangle
            float height = sqrt(3.0f) / 2.0f;
            equilateralTriangle = Triangle(
                Vector3(-0.5f, -height/3.0f, 0),
                Vector3(0.5f, -height/3.0f, 0),
                Vector3(0, 2.0f*height/3.0f, 0)
            );
            
            // Large triangle
            largeTriangle = Triangle(
                Vector3(-5, -5, 0),
                Vector3(5, -5, 0),
                Vector3(0, 5, 0)
            );
            
            // Degenerate triangle (collinear points)
            degenerateTriangle = Triangle(
                Vector3(0, 0, 0),
                Vector3(1, 0, 0),
                Vector3(2, 0, 0)
            );
            
            // 3D triangle (not on XY plane)
            triangle3D = Triangle(
                Vector3(1, 0, 0),
                Vector3(0, 1, 0),
                Vector3(0, 0, 1)
            );
            
            // Test geometries
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            xyPlane = Plane(Vector3(0, 0, 1), 0);
        }

        // Test triangle instances
        Triangle standardTriangle;
        Triangle rightTriangle;
        Triangle equilateralTriangle;
        Triangle largeTriangle;
        Triangle degenerateTriangle;
        Triangle triangle3D;
        
        // Other test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Plane xyPlane;
        
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(TriangleTest, DefaultConstructor_InitializesZeroVertices)
    {
        Triangle triangle;
        
        EXPECT_EQ(triangle.a, Vector3(0, 0, 0));
        EXPECT_EQ(triangle.b, Vector3(0, 0, 0));
        EXPECT_EQ(triangle.c, Vector3(0, 0, 0));
    }

    TEST_F(TriangleTest, ParameterizedConstructor_SetsVertices)
    {
        Vector3 a(1, 0, 0);
        Vector3 b(0, 1, 0);
        Vector3 c(0, 0, 1);
        Triangle triangle(a, b, c);
        
        EXPECT_EQ(triangle.a, a);
        EXPECT_EQ(triangle.b, b);
        EXPECT_EQ(triangle.c, c);
    }

    // ============================================================================
    // Union Access Pattern Tests
    // ============================================================================

    TEST_F(TriangleTest, UnionAccess_PointsArray_AccessesVerticesCorrectly)
    {
        EXPECT_EQ(standardTriangle.points[0], standardTriangle.a);
        EXPECT_EQ(standardTriangle.points[1], standardTriangle.b);
        EXPECT_EQ(standardTriangle.points[2], standardTriangle.c);
    }

    TEST_F(TriangleTest, UnionAccess_ValuesArray_AccessesComponentsCorrectly)
    {
        // Test that values array provides access to raw float components
        EXPECT_EQ(standardTriangle.values[0], standardTriangle.a.x);
        EXPECT_EQ(standardTriangle.values[1], standardTriangle.a.y);
        EXPECT_EQ(standardTriangle.values[2], standardTriangle.a.z);
        
        EXPECT_EQ(standardTriangle.values[3], standardTriangle.b.x);
        EXPECT_EQ(standardTriangle.values[4], standardTriangle.b.y);
        EXPECT_EQ(standardTriangle.values[5], standardTriangle.b.z);
        
        EXPECT_EQ(standardTriangle.values[6], standardTriangle.c.x);
        EXPECT_EQ(standardTriangle.values[7], standardTriangle.c.y);
        EXPECT_EQ(standardTriangle.values[8], standardTriangle.c.z);
    }

    TEST_F(TriangleTest, UnionAccess_ModificationThroughArrays_UpdatesVertices)
    {
        Triangle triangle = standardTriangle;
        
        // Modify through points array
        triangle.points[0] = Vector3(5, 5, 5);
        EXPECT_EQ(triangle.a, Vector3(5, 5, 5));
        
        // Modify through values array
        triangle.values[3] = 10.0f;  // b.x
        EXPECT_EQ(triangle.b.x, 10.0f);
    }

    // ============================================================================
    // Point Containment Tests
    // ============================================================================

    TEST_F(TriangleTest, Contains_PointAtVertices_ReturnsTrue)
    {
        EXPECT_TRUE(standardTriangle.Contains(standardTriangle.a));
        EXPECT_TRUE(standardTriangle.Contains(standardTriangle.b));
        EXPECT_TRUE(standardTriangle.Contains(standardTriangle.c));
    }

    TEST_F(TriangleTest, Contains_PointOnEdges_ReturnsTrue)
    {
        Vector3 midpointAB = (standardTriangle.a + standardTriangle.b) * 0.5f;
        Vector3 midpointBC = (standardTriangle.b + standardTriangle.c) * 0.5f;
        Vector3 midpointCA = (standardTriangle.c + standardTriangle.a) * 0.5f;
        
        EXPECT_TRUE(standardTriangle.Contains(midpointAB));
        EXPECT_TRUE(standardTriangle.Contains(midpointBC));
        EXPECT_TRUE(standardTriangle.Contains(midpointCA));
    }

    TEST_F(TriangleTest, Contains_PointInsideTriangle_ReturnsTrue)
    {
        Vector3 inside(0, -0.5f, 0);  // Point clearly inside standard triangle
        EXPECT_TRUE(standardTriangle.Contains(inside));
        
        // Test centroid
        Vector3 centroid = (standardTriangle.a + standardTriangle.b + standardTriangle.c) / 3.0f;
        EXPECT_TRUE(standardTriangle.Contains(centroid));
    }

    TEST_F(TriangleTest, Contains_PointOutsideTriangle_ReturnsFalse)
    {
        Vector3 outside(2, 2, 0);  // Point clearly outside standard triangle
        EXPECT_FALSE(standardTriangle.Contains(outside));
        
        Vector3 outside2(-2, -2, 0);
        EXPECT_FALSE(standardTriangle.Contains(outside2));
    }

    TEST_F(TriangleTest, Contains_PointOffTrianglePlane_ReturnsFalse)
    {
        Vector3 abovePlane(0, 0, 1);  // Above the triangle plane
        Vector3 belowPlane(0, 0, -1); // Below the triangle plane
        
        EXPECT_FALSE(standardTriangle.Contains(abovePlane));
        EXPECT_FALSE(standardTriangle.Contains(belowPlane));
    }

    TEST_F(TriangleTest, Contains_DegenerateTriangle_HandledGracefully)
    {
        // Test with degenerate triangle - behavior may be implementation defined
        Vector3 onLine(1.5f, 0, 0);  // Point on the line of collinear vertices
        
        EXPECT_NO_THROW({
            bool result = degenerateTriangle.Contains(onLine);
            // Result may vary by implementation, just ensure no crash
        });
    }

    // ============================================================================
    // Closest Point Tests
    // ============================================================================

    TEST_F(TriangleTest, ClosestPoint_PointInside_ReturnsSamePoint)
    {
        Vector3 inside(0, -0.5f, 0);
        Vector3 closest = standardTriangle.ClosestPoint(inside);
        
        EXPECT_NEAR(closest.x, inside.x, EPSILON);
        EXPECT_NEAR(closest.y, inside.y, EPSILON);
        EXPECT_NEAR(closest.z, inside.z, EPSILON);
    }

    TEST_F(TriangleTest, ClosestPoint_PointAtVertices_ReturnsSamePoint)
    {
        Vector3 closest_a = standardTriangle.ClosestPoint(standardTriangle.a);
        Vector3 closest_b = standardTriangle.ClosestPoint(standardTriangle.b);
        Vector3 closest_c = standardTriangle.ClosestPoint(standardTriangle.c);
        
        EXPECT_NEAR(closest_a.x, standardTriangle.a.x, EPSILON);
        EXPECT_NEAR(closest_a.y, standardTriangle.a.y, EPSILON);
        EXPECT_NEAR(closest_a.z, standardTriangle.a.z, EPSILON);
        
        EXPECT_NEAR(closest_b.x, standardTriangle.b.x, EPSILON);
        EXPECT_NEAR(closest_b.y, standardTriangle.b.y, EPSILON);
        EXPECT_NEAR(closest_b.z, standardTriangle.b.z, EPSILON);
        
        EXPECT_NEAR(closest_c.x, standardTriangle.c.x, EPSILON);
        EXPECT_NEAR(closest_c.y, standardTriangle.c.y, EPSILON);
        EXPECT_NEAR(closest_c.z, standardTriangle.c.z, EPSILON);
    }

    TEST_F(TriangleTest, ClosestPoint_PointOutsideNearEdge_ReturnsEdgePoint)
    {
        Vector3 outsideNearEdge(0, -2, 0);  // Below bottom edge
        Vector3 closest = standardTriangle.ClosestPoint(outsideNearEdge);
        
        // Should project onto bottom edge
        EXPECT_NEAR(closest.y, -1.0f, EPSILON);
        EXPECT_NEAR(closest.z, 0.0f, EPSILON);
    }

    TEST_F(TriangleTest, ClosestPoint_PointOutsideNearVertex_ReturnsVertex)
    {
        Vector3 outsideNearA(-2, -2, 0);  // Near vertex A
        Vector3 closest = standardTriangle.ClosestPoint(outsideNearA);
        
        // Should be vertex A
        EXPECT_NEAR(closest.x, standardTriangle.a.x, EPSILON);
        EXPECT_NEAR(closest.y, standardTriangle.a.y, EPSILON);
        EXPECT_NEAR(closest.z, standardTriangle.a.z, EPSILON);
    }

    TEST_F(TriangleTest, ClosestPoint_PointAboveTriangle_ProjectsToPlane)
    {
        Vector3 above(0, 0, 5);  // Above triangle center
        Vector3 closest = standardTriangle.ClosestPoint(above);
        
        // Should project to triangle plane
        EXPECT_NEAR(closest.z, 0.0f, EPSILON);  // On triangle plane
        
        // Should be inside or on triangle boundary
        EXPECT_TRUE(standardTriangle.Contains(closest));
    }

    // ============================================================================
    // Barycentric Coordinate Tests
    // ============================================================================

    TEST_F(TriangleTest, Barycentric_VertexA_ReturnsCorrectCoordinates)
    {
        Vector3 bary = standardTriangle.Barycentric(standardTriangle.a);
        
        EXPECT_NEAR(bary.x, 1.0f, EPSILON);  // u = 1 for vertex A
        EXPECT_NEAR(bary.y, 0.0f, EPSILON);  // v = 0
        EXPECT_NEAR(bary.z, 0.0f, EPSILON);  // w = 0
    }

    TEST_F(TriangleTest, Barycentric_VertexB_ReturnsCorrectCoordinates)
    {
        Vector3 bary = standardTriangle.Barycentric(standardTriangle.b);
        
        EXPECT_NEAR(bary.x, 0.0f, EPSILON);  // u = 0
        EXPECT_NEAR(bary.y, 1.0f, EPSILON);  // v = 1 for vertex B
        EXPECT_NEAR(bary.z, 0.0f, EPSILON);  // w = 0
    }

    TEST_F(TriangleTest, Barycentric_VertexC_ReturnsCorrectCoordinates)
    {
        Vector3 bary = standardTriangle.Barycentric(standardTriangle.c);
        
        EXPECT_NEAR(bary.x, 0.0f, EPSILON);  // u = 0
        EXPECT_NEAR(bary.y, 0.0f, EPSILON);  // v = 0
        EXPECT_NEAR(bary.z, 1.0f, EPSILON);  // w = 1 for vertex C
    }

    TEST_F(TriangleTest, Barycentric_EdgeMidpoints_ReturnsCorrectCoordinates)
    {
        Vector3 midAB = (standardTriangle.a + standardTriangle.b) * 0.5f;
        Vector3 baryAB = standardTriangle.Barycentric(midAB);
        
        EXPECT_NEAR(baryAB.x, 0.5f, EPSILON);  // u = 0.5
        EXPECT_NEAR(baryAB.y, 0.5f, EPSILON);  // v = 0.5
        EXPECT_NEAR(baryAB.z, 0.0f, EPSILON);  // w = 0
    }

    TEST_F(TriangleTest, Barycentric_Centroid_ReturnsEqualCoordinates)
    {
        Vector3 centroid = (standardTriangle.a + standardTriangle.b + standardTriangle.c) / 3.0f;
        Vector3 bary = standardTriangle.Barycentric(centroid);
        
        EXPECT_NEAR(bary.x, 1.0f/3.0f, EPSILON);
        EXPECT_NEAR(bary.y, 1.0f/3.0f, EPSILON);
        EXPECT_NEAR(bary.z, 1.0f/3.0f, EPSILON);
    }

    TEST_F(TriangleTest, Barycentric_SumEqualsOne_ForPointsOnPlane)
    {
        Vector3 testPoint(0, -0.5f, 0);  // Point inside triangle
        Vector3 bary = standardTriangle.Barycentric(testPoint);
        
        float sum = bary.x + bary.y + bary.z;
        EXPECT_NEAR(sum, 1.0f, EPSILON);
    }

    TEST_F(TriangleTest, Barycentric_ReconstructsOriginalPoint)
    {
        Vector3 testPoint(0, -0.5f, 0);
        Vector3 bary = standardTriangle.Barycentric(testPoint);
        
        Vector3 reconstructed = standardTriangle.a * bary.x + 
                               standardTriangle.b * bary.y + 
                               standardTriangle.c * bary.z;
        
        EXPECT_NEAR(reconstructed.x, testPoint.x, EPSILON);
        EXPECT_NEAR(reconstructed.y, testPoint.y, EPSILON);
        EXPECT_NEAR(reconstructed.z, testPoint.z, EPSILON);
    }

    // ============================================================================
    // Triangle-AABB Intersection Tests
    // ============================================================================

    TEST_F(TriangleTest, IntersectsAABB_TriangleContainingCube_ReturnsTrue)
    {
        EXPECT_TRUE(largeTriangle.Intersects(unitCube));
    }

    TEST_F(TriangleTest, IntersectsAABB_TriangleInsideCube_ReturnsTrue)
    {
        Triangle smallTriangle(
            Vector3(-0.5f, -0.5f, 0),
            Vector3(0.5f, -0.5f, 0),
            Vector3(0, 0.5f, 0)
        );
        EXPECT_TRUE(smallTriangle.Intersects(unitCube));
    }

    TEST_F(TriangleTest, IntersectsAABB_TriangleIntersectingCube_ReturnsTrue)
    {
        EXPECT_TRUE(standardTriangle.Intersects(unitCube));
    }

    TEST_F(TriangleTest, IntersectsAABB_TriangleSeparatedFromCube_ReturnsFalse)
    {
        Triangle separated(
            Vector3(5, 5, 5),
            Vector3(6, 5, 5),
            Vector3(5, 6, 5)
        );
        EXPECT_FALSE(separated.Intersects(unitCube));
    }

    TEST_F(TriangleTest, IntersectsAABB_TriangleTouchingCubeFace_ReturnsTrue)
    {
        Triangle touching(
            Vector3(-1, -1, 1),
            Vector3(1, -1, 1),
            Vector3(0, 1, 1)
        );
        EXPECT_TRUE(touching.Intersects(unitCube));
    }

    TEST_F(TriangleTest, IntersectsAABB_TriangleTouchingCubeVertex_ReturnsTrue)
    {
        Triangle touchingVertex(
            Vector3(1, 1, 1),
            Vector3(2, 1, 1),
            Vector3(1, 2, 1)
        );
        EXPECT_TRUE(touchingVertex.Intersects(unitCube));
    }

    // ============================================================================
    // Triangle-OBB Intersection Tests
    // ============================================================================

    TEST_F(TriangleTest, IntersectsOBB_TriangleIntersectingAlignedOBB_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb alignedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        
        EXPECT_TRUE(standardTriangle.Intersects(alignedOBB));
    }

    TEST_F(TriangleTest, IntersectsOBB_TriangleIntersectingRotatedOBB_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        
        EXPECT_TRUE(standardTriangle.Intersects(rotatedOBB));
    }

    TEST_F(TriangleTest, IntersectsOBB_TriangleSeparatedFromOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb separatedOBB(Vector3(5, 5, 5), Vector3(1, 1, 1), identity);
        
        EXPECT_FALSE(standardTriangle.Intersects(separatedOBB));
    }

    TEST_F(TriangleTest, IntersectsOBB_ComplexRotatedOBB_HandlesCorrectly)
    {
        Matrix3 rotX = Matrix3::RotationX(MathF::pi / 6.0f);
        Matrix3 rotY = Matrix3::RotationY(MathF::pi / 4.0f);
        Matrix3 combined = rotY * rotX;
        Obb complexOBB(Vector3(0, 0, 0), Vector3(0.8f, 0.8f, 0.8f), combined);
        
        EXPECT_TRUE(standardTriangle.Intersects(complexOBB));
    }

    // ============================================================================
    // Triangle-Sphere Intersection Tests
    // ============================================================================

    TEST_F(TriangleTest, IntersectsSphere_TriangleInsideSphere_ReturnsTrue)
    {
        EXPECT_TRUE(standardTriangle.Intersects(unitSphere));
    }

    TEST_F(TriangleTest, IntersectsSphere_SphereContainingTriangle_ReturnsTrue)
    {
        Sphere largeSphere(Vector3(0, 0, 0), 5.0f);
        EXPECT_TRUE(standardTriangle.Intersects(largeSphere));
    }

    TEST_F(TriangleTest, IntersectsSphere_TriangleTouchingSphere_ReturnsTrue)
    {
        Triangle touching(
            Vector3(1, 0, 0),
            Vector3(2, -1, 0),
            Vector3(2, 1, 0)
        );
        EXPECT_TRUE(touching.Intersects(unitSphere));
    }

    TEST_F(TriangleTest, IntersectsSphere_TriangleSeparatedFromSphere_ReturnsFalse)
    {
        Triangle separated(
            Vector3(5, 5, 5),
            Vector3(6, 5, 5),
            Vector3(5, 6, 5)
        );
        EXPECT_FALSE(separated.Intersects(unitSphere));
    }

    TEST_F(TriangleTest, IntersectsSphere_TriangleVertexTouchingSphere_ReturnsTrue)
    {
        Triangle vertexTouching(
            Vector3(1, 0, 0),  // Vertex on sphere surface
            Vector3(3, 0, 0),
            Vector3(2, 2, 0)
        );
        EXPECT_TRUE(vertexTouching.Intersects(unitSphere));
    }

    // ============================================================================
    // Triangle-Plane Intersection Tests
    // ============================================================================

    TEST_F(TriangleTest, IntersectsPlane_TriangleOnPlane_ReturnsTrue)
    {
        EXPECT_TRUE(standardTriangle.Intersects(xyPlane));
    }

    TEST_F(TriangleTest, IntersectsPlane_TriangleCrossingPlane_ReturnsTrue)
    {
        Triangle crossing(
            Vector3(0, 0, -1),
            Vector3(1, 0, 1),
            Vector3(-1, 0, 1)
        );
        Plane xzPlane(Vector3(0, 0, 1), 0);
        EXPECT_TRUE(crossing.Intersects(xzPlane));
    }

    TEST_F(TriangleTest, IntersectsPlane_TriangleSeparatedFromPlane_ReturnsFalse)
    {
        Plane separated(Vector3(0, 0, 1), -2);  // Below triangle
        EXPECT_FALSE(standardTriangle.Intersects(separated));
    }

    TEST_F(TriangleTest, IntersectsPlane_TriangleTouchingPlane_ReturnsTrue)
    {
        Plane touchingVertex(Vector3(0, 0, 1), 0);  // XY plane touching triangle
        EXPECT_TRUE(standardTriangle.Intersects(touchingVertex));
    }

    TEST_F(TriangleTest, IntersectsPlane_OneVertexOnPlane_ReturnsTrue)
    {
        Triangle oneOnPlane(
            Vector3(0, 0, 0),  // On XY plane
            Vector3(1, 1, 1),  // Above plane
            Vector3(-1, -1, -1) // Below plane
        );
        EXPECT_TRUE(oneOnPlane.Intersects(xyPlane));
    }

    // ============================================================================
    // Triangle-Triangle Intersection Tests
    // ============================================================================

    TEST_F(TriangleTest, IntersectsTriangle_IdenticalTriangles_ReturnsTrue)
    {
        EXPECT_TRUE(standardTriangle.Intersects(standardTriangle));
    }

    TEST_F(TriangleTest, IntersectsTriangle_OverlappingTriangles_ReturnsTrue)
    {
        Triangle overlapping(
            Vector3(-0.5f, -0.5f, 0),
            Vector3(0.5f, -0.5f, 0),
            Vector3(0, 0.5f, 0)
        );
        EXPECT_TRUE(standardTriangle.Intersects(overlapping));
    }

    TEST_F(TriangleTest, IntersectsTriangle_TouchingTriangles_ReturnsTrue)
    {
        Triangle touching(
            Vector3(0, 1, 0),  // Shares vertex with standardTriangle.c
            Vector3(1, 2, 0),
            Vector3(-1, 2, 0)
        );
        EXPECT_TRUE(standardTriangle.Intersects(touching));
    }

    TEST_F(TriangleTest, IntersectsTriangle_EdgeCrossingTriangles_ReturnsTrue)
    {
        Triangle crossing(
            Vector3(-2, 0, 0),
            Vector3(2, 0, 0),
            Vector3(0, 0, 1)
        );
        EXPECT_TRUE(standardTriangle.Intersects(crossing));
    }

    TEST_F(TriangleTest, IntersectsTriangle_SeparatedTriangles_ReturnsFalse)
    {
        Triangle separated(
            Vector3(5, 5, 5),
            Vector3(6, 5, 5),
            Vector3(5, 6, 5)
        );
        EXPECT_FALSE(standardTriangle.Intersects(separated));
    }

    TEST_F(TriangleTest, IntersectsTriangle_CoplanarNonOverlapping_ReturnsFalse)
    {
        Triangle coplanarSeparated(
            Vector3(3, 3, 0),  // Same plane but no overlap
            Vector3(4, 3, 0),
            Vector3(3, 4, 0)
        );
        EXPECT_FALSE(standardTriangle.Intersects(coplanarSeparated));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(TriangleTest, EdgeCases_DegenerateTriangle_HandledGracefully)
    {
        Vector3 testPoint(1, 0, 0);
        
        // Test ensures no crashes occur with degenerate triangle
        EXPECT_NO_THROW({
            degenerateTriangle.Contains(testPoint);
            degenerateTriangle.ClosestPoint(testPoint);
            degenerateTriangle.Barycentric(testPoint);
        });
    }

    TEST_F(TriangleTest, EdgeCases_ZeroAreaTriangle_BehavesConsistently)
    {
        Triangle zeroArea(
            Vector3(1, 1, 1),
            Vector3(1, 1, 1),
            Vector3(1, 1, 1)
        );
        
        EXPECT_TRUE(zeroArea.Contains(Vector3(1, 1, 1)));
        EXPECT_FALSE(zeroArea.Contains(Vector3(2, 2, 2)));
        
        Vector3 closest = zeroArea.ClosestPoint(Vector3(5, 5, 5));
        EXPECT_EQ(closest, Vector3(1, 1, 1));
    }

    TEST_F(TriangleTest, EdgeCases_VeryLargeTriangle_MaintainsPrecision)
    {
        Triangle large(
            Vector3(-1e6f, -1e6f, 0),
            Vector3(1e6f, -1e6f, 0),
            Vector3(0, 1e6f, 0)
        );
        
        Vector3 testPoint(0, 0, 0);
        EXPECT_TRUE(large.Contains(testPoint));
        
        Vector3 closest = large.ClosestPoint(testPoint);
        EXPECT_NEAR(closest.x, testPoint.x, 1.0f);  // Relaxed precision
        EXPECT_NEAR(closest.y, testPoint.y, 1.0f);
        EXPECT_NEAR(closest.z, testPoint.z, EPSILON);
    }

    TEST_F(TriangleTest, EdgeCases_VerySmallTriangle_MaintainsPrecision)
    {
        Triangle tiny(
            Vector3(-1e-6f, -1e-6f, 0),
            Vector3(1e-6f, -1e-6f, 0),
            Vector3(0, 1e-6f, 0)
        );
        
        Vector3 testPoint(0, 0, 0);
        EXPECT_TRUE(tiny.Contains(testPoint));
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(TriangleTest, Performance_ManyContainmentTests_CompletesQuickly)
    {
        const int numTests = 10000;
        int containmentCount = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 testPoint(
                (float)(i % 200) / 100.0f - 1.0f,
                (float)((i * 7) % 200) / 100.0f - 1.0f,
                0.0f
            );
            
            if (standardTriangle.Contains(testPoint))
            {
                containmentCount++;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 50);  // Less than 50ms
        EXPECT_GT(containmentCount, 0);   // Some points should be contained
    }

    TEST_F(TriangleTest, Performance_ManyBarycentricCalculations_CompletesQuickly)
    {
        const int numTests = 10000;
        Vector3 totalBary(0, 0, 0);
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 testPoint(
                (float)(i % 100) / 50.0f - 1.0f,
                (float)((i * 7) % 100) / 50.0f - 1.0f,
                0.0f
            );
            
            Vector3 bary = standardTriangle.Barycentric(testPoint);
            totalBary = totalBary + bary;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        EXPECT_LT(duration.count(), 50);  // Less than 50ms
        EXPECT_GT(totalBary.Magnitude(), 0.0f);  // Some meaningful calculations
    }
}