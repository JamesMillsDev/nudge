#include <gtest/gtest.h>
#include <chrono>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/Line.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

namespace Nudge
{
    /**
     * @brief Test fixture for Line segment unit tests
     * 
     * Provides common test data and utility methods for Line testing.
     * Sets up standard test cases including various line configurations
     * and target geometries for intersection testing.
     */
    class LineTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Standard unit cube centered at origin for testing
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
        
            // Standard line segments for testing
            horizontalLine = Line(Vector3(-2, 0, 0), Vector3(2, 0, 0));
            verticalLine = Line(Vector3(0, -2, 0), Vector3(0, 2, 0));
            depthLine = Line(Vector3(0, 0, -2), Vector3(0, 0, 2));
            diagonalLine = Line(Vector3(-2, -2, -2), Vector3(2, 2, 2));
        
            // Lines with different configurations
            insideLine = Line(Vector3(-0.5f, 0, 0), Vector3(0.5f, 0, 0));
            outsideLine = Line(Vector3(3, 3, 3), Vector3(4, 4, 4));
            touchingLine = Line(Vector3(-2, 1, 0), Vector3(0, 1, 0));
        
            // Other test geometries
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            testPlane = Plane(Vector3(1, 0, 0), 0);  // YZ plane through origin
            testTriangle = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
        }

        // Test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Plane testPlane;
        Triangle testTriangle;
    
        // Test line segments
        Line horizontalLine;
        Line verticalLine;
        Line depthLine;
        Line diagonalLine;
        Line insideLine;
        Line outsideLine;
        Line touchingLine;
    
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // Constructor Tests
    // ============================================================================

    TEST_F(LineTest, DefaultConstructor_InitializesYAxisLine)
    {
        Line line;
    
        EXPECT_EQ(line.start, Vector3(0, 0, 0));
        EXPECT_EQ(line.end, Vector3(0, 1, 0));  // Y-axis line as documented
    }

    TEST_F(LineTest, ParameterizedConstructor_SetsStartAndEnd)
    {
        Vector3 start(1, 2, 3);
        Vector3 end(4, 5, 6);
        Line line(start, end);
    
        EXPECT_EQ(line.start, start);
        EXPECT_EQ(line.end, end);
    }

    // ============================================================================
    // Length Calculation Tests
    // ============================================================================

    TEST_F(LineTest, Length_HorizontalLine_ReturnsCorrectLength)
    {
        EXPECT_NEAR(horizontalLine.Length(), 4.0f, EPSILON);
    }

    TEST_F(LineTest, Length_DiagonalLine_ReturnsCorrectLength)
    {
        float expectedLength = sqrt(3.0f) * 4.0f;  // sqrt(4² + 4² + 4²)
        EXPECT_NEAR(diagonalLine.Length(), expectedLength, EPSILON);
    }

    TEST_F(LineTest, Length_ZeroLengthLine_ReturnsZero)
    {
        Line zeroLine(Vector3(1, 1, 1), Vector3(1, 1, 1));
        EXPECT_NEAR(zeroLine.Length(), 0.0f, EPSILON);
    }

    TEST_F(LineTest, LengthSqr_HorizontalLine_ReturnsCorrectSquaredLength)
    {
        EXPECT_NEAR(horizontalLine.LengthSqr(), 16.0f, EPSILON);
    }

    TEST_F(LineTest, LengthSqr_DiagonalLine_ReturnsCorrectSquaredLength)
    {
        float expectedLengthSqr = 48.0f;  // 4² + 4² + 4²
        EXPECT_NEAR(diagonalLine.LengthSqr(), expectedLengthSqr, EPSILON);
    }

    TEST_F(LineTest, LengthSqr_ZeroLengthLine_ReturnsZero)
    {
        Line zeroLine(Vector3(2, 3, 4), Vector3(2, 3, 4));
        EXPECT_NEAR(zeroLine.LengthSqr(), 0.0f, EPSILON);
    }

    // ============================================================================
    // Point Containment Tests
    // ============================================================================

    TEST_F(LineTest, Contains_PointAtStartEnd_ReturnsTrue)
    {
        EXPECT_TRUE(horizontalLine.Contains(horizontalLine.start));
        EXPECT_TRUE(horizontalLine.Contains(horizontalLine.end));
    }

    TEST_F(LineTest, Contains_PointAtMidpoint_ReturnsTrue)
    {
        Vector3 midpoint = (horizontalLine.start + horizontalLine.end) * 0.5f;
        EXPECT_TRUE(horizontalLine.Contains(midpoint));
    }

    TEST_F(LineTest, Contains_PointOnLine_ReturnsTrue)
    {
        Vector3 onLine = horizontalLine.start * 0.75f + horizontalLine.end * 0.25f;
        EXPECT_TRUE(horizontalLine.Contains(onLine));
    }

    TEST_F(LineTest, Contains_PointOffLine_ReturnsFalse)
    {
        Vector3 offLine(0, 1, 0);  // Perpendicular to horizontal line
        EXPECT_FALSE(horizontalLine.Contains(offLine));
    }

    TEST_F(LineTest, Contains_PointOnLineExtension_ReturnsFalse)
    {
        Vector3 extended = horizontalLine.end + Vector3(1, 0, 0);  // Beyond end
        EXPECT_FALSE(horizontalLine.Contains(extended));
    }

    TEST_F(LineTest, Contains_DegenerateLine_OnlyContainsExactPoint)
    {
        Line pointLine(Vector3(1, 2, 3), Vector3(1, 2, 3));
        
        EXPECT_TRUE(pointLine.Contains(Vector3(1, 2, 3)));
        EXPECT_FALSE(pointLine.Contains(Vector3(1.001f, 2, 3)));
    }

    // ============================================================================
    // Closest Point Tests
    // ============================================================================

    TEST_F(LineTest, ClosestPoint_PointOnLine_ReturnsSamePoint)
    {
        Vector3 onLine = (horizontalLine.start + horizontalLine.end) * 0.5f;
        Vector3 closest = horizontalLine.ClosestPoint(onLine);
        
        EXPECT_NEAR(closest.x, onLine.x, EPSILON);
        EXPECT_NEAR(closest.y, onLine.y, EPSILON);
        EXPECT_NEAR(closest.z, onLine.z, EPSILON);
    }

    TEST_F(LineTest, ClosestPoint_PointNearStart_ReturnsStart)
    {
        Vector3 nearStart = horizontalLine.start + Vector3(0, 1, 0);  // Perpendicular to start
        Vector3 closest = horizontalLine.ClosestPoint(nearStart);
        
        EXPECT_NEAR(closest.x, horizontalLine.start.x, EPSILON);
        EXPECT_NEAR(closest.y, horizontalLine.start.y, EPSILON);
        EXPECT_NEAR(closest.z, horizontalLine.start.z, EPSILON);
    }

    TEST_F(LineTest, ClosestPoint_PointNearEnd_ReturnsEnd)
    {
        Vector3 nearEnd = horizontalLine.end + Vector3(0, 1, 0);  // Perpendicular to end
        Vector3 closest = horizontalLine.ClosestPoint(nearEnd);
        
        EXPECT_NEAR(closest.x, horizontalLine.end.x, EPSILON);
        EXPECT_NEAR(closest.y, horizontalLine.end.y, EPSILON);
        EXPECT_NEAR(closest.z, horizontalLine.end.z, EPSILON);
    }

    TEST_F(LineTest, ClosestPoint_PointPerpendicularToMiddle_ReturnsMiddlePoint)
    {
        Vector3 middle = (horizontalLine.start + horizontalLine.end) * 0.5f;
        Vector3 perpendicular = middle + Vector3(0, 5, 0);
        Vector3 closest = horizontalLine.ClosestPoint(perpendicular);
        
        EXPECT_NEAR(closest.x, middle.x, EPSILON);
        EXPECT_NEAR(closest.y, middle.y, EPSILON);
        EXPECT_NEAR(closest.z, middle.z, EPSILON);
    }

    TEST_F(LineTest, ClosestPoint_DegenerateLine_ReturnsStartPoint)
    {
        Line pointLine(Vector3(1, 2, 3), Vector3(1, 2, 3));
        Vector3 testPoint(5, 5, 5);
        Vector3 closest = pointLine.ClosestPoint(testPoint);
        
        EXPECT_EQ(closest, Vector3(1, 2, 3));
    }

    // ============================================================================
    // Line-AABB Intersection Tests
    // ============================================================================

    TEST_F(LineTest, TestAABB_SegmentCrossingBox_ReturnsTrue)
    {
        EXPECT_TRUE(horizontalLine.Test(unitCube));
        EXPECT_TRUE(verticalLine.Test(unitCube));
        EXPECT_TRUE(depthLine.Test(unitCube));
        EXPECT_TRUE(diagonalLine.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentInsideBox_ReturnsTrue)
    {
        EXPECT_TRUE(insideLine.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentStartingInside_ReturnsTrue)
    {
        Line startInside(Vector3(0, 0, 0), Vector3(2, 0, 0));
        EXPECT_TRUE(startInside.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentEndingInside_ReturnsTrue)
    {
        Line endInside(Vector3(-2, 0, 0), Vector3(0, 0, 0));
        EXPECT_TRUE(endInside.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentMissingBox_ReturnsFalse)
    {
        Line missLine(Vector3(-3, 3, 0), Vector3(-2, 3, 0));
        EXPECT_FALSE(missLine.Test(unitCube));
    
        EXPECT_FALSE(outsideLine.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentTooShort_ReturnsFalse)
    {
        Line shortLine(Vector3(-2, 0, 0), Vector3(-1.5f, 0, 0));
        EXPECT_FALSE(shortLine.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_SegmentTouchingBox_ReturnsTrue)
    {
        EXPECT_TRUE(touchingLine.Test(unitCube));
    
        Line touchingCorner(Vector3(0, 0, 0), Vector3(1, 1, 1));
        EXPECT_TRUE(touchingCorner.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_DegenerateLineInsideBox_ReturnsTrue)
    {
        Line pointInside(Vector3(0, 0, 0), Vector3(0, 0, 0));
        EXPECT_TRUE(pointInside.Test(unitCube));
    }

    TEST_F(LineTest, TestAABB_DegenerateLineOutsideBox_ReturnsFalse)
    {
        Line pointOutside(Vector3(5, 5, 5), Vector3(5, 5, 5));
        EXPECT_FALSE(pointOutside.Test(unitCube));
    }

    // ============================================================================
    // Line-Sphere Intersection Tests
    // ============================================================================

    TEST_F(LineTest, TestSphere_SegmentThroughCenter_ReturnsTrue)
    {
        EXPECT_TRUE(horizontalLine.Test(unitSphere));
        EXPECT_TRUE(verticalLine.Test(unitSphere));
        EXPECT_TRUE(depthLine.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_SegmentInsideSphere_ReturnsTrue)
    {
        Line insideSphere(Vector3(-0.5f, 0, 0), Vector3(0.5f, 0, 0));
        EXPECT_TRUE(insideSphere.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_SegmentMissingSphere_ReturnsFalse)
    {
        Line missLine(Vector3(-3, 2, 0), Vector3(-2, 2, 0));
        EXPECT_FALSE(missLine.Test(unitSphere));
    
        EXPECT_FALSE(outsideLine.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_SegmentTangentToSphere_ReturnsTrue)
    {
        Line tangentLine(Vector3(-2, 1, 0), Vector3(2, 1, 0));
        EXPECT_TRUE(tangentLine.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_SegmentTooShort_ReturnsFalse)
    {
        Line shortLine(Vector3(-2, 0, 0), Vector3(-1.5f, 0, 0));
        EXPECT_FALSE(shortLine.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_SegmentStartingInside_ReturnsTrue)
    {
        Line startInside(Vector3(0, 0, 0), Vector3(2, 0, 0));
        EXPECT_TRUE(startInside.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_DegenerateLineInsideSphere_ReturnsTrue)
    {
        Line pointInside(Vector3(0.5f, 0, 0), Vector3(0.5f, 0, 0));
        EXPECT_TRUE(pointInside.Test(unitSphere));
    }

    TEST_F(LineTest, TestSphere_DegenerateLineOutsideSphere_ReturnsFalse)
    {
        Line pointOutside(Vector3(5, 0, 0), Vector3(5, 0, 0));
        EXPECT_FALSE(pointOutside.Test(unitSphere));
    }

    // ============================================================================
    // Line-Plane Intersection Tests
    // ============================================================================

    TEST_F(LineTest, TestPlane_SegmentCrossingPlane_ReturnsTrue)
    {
        EXPECT_TRUE(horizontalLine.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_SegmentOnPlane_ReturnsTrue)
    {
        Line onPlane(Vector3(0, -1, -1), Vector3(0, 1, 1));
        EXPECT_TRUE(onPlane.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_SegmentParallelToPlane_ReturnsFalse)
    {
        Line parallelLine(Vector3(1, -1, 0), Vector3(1, 1, 0));
        EXPECT_FALSE(parallelLine.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_SegmentTouchingPlane_ReturnsTrue)
    {
        Line touchingPlane(Vector3(-1, 0, 0), Vector3(0, 0, 0));
        EXPECT_TRUE(touchingPlane.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_SegmentNotReachingPlane_ReturnsFalse)
    {
        Line shortLine(Vector3(-2, 0, 0), Vector3(-0.5f, 0, 0));
        EXPECT_FALSE(shortLine.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_DegenerateLineOnPlane_ReturnsTrue)
    {
        Line pointOnPlane(Vector3(0, 1, 1), Vector3(0, 1, 1));
        EXPECT_TRUE(pointOnPlane.Test(testPlane));
    }

    TEST_F(LineTest, TestPlane_DegenerateLineOffPlane_ReturnsFalse)
    {
        Line pointOffPlane(Vector3(1, 1, 1), Vector3(1, 1, 1));
        EXPECT_FALSE(pointOffPlane.Test(testPlane));
    }

    // ============================================================================
    // Line-Triangle Intersection Tests
    // ============================================================================

    TEST_F(LineTest, TestTriangle_SegmentThroughTriangleCenter_ReturnsTrue)
    {
        Line centerLine(Vector3(0, 0, -1), Vector3(0, 0, 1));
        EXPECT_TRUE(centerLine.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_SegmentMissingTriangle_ReturnsFalse)
    {
        Line missLine(Vector3(2, 0, -1), Vector3(2, 0, 1));
        EXPECT_FALSE(missLine.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_SegmentHittingTriangleEdge_ReturnsTrue)
    {
        Line edgeLine(Vector3(0, -1, -1), Vector3(0, -1, 1));
        EXPECT_TRUE(edgeLine.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_SegmentHittingTriangleVertex_ReturnsTrue)
    {
        Line vertexLine(Vector3(-1, -1, -1), Vector3(-1, -1, 1));
        EXPECT_TRUE(vertexLine.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_SegmentOnTrianglePlane_ReturnsTrue)
    {
        Line onTriangle(Vector3(-0.5f, -0.5f, 0), Vector3(0.5f, 0.5f, 0));
        EXPECT_TRUE(onTriangle.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_SegmentTooShort_ReturnsFalse)
    {
        Line shortLine(Vector3(0, 0, -0.5f), Vector3(0, 0, -0.1f));
        EXPECT_FALSE(shortLine.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_DegenerateLineInsideTriangle_ReturnsTrue)
    {
        Line pointInTriangle(Vector3(0, -0.5f, 0), Vector3(0, -0.5f, 0));
        EXPECT_TRUE(pointInTriangle.Test(testTriangle));
    }

    TEST_F(LineTest, TestTriangle_DegenerateLineOutsideTriangle_ReturnsFalse)
    {
        Line pointOutside(Vector3(5, 5, 0), Vector3(5, 5, 0));
        EXPECT_FALSE(pointOutside.Test(testTriangle));
    }

    // ============================================================================
    // Line-OBB Intersection Tests
    // ============================================================================

    TEST_F(LineTest, TestOBB_SegmentIntersectingAlignedOBB_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb unitOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
    
        EXPECT_TRUE(horizontalLine.Test(unitOBB));
        EXPECT_TRUE(verticalLine.Test(unitOBB));
        EXPECT_TRUE(insideLine.Test(unitOBB));
    }

    TEST_F(LineTest, TestOBB_SegmentMissingOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb unitOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
    
        EXPECT_FALSE(outsideLine.Test(unitOBB));
    }

    TEST_F(LineTest, TestOBB_RotatedOBB_WorksCorrectly)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
    
        Line throughRotated(Vector3(-2, 0, 0), Vector3(2, 0, 0));
        EXPECT_TRUE(throughRotated.Test(rotatedOBB));
    }

    TEST_F(LineTest, TestOBB_DegenerateLineInsideOBB_ReturnsTrue)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb unitOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        
        Line pointInside(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f));
        EXPECT_TRUE(pointInside.Test(unitOBB));
    }

    TEST_F(LineTest, TestOBB_DegenerateLineOutsideOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb unitOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        
        Line pointOutside(Vector3(5, 5, 5), Vector3(5, 5, 5));
        EXPECT_FALSE(pointOutside.Test(unitOBB));
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(LineTest, EdgeCases_VeryLongSegment_MaintainsPrecision)
    {
        Line longLine(Vector3(-1e6f, 0, 0), Vector3(1e6f, 0, 0));
    
        EXPECT_TRUE(longLine.Test(unitCube));
        EXPECT_TRUE(longLine.Test(unitSphere));
    }

    TEST_F(LineTest, EdgeCases_VeryShortSegment_WorksCorrectly)
    {
        Line tinyLine(Vector3(0, 0, 0), Vector3(1e-6f, 0, 0));
    
        EXPECT_TRUE(tinyLine.Test(unitCube));
        EXPECT_TRUE(tinyLine.Test(unitSphere));
    }

    TEST_F(LineTest, EdgeCases_SegmentReversed_GivesSameResult)
    {
        Line forward(Vector3(-2, 0, 0), Vector3(2, 0, 0));
        Line backward(Vector3(2, 0, 0), Vector3(-2, 0, 0));
    
        EXPECT_EQ(forward.Test(unitCube), backward.Test(unitCube));
        EXPECT_EQ(forward.Test(unitSphere), backward.Test(unitSphere));
        EXPECT_EQ(forward.Test(testPlane), backward.Test(testPlane));
        EXPECT_EQ(forward.Test(testTriangle), backward.Test(testTriangle));
    }

    TEST_F(LineTest, EdgeCases_SegmentOnGeometryBoundary_HandledCorrectly)
    {
        // Segment exactly on cube face
        Line onCubeFace(Vector3(-1, 0, 1), Vector3(1, 0, 1));
        EXPECT_TRUE(onCubeFace.Test(unitCube));

        // Segment exactly on sphere surface
        Line onSphereSurface(Vector3(-1, 0, 0), Vector3(1, 0, 0));
        EXPECT_TRUE(onSphereSurface.Test(unitSphere));
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(LineTest, Performance_ManyLineTests_CompletesQuickly)
    {
        const int numTests = 10000;
        int hitCount = 0;
    
        auto start = std::chrono::high_resolution_clock::now();
    
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomStart(
                (float)(i % 200) / 100.0f - 1.0f,
                (float)((i * 7) % 200) / 100.0f - 1.0f,
                -2.0f
            );
        
            Vector3 randomEnd = randomStart + Vector3(0, 0, 4);
            Line testLine(randomStart, randomEnd);
        
            if (testLine.Test(unitCube))
            {
                hitCount++;
            }
        }
    
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
        EXPECT_LT(duration.count(), 50);  // Less than 50ms
        EXPECT_GT(hitCount, 0);           // Some hits should occur
    }

    TEST_F(LineTest, Performance_LengthCalculations_OptimizedCorrectly)
    {
        const int numTests = 100000;
        float totalLength = 0.0f;
        float totalLengthSqr = 0.0f;
    
        auto start = std::chrono::high_resolution_clock::now();
    
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomStart((float)(i % 100), (float)((i * 7) % 100), 0);
            Vector3 randomEnd = randomStart + Vector3(1, 1, 1);
            Line testLine(randomStart, randomEnd);
        
            totalLengthSqr += testLine.LengthSqr();  // Should be faster
            if (i % 10 == 0)  // Only calculate expensive sqrt occasionally
            {
                totalLength += testLine.Length();
            }
        }
    
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
        EXPECT_LT(duration.count(), 100);  // Should be fast due to LengthSqr optimization
        EXPECT_GT(totalLengthSqr, 0.0f);
        EXPECT_GT(totalLength, 0.0f);
    }
}