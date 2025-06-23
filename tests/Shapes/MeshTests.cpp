#include <gtest/gtest.h>
#include <chrono>
#include <memory>

#include "Nudge/Shapes/Mesh.hpp"
#include "Nudge/Shapes/Triangle.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/MathF.hpp"

namespace Nudge
{
    /**
     * @brief Test fixture for Mesh and BvhNode unit tests
     * 
     * Provides common test data and utility methods for Mesh testing.
     * Sets up standard test cases including simple meshes, complex meshes,
     * and various geometric primitives for intersection testing.
     */
    class MeshTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Create simple triangle mesh (single triangle)
            CreateSimpleMesh();
            
            // Create cube mesh (12 triangles forming a unit cube)
            CreateCubeMesh();
            
            // Create pyramid mesh (6 triangles forming a pyramid)
            CreatePyramidMesh();
            
            // Create large mesh for performance testing
            CreateLargeMesh();
            
            // Test geometries for intersection testing
            unitCube = Aabb(Vector3(0, 0, 0), Vector3(1, 1, 1));
            unitSphere = Sphere(Vector3(0, 0, 0), 1.0f);
            testPlane = Plane(Vector3(1, 0, 0), 0);  // YZ plane through origin
            
            Matrix3 identity = Matrix3::Identity();
            unitOBB = Obb(Vector3(0, 0, 0), Vector3(1, 1, 1), identity);
        }

        void TearDown() override
        {
            // Clean up mesh data
            CleanupMesh(simpleMesh);
            CleanupMesh(cubeMesh);
            CleanupMesh(pyramidMesh);
            CleanupMesh(largeMesh);
        }

        void CreateSimpleMesh()
        {
            simpleMesh.numTriangles = 1;
            simpleMesh.triangles = new Triangle[1];
            simpleMesh.triangles[0] = Triangle(
                Vector3(-1, -1, 0),
                Vector3(1, -1, 0),
                Vector3(0, 1, 0)
            );
            simpleMesh.accelerator = nullptr;
        }

        void CreateCubeMesh()
        {
            cubeMesh.numTriangles = 12;  // 6 faces * 2 triangles per face
            cubeMesh.triangles = new Triangle[12];
            
            // Front face (z = 1)
            cubeMesh.triangles[0] = Triangle(Vector3(-1, -1, 1), Vector3(1, -1, 1), Vector3(1, 1, 1));
            cubeMesh.triangles[1] = Triangle(Vector3(-1, -1, 1), Vector3(1, 1, 1), Vector3(-1, 1, 1));
            
            // Back face (z = -1)
            cubeMesh.triangles[2] = Triangle(Vector3(-1, -1, -1), Vector3(-1, 1, -1), Vector3(1, 1, -1));
            cubeMesh.triangles[3] = Triangle(Vector3(-1, -1, -1), Vector3(1, 1, -1), Vector3(1, -1, -1));
            
            // Left face (x = -1)
            cubeMesh.triangles[4] = Triangle(Vector3(-1, -1, -1), Vector3(-1, -1, 1), Vector3(-1, 1, 1));
            cubeMesh.triangles[5] = Triangle(Vector3(-1, -1, -1), Vector3(-1, 1, 1), Vector3(-1, 1, -1));
            
            // Right face (x = 1)
            cubeMesh.triangles[6] = Triangle(Vector3(1, -1, -1), Vector3(1, 1, 1), Vector3(1, -1, 1));
            cubeMesh.triangles[7] = Triangle(Vector3(1, -1, -1), Vector3(1, 1, -1), Vector3(1, 1, 1));
            
            // Bottom face (y = -1)
            cubeMesh.triangles[8] = Triangle(Vector3(-1, -1, -1), Vector3(1, -1, -1), Vector3(1, -1, 1));
            cubeMesh.triangles[9] = Triangle(Vector3(-1, -1, -1), Vector3(1, -1, 1), Vector3(-1, -1, 1));
            
            // Top face (y = 1)
            cubeMesh.triangles[10] = Triangle(Vector3(-1, 1, -1), Vector3(-1, 1, 1), Vector3(1, 1, 1));
            cubeMesh.triangles[11] = Triangle(Vector3(-1, 1, -1), Vector3(1, 1, 1), Vector3(1, 1, -1));
            
            cubeMesh.accelerator = nullptr;
        }

        void CreatePyramidMesh()
        {
            pyramidMesh.numTriangles = 6;  // 4 sides + 2 triangles for square base
            pyramidMesh.triangles = new Triangle[6];
            
            Vector3 apex(0, 0, 2);
            Vector3 base1(-1, -1, 0);
            Vector3 base2(1, -1, 0);
            Vector3 base3(1, 1, 0);
            Vector3 base4(-1, 1, 0);
            
            // Side faces
            pyramidMesh.triangles[0] = Triangle(base1, base2, apex);
            pyramidMesh.triangles[1] = Triangle(base2, base3, apex);
            pyramidMesh.triangles[2] = Triangle(base3, base4, apex);
            pyramidMesh.triangles[3] = Triangle(base4, base1, apex);
            
            // Base triangles
            pyramidMesh.triangles[4] = Triangle(base1, base3, base2);
            pyramidMesh.triangles[5] = Triangle(base1, base4, base3);
            
            pyramidMesh.accelerator = nullptr;
        }

        void CreateLargeMesh()
        {
            // Create a mesh with many triangles for performance testing
            const int gridSize = 20;  // 20x20 grid
            largeMesh.numTriangles = gridSize * gridSize * 2;  // 2 triangles per grid cell
            largeMesh.triangles = new Triangle[largeMesh.numTriangles];
            
            int triangleIndex = 0;
            for (int x = 0; x < gridSize; ++x)
            {
                for (int z = 0; z < gridSize; ++z)
                {
                    float x1 = (float)x - gridSize/2.0f;
                    float x2 = x1 + 1.0f;
                    float z1 = (float)z - gridSize/2.0f;
                    float z2 = z1 + 1.0f;
                    float y = 0.0f;
                    
                    // Two triangles per grid cell
                    largeMesh.triangles[triangleIndex++] = Triangle(
                        Vector3(x1, y, z1),
                        Vector3(x2, y, z1),
                        Vector3(x2, y, z2)
                    );
                    largeMesh.triangles[triangleIndex++] = Triangle(
                        Vector3(x1, y, z1),
                        Vector3(x2, y, z2),
                        Vector3(x1, y, z2)
                    );
                }
            }
            
            largeMesh.accelerator = nullptr;
        }

        void CleanupMesh(Mesh& mesh)
        {
            if (mesh.accelerator)
            {
                mesh.accelerator->Free();
                delete mesh.accelerator;
                mesh.accelerator = nullptr;
            }
            
            if (mesh.triangles)
            {
                delete[] mesh.triangles;
                mesh.triangles = nullptr;
            }
            
            mesh.numTriangles = 0;
        }

        // Test mesh instances
        Mesh simpleMesh;
        Mesh cubeMesh;
        Mesh pyramidMesh;
        Mesh largeMesh;
        
        // Other test geometries
        Aabb unitCube;
        Sphere unitSphere;
        Plane testPlane;
        Obb unitOBB;
        
        // Tolerance for floating-point comparisons
        static constexpr float EPSILON = 1e-6f;
    };

    // ============================================================================
    // BvhNode Tests
    // ============================================================================

    TEST_F(MeshTest, BvhNode_DefaultConstructor_InitializesCorrectly)
    {
        BvhNode node;
        
        // Node should be initialized with null pointers
        EXPECT_EQ(node.children, nullptr);
        EXPECT_EQ(node.triangles, nullptr);
        EXPECT_EQ(node.numTriangles, 0);
        
        // Bounds are implementation defined for default constructor
        EXPECT_NO_THROW({
            Vector3 min = node.bounds.Min();
            Vector3 max = node.bounds.Max();
        });
    }

    TEST_F(MeshTest, BvhNode_Split_CreatesBVHStructure)
    {
        BvhNode rootNode;
        rootNode.numTriangles = cubeMesh.numTriangles;
        rootNode.triangles = new int[cubeMesh.numTriangles];
        
        // Initialize with all triangle indices
        for (int i = 0; i < cubeMesh.numTriangles; ++i)
        {
            rootNode.triangles[i] = i;
        }
        
        // Set bounds to encompass all triangles
        rootNode.bounds = Aabb(Vector3(-1, -1, -1), Vector3(2, 2, 2));
        
        // Split the node
        rootNode.Split(&cubeMesh, 3);  // 3 levels deep
        
        // After splitting, root should become internal node
        EXPECT_NE(rootNode.children, nullptr);
        EXPECT_EQ(rootNode.triangles, nullptr);  // Moved to children
        EXPECT_EQ(rootNode.numTriangles, 0);
        
        // Cleanup
        rootNode.Free();
    }

    TEST_F(MeshTest, BvhNode_Free_CleansUpMemoryCorrectly)
    {
        BvhNode* node = new BvhNode();
        
        // Set up node with some data
        node->numTriangles = 3;
        node->triangles = new int[3]{0, 1, 2};
        node->children = nullptr;
        
        // Free should clean up the triangles array
        node->Free();
        
        EXPECT_EQ(node->triangles, nullptr);
        EXPECT_EQ(node->numTriangles, 0);
        EXPECT_EQ(node->children, nullptr);
        
        delete node;
    }

    // ============================================================================
    // Mesh Constructor Tests
    // ============================================================================

    TEST_F(MeshTest, Mesh_DefaultConstructor_InitializesCorrectly)
    {
        Mesh mesh;
        
        EXPECT_EQ(mesh.numTriangles, 0);
        EXPECT_EQ(mesh.triangles, nullptr);
        EXPECT_EQ(mesh.vertices, nullptr);
        EXPECT_EQ(mesh.values, nullptr);
        EXPECT_EQ(mesh.accelerator, nullptr);
    }

    // ============================================================================
    // Union Access Pattern Tests
    // ============================================================================

    TEST_F(MeshTest, Mesh_UnionAccess_TrianglesAndVerticesSameMemory)
    {
        // Test that triangles and vertices point to the same memory
        EXPECT_EQ((void*)simpleMesh.triangles, (void*)simpleMesh.vertices);
        EXPECT_EQ((void*)simpleMesh.triangles, (void*)simpleMesh.values);
    }

    TEST_F(MeshTest, Mesh_UnionAccess_VertexAccessWorksCorrectly)
    {
        // Access triangle vertices through vertices array
        Vector3 vertex0 = simpleMesh.vertices[0];  // First vertex of first triangle
        Vector3 vertex1 = simpleMesh.vertices[1];  // Second vertex of first triangle
        Vector3 vertex2 = simpleMesh.vertices[2];  // Third vertex of first triangle
        
        EXPECT_EQ(vertex0, simpleMesh.triangles[0].a);
        EXPECT_EQ(vertex1, simpleMesh.triangles[0].b);
        EXPECT_EQ(vertex2, simpleMesh.triangles[0].c);
    }

    TEST_F(MeshTest, Mesh_UnionAccess_ValueAccessWorksCorrectly)
    {
        // Access raw float components
        EXPECT_EQ(simpleMesh.values[0], simpleMesh.triangles[0].a.x);
        EXPECT_EQ(simpleMesh.values[1], simpleMesh.triangles[0].a.y);
        EXPECT_EQ(simpleMesh.values[2], simpleMesh.triangles[0].a.z);
        
        EXPECT_EQ(simpleMesh.values[3], simpleMesh.triangles[0].b.x);
        EXPECT_EQ(simpleMesh.values[4], simpleMesh.triangles[0].b.y);
        EXPECT_EQ(simpleMesh.values[5], simpleMesh.triangles[0].b.z);
        
        EXPECT_EQ(simpleMesh.values[6], simpleMesh.triangles[0].c.x);
        EXPECT_EQ(simpleMesh.values[7], simpleMesh.triangles[0].c.y);
        EXPECT_EQ(simpleMesh.values[8], simpleMesh.triangles[0].c.z);
    }

    // ============================================================================
    // Acceleration Structure Tests
    // ============================================================================

    TEST_F(MeshTest, Mesh_Accelerate_BuildsBVH)
    {
        EXPECT_EQ(cubeMesh.accelerator, nullptr);
        
        cubeMesh.Accelerate();
        
        EXPECT_NE(cubeMesh.accelerator, nullptr);
        
        // BVH should have valid bounds
        Vector3 min = cubeMesh.accelerator->bounds.Min();
        Vector3 max = cubeMesh.accelerator->bounds.Max();
        
        EXPECT_LE(min.x, max.x);
        EXPECT_LE(min.y, max.y);
        EXPECT_LE(min.z, max.z);
    }

    TEST_F(MeshTest, Mesh_Accelerate_IdempotentOperation)
    {
        cubeMesh.Accelerate();
        BvhNode* firstAccelerator = cubeMesh.accelerator;
        
        cubeMesh.Accelerate();
        
        // Should not recreate accelerator if already exists
        EXPECT_EQ(cubeMesh.accelerator, firstAccelerator);
    }

    TEST_F(MeshTest, Mesh_Accelerate_EmptyMesh_HandledGracefully)
    {
        Mesh emptyMesh;
        emptyMesh.numTriangles = 0;
        emptyMesh.triangles = nullptr;
        emptyMesh.accelerator = nullptr;
        
        EXPECT_NO_THROW({
            emptyMesh.Accelerate();
        });
    }

    // ============================================================================
    // Mesh-AABB Intersection Tests
    // ============================================================================

    TEST_F(MeshTest, IntersectsAABB_SimpleMeshIntersectingAABB_ReturnsTrue)
    {
        EXPECT_TRUE(simpleMesh.Intersects(unitCube));
    }

    TEST_F(MeshTest, IntersectsAABB_CubeMeshIntersectingAABB_ReturnsTrue)
    {
        EXPECT_TRUE(cubeMesh.Intersects(unitCube));
    }

    TEST_F(MeshTest, IntersectsAABB_MeshSeparatedFromAABB_ReturnsFalse)
    {
        Aabb separatedAABB(Vector3(10, 10, 10), Vector3(1, 1, 1));
        EXPECT_FALSE(simpleMesh.Intersects(separatedAABB));
        EXPECT_FALSE(cubeMesh.Intersects(separatedAABB));
    }

    TEST_F(MeshTest, IntersectsAABB_MeshContainedInAABB_ReturnsTrue)
    {
        Aabb largeAABB(Vector3(0, 0, 0), Vector3(10, 10, 10));
        EXPECT_TRUE(simpleMesh.Intersects(largeAABB));
        EXPECT_TRUE(cubeMesh.Intersects(largeAABB));
    }

    TEST_F(MeshTest, IntersectsAABB_WithBVH_SameResultAsWithoutBVH)
    {
        Aabb testAABB(Vector3(0.5f, 0.5f, 0.5f), Vector3(1, 1, 1));
        
        // Test without BVH
        bool resultWithoutBVH = cubeMesh.Intersects(testAABB);
        
        // Build BVH and test again
        cubeMesh.Accelerate();
        bool resultWithBVH = cubeMesh.Intersects(testAABB);
        
        EXPECT_EQ(resultWithoutBVH, resultWithBVH);
    }

    TEST_F(MeshTest, IntersectsAABB_PyramidMesh_WorksCorrectly)
    {
        EXPECT_TRUE(pyramidMesh.Intersects(unitCube));
        
        Aabb highAABB(Vector3(0, 0, 3), Vector3(1, 1, 1));
        EXPECT_FALSE(pyramidMesh.Intersects(highAABB));
    }

    // ============================================================================
    // Mesh-OBB Intersection Tests
    // ============================================================================

    TEST_F(MeshTest, IntersectsOBB_AlignedOBBIntersectingMesh_ReturnsTrue)
    {
        EXPECT_TRUE(simpleMesh.Intersects(unitOBB));
        EXPECT_TRUE(cubeMesh.Intersects(unitOBB));
    }

    TEST_F(MeshTest, IntersectsOBB_RotatedOBBIntersectingMesh_ReturnsTrue)
    {
        Matrix3 rotation = Matrix3::RotationZ(MathF::pi / 4.0f);
        Obb rotatedOBB(Vector3(0, 0, 0), Vector3(1, 1, 1), rotation);
        
        EXPECT_TRUE(cubeMesh.Intersects(rotatedOBB));
    }

    TEST_F(MeshTest, IntersectsOBB_SeparatedOBB_ReturnsFalse)
    {
        Matrix3 identity = Matrix3::Identity();
        Obb separatedOBB(Vector3(10, 10, 10), Vector3(1, 1, 1), identity);
        
        EXPECT_FALSE(simpleMesh.Intersects(separatedOBB));
        EXPECT_FALSE(cubeMesh.Intersects(separatedOBB));
    }

    TEST_F(MeshTest, IntersectsOBB_WithBVH_SameResultAsWithoutBVH)
    {
        Matrix3 rotation = Matrix3::RotationY(MathF::pi / 6.0f);
        Obb testOBB(Vector3(0.5f, 0.5f, 0.5f), Vector3(1, 1, 1), rotation);
        
        // Test without BVH
        bool resultWithoutBVH = cubeMesh.Intersects(testOBB);
        
        // Build BVH and test again
        cubeMesh.Accelerate();
        bool resultWithBVH = cubeMesh.Intersects(testOBB);
        
        EXPECT_EQ(resultWithoutBVH, resultWithBVH);
    }

    // ============================================================================
    // Mesh-Sphere Intersection Tests
    // ============================================================================

    TEST_F(MeshTest, IntersectsSphere_SphereIntersectingMesh_ReturnsTrue)
    {
        EXPECT_TRUE(simpleMesh.Intersects(unitSphere));
        EXPECT_TRUE(cubeMesh.Intersects(unitSphere));
    }

    TEST_F(MeshTest, IntersectsSphere_SphereContainingMesh_ReturnsTrue)
    {
        Sphere largeSphere(Vector3(0, 0, 0), 10.0f);
        EXPECT_TRUE(simpleMesh.Intersects(largeSphere));
        EXPECT_TRUE(cubeMesh.Intersects(largeSphere));
    }

    TEST_F(MeshTest, IntersectsSphere_SeparatedSphere_ReturnsFalse)
    {
        Sphere separatedSphere(Vector3(10, 10, 10), 1.0f);
        EXPECT_FALSE(simpleMesh.Intersects(separatedSphere));
        EXPECT_FALSE(cubeMesh.Intersects(separatedSphere));
    }

    TEST_F(MeshTest, IntersectsSphere_SphereTouchingMesh_ReturnsTrue)
    {
        Sphere touchingSphere(Vector3(2, 0, 0), 1.0f);  // Just touching cube edge
        EXPECT_TRUE(cubeMesh.Intersects(touchingSphere));
    }

    TEST_F(MeshTest, IntersectsSphere_WithBVH_SameResultAsWithoutBVH)
    {
        Sphere testSphere(Vector3(0.5f, 0.5f, 0.5f), 1.5f);
        
        // Test without BVH
        bool resultWithoutBVH = cubeMesh.Intersects(testSphere);
        
        // Build BVH and test again
        cubeMesh.Accelerate();
        bool resultWithBVH = cubeMesh.Intersects(testSphere);
        
        EXPECT_EQ(resultWithoutBVH, resultWithBVH);
    }

    // ============================================================================
    // Mesh-Plane Intersection Tests
    // ============================================================================

    TEST_F(MeshTest, IntersectsPlane_PlaneCrossingMesh_ReturnsTrue)
    {
        EXPECT_TRUE(simpleMesh.Intersects(testPlane));
        EXPECT_TRUE(cubeMesh.Intersects(testPlane));
    }

    TEST_F(MeshTest, IntersectsPlane_PlaneSeparatedFromMesh_ReturnsFalse)
    {
        Plane separatedPlane(Vector3(1, 0, 0), -10);  // Far from mesh
        EXPECT_FALSE(simpleMesh.Intersects(separatedPlane));
        EXPECT_FALSE(cubeMesh.Intersects(separatedPlane));
    }

    TEST_F(MeshTest, IntersectsPlane_PlaneTouchingMesh_ReturnsTrue)
    {
        Plane touchingPlane(Vector3(1, 0, 0), -1);  // Touching cube face
        EXPECT_TRUE(cubeMesh.Intersects(touchingPlane));
    }

    TEST_F(MeshTest, IntersectsPlane_DiagonalPlaneCrossingMesh_ReturnsTrue)
    {
        Plane diagonalPlane(Vector3(1, 1, 1).Normalized(), 0);
        EXPECT_TRUE(cubeMesh.Intersects(diagonalPlane));
    }

    TEST_F(MeshTest, IntersectsPlane_WithBVH_SameResultAsWithoutBVH)
    {
        Plane testPlane2(Vector3(0, 1, 0), -0.5f);
        
        // Test without BVH
        bool resultWithoutBVH = cubeMesh.Intersects(testPlane2);
        
        // Build BVH and test again
        cubeMesh.Accelerate();
        bool resultWithBVH = cubeMesh.Intersects(testPlane2);
        
        EXPECT_EQ(resultWithoutBVH, resultWithBVH);
    }

    // ============================================================================
    // Edge Cases and Robustness Tests
    // ============================================================================

    TEST_F(MeshTest, EdgeCases_EmptyMesh_HandledGracefully)
    {
        Mesh emptyMesh;
        emptyMesh.numTriangles = 0;
        emptyMesh.triangles = nullptr;
        emptyMesh.accelerator = nullptr;
        
        // All intersection tests should return false for empty mesh
        EXPECT_FALSE(emptyMesh.Intersects(unitCube));
        EXPECT_FALSE(emptyMesh.Intersects(unitOBB));
        EXPECT_FALSE(emptyMesh.Intersects(unitSphere));
        EXPECT_FALSE(emptyMesh.Intersects(testPlane));
    }

    TEST_F(MeshTest, EdgeCases_SingleTriangleMesh_WorksCorrectly)
    {
        // Simple mesh has only one triangle
        EXPECT_EQ(simpleMesh.numTriangles, 1);
        
        // Should still work for all intersection types
        EXPECT_TRUE(simpleMesh.Intersects(unitCube));
        EXPECT_TRUE(simpleMesh.Intersects(unitOBB));
        EXPECT_TRUE(simpleMesh.Intersects(unitSphere));
        EXPECT_TRUE(simpleMesh.Intersects(testPlane));
    }

    TEST_F(MeshTest, EdgeCases_LargeMesh_MaintainsCorrectness)
    {
        // Large mesh should still give correct results
        Aabb testAABB(Vector3(0, -1, 0), Vector3(5, 2, 5));
        EXPECT_TRUE(largeMesh.Intersects(testAABB));
        
        Aabb separatedAABB(Vector3(100, 100, 100), Vector3(1, 1, 1));
        EXPECT_FALSE(largeMesh.Intersects(separatedAABB));
    }

    TEST_F(MeshTest, EdgeCases_BVHWithEmptyNode_HandledGracefully)
    {
        // Force BVH creation on simple mesh
        simpleMesh.Accelerate();
        
        // Should still work correctly
        EXPECT_TRUE(simpleMesh.Intersects(unitCube));
        EXPECT_FALSE(simpleMesh.Intersects(Aabb(Vector3(10, 10, 10), Vector3(1, 1, 1))));
    }

    // ============================================================================
    // Performance Tests
    // ============================================================================

    TEST_F(MeshTest, Performance_LargeMeshWithoutBVH_CompletesInReasonableTime)
    {
        const int numTests = 100;
        int intersectionCount = 0;
        
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomCenter(
                (float)(i % 20) - 10.0f,
                0.5f,
                (float)((i * 7) % 20) - 10.0f
            );
            
            Aabb testAABB(randomCenter, Vector3(2, 2, 2));
            if (largeMesh.Intersects(testAABB))
            {
                intersectionCount++;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        // Should complete even without BVH (though slower)
        EXPECT_LT(duration.count(), 1000);  // Less than 1 second
        EXPECT_GT(intersectionCount, 0);    // Some intersections should occur
    }

    TEST_F(MeshTest, Performance_LargeMeshWithBVH_SignificantlyFaster)
    {
        const int numTests = 1000;  // More tests with BVH
        
        // Test without BVH
        auto start1 = std::chrono::high_resolution_clock::now();
        
        int intersectionCount1 = 0;
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomCenter(
                (float)(i % 20) - 10.0f,
                0.5f,
                (float)((i * 7) % 20) - 10.0f
            );
            
            Aabb testAABB(randomCenter, Vector3(2, 2, 2));
            if (largeMesh.Intersects(testAABB))
            {
                intersectionCount1++;
            }
        }
        
        auto end1 = std::chrono::high_resolution_clock::now();
        auto durationWithoutBVH = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
        
        // Build BVH
        largeMesh.Accelerate();
        
        // Test with BVH
        auto start2 = std::chrono::high_resolution_clock::now();
        
        int intersectionCount2 = 0;
        for (int i = 0; i < numTests; ++i)
        {
            Vector3 randomCenter(
                (float)(i % 20) - 10.0f,
                0.5f,
                (float)((i * 7) % 20) - 10.0f
            );
            
            Aabb testAABB(randomCenter, Vector3(2, 2, 2));
            if (largeMesh.Intersects(testAABB))
            {
                intersectionCount2++;
            }
        }
        
        auto end2 = std::chrono::high_resolution_clock::now();
        auto durationWithBVH = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
        
        // Results should be the same
        EXPECT_EQ(intersectionCount1, intersectionCount2);
        
        // BVH should be faster (allow some tolerance for measurement variance)
        EXPECT_LE(durationWithBVH.count(), durationWithoutBVH.count() + 50);
        
        // Both should complete in reasonable time
        EXPECT_LT(durationWithBVH.count(), 500);  // Less than 500ms with BVH
    }

    TEST_F(MeshTest, Performance_BVHConstruction_CompletesQuickly)
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        largeMesh.Accelerate();
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        // BVH construction should be reasonably fast
        EXPECT_LT(duration.count(), 1000);  // Less than 1 second
        EXPECT_NE(largeMesh.accelerator, nullptr);
    }
}