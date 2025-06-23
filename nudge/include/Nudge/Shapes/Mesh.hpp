#pragma once

#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/AABB.hpp"

namespace Nudge
{
    class Aabb;
    class Mesh;
    class Obb;
    class Plane;
    class Sphere;

    /**
     * @brief Node in a Bounding Volume Hierarchy (BVH) tree for spatial acceleration
     *
     * A BvhNode represents either:
     * - Internal node: Contains children nodes and spatial bounds (no triangle data)
     * - Leaf node: Contains triangle indices and spatial bounds (no children)
     *
     * The BVH uses octree subdivision (8 children per internal node) to hierarchically
     * organize triangles for fast spatial queries like ray intersection and collision detection.
     *
     * Memory Layout:
     * - Internal nodes: bounds + children array, triangles = nullptr
     * - Leaf nodes: bounds + triangle indices, children = nullptr
     */
    class BvhNode
    {
    public:
        Aabb bounds;        ///< Axis-aligned bounding box containing all geometry in this node
        BvhNode* children;  ///< Array of 8 child nodes (nullptr for leaf nodes)
        int numTriangles;   ///< Number of triangles in this node (0 for internal nodes)
        int* triangles;     ///< Array of triangle indices referencing parent mesh (nullptr for internal nodes)

    public:
        /**
         * @brief Default constructor initializing empty BVH node
         *
         * Creates a node with no children, no triangles, and undefined bounds.
         * The bounds must be set separately before use.
         */
        BvhNode();

    public:
        /**
         * @brief Recursively subdivides this node using octree spatial partitioning
         * @param mesh Parent mesh containing triangle data for intersection testing
         * @param depth Maximum recursion depth remaining (decremented each level)
         *
         * Subdivides the current node into 8 octant children and distributes triangles
         * based on triangle-AABB intersection tests. Continues recursively until:
         * - Maximum depth is reached, or
         * - Node contains no triangles
         *
         * After subdivision, this node becomes internal (triangles moved to children).
         *
         * @note Triangles may exist in multiple children if they span octant boundaries
         * @see BVH_CHILD_COUNT constant for octree configuration
         */
        void Split(Mesh* mesh, int depth);

        /**
         * @brief Recursively deallocates all memory in this BVH subtree
         *
         * Performs depth-first cleanup of:
         * - All child nodes and their subtrees
         * - Triangle index arrays in leaf nodes
         * - Child node arrays in internal nodes
         *
         * CRITICAL: Must be called before destroying BvhNode to prevent memory leaks.
         * The destructor does not automatically call Free() to allow for controlled cleanup.
         *
         * @warning After calling Free(), this node is in an invalid state and should not be used
         */
        void Free();
    };

    /**
     * @brief Triangle mesh with optional BVH acceleration structure
     *
     * Represents a collection of triangles that can be spatially organized using
     * a Bounding Volume Hierarchy for efficient geometric queries. The mesh supports
     * multiple data access patterns through a union:
     *
     * - Triangle access: mesh.triangles[i] for semantic triangle operations
     * - Vertex access: mesh.vertices[i] for vertex-based operations
     * - Raw float access: mesh.values[i] for low-level data manipulation
     *
     * The BVH acceleration structure is built on-demand via Accelerate() and provides
     * logarithmic-time spatial queries for collision detection and ray tracing.
     */
    class Mesh
    {
    public:
        int numTriangles;   ///< Number of triangles in the mesh

        /**
         * @brief Union providing multiple access patterns for mesh geometry data
         *
         * The same memory can be accessed as:
         * - triangles: Array of Triangle objects (numTriangles elements)
         * - vertices: Array of Vector3 vertices (numTriangles * 3 elements)
         * - values: Raw float array for all coordinate data (numTriangles * 9 elements)
         *
         * Memory Layout: [T0.a.x, T0.a.y, T0.a.z, T0.b.x, T0.b.y, T0.b.z, T0.c.x, T0.c.y, T0.c.z, T1.a.x, ...]
         *
         * @warning Ensure data is properly initialized before accessing through different union members
         */
        union
        {
            Triangle* triangles;  ///< Triangle-based access: mesh.triangles[i] 
            Vector3* vertices;    ///< Vertex-based access: mesh.vertices[i] (3 vertices per triangle)
            float* values;        ///< Raw float access: mesh.values[i] (9 floats per triangle)
        };

        BvhNode* accelerator;   ///< Root of BVH tree (nullptr until Accelerate() is called)

    public:
        /**
         * @brief Default constructor for empty mesh
         *
         * Initializes mesh with no triangles and no acceleration structure.
         * Geometry data must be allocated and assigned separately.
         */
        Mesh();

    public:
        /**
         * @brief Builds BVH acceleration structure for spatial queries
         *
         * Creates an octree-based Bounding Volume Hierarchy to accelerate:
         * - Ray-mesh intersection testing
         * - Collision detection queries
         * - Spatial proximity searches
         * - Frustum culling operations
         *
         * The BVH is built with the following characteristics:
         * - Octree subdivision (8 children per internal node)
         * - Fixed maximum depth (configurable in implementation)
         * - Triangle-AABB intersection for spatial partitioning
         * - On-demand construction (idempotent - safe to call multiple times)
         *
         * @note This operation has O(n * log(n) * depth) complexity where n = numTriangles
         * @note Memory usage increases due to triangle indices stored in multiple nodes
         * @see BvhNode::Split() for subdivision algorithm details
         * @see BvhNode::Free() for cleanup when mesh is destroyed
         */
        void Accelerate();

        /**
		 * @brief Tests if the mesh intersects with an Axis-Aligned Bounding Box
		 * @param other AABB to test intersection against
		 * @return True if any triangle in the mesh intersects or is contained within the AABB
		 *
		 * This method performs mesh-AABB intersection testing using the BVH acceleration
		 * structure if available, otherwise falls back to brute-force triangle iteration.
		 *
		 * Algorithm with BVH:
		 * 1. Traverse BVH tree, testing AABB against node bounds
		 * 2. Skip entire subtrees when node bounds don't intersect the query AABB
		 * 3. Test individual triangles only in leaf nodes that intersect
		 *
		 * Algorithm without BVH:
		 * 1. Iterate through all mesh triangles
		 * 2. Test each triangle against the AABB using SAT-based intersection
		 *
		 * @note Performance scales from O(n) to O(log n) average case with BVH acceleration
		 * @note Call Accelerate() first to build BVH for optimal performance on large meshes
		 * @see Triangle::Intersects(const Aabb&) for triangle-level intersection testing
		 * @see BvhNode for spatial acceleration structure details
		 */
        bool Intersects(const Aabb& other) const;

        /**
         * @brief Tests if the mesh intersects with an Oriented Bounding Box
         * @param other OBB to test intersection against
         * @return True if any triangle in the mesh intersects or is contained within the OBB
         *
         * Similar to AABB intersection but handles arbitrarily oriented bounding boxes.
         * The BVH traversal uses conservative AABB-OBB intersection tests to prune
         * the search space before performing precise triangle-OBB intersection tests.
         *
         * Algorithm with BVH:
         * 1. Traverse BVH using AABB-OBB intersection for node pruning
         * 2. Test triangles in intersecting leaf nodes against the OBB
         * 3. Early exit on first intersection found
         *
         * @note More expensive than AABB intersection due to arbitrary OBB orientation
         * @note BVH acceleration provides significant speedup for complex meshes
         * @see Triangle::Intersects(const Obb&) for triangle-OBB SAT testing
         * @see Interval::TriangleObb() for the underlying SAT implementation
         */
        bool Intersects(const Obb& other) const;

        /**
         * @brief Tests if the mesh intersects with a plane
         * @param other Plane to test intersection against
         * @return True if any triangle in the mesh crosses, touches, or lies on the plane
         *
         * Plane intersection testing determines if the mesh is split by the plane.
         * A mesh intersects a plane if any triangle has vertices on both sides of
         * the plane or if any triangle lies exactly on the plane.
         *
         * Algorithm with BVH:
         * 1. Traverse BVH testing node bounds against plane
         * 2. A node intersects if its AABB spans both sides of the plane
         * 3. Test triangles in intersecting leaf nodes for plane intersection
         *
         * Algorithm without BVH:
         * 1. Iterate all triangles testing each against the plane
         * 2. Use signed distance tests for triangle vertices
         *
         * @note Useful for frustum culling, CSG operations, and spatial partitioning
         * @note BVH pruning is highly effective for plane tests due to spatial coherence
         * @see Triangle::Intersects(const Plane&) for triangle-plane intersection
         * @see Plane class for signed distance calculations
         */
        bool Intersects(const Plane& other) const;

        /**
         * @brief Tests if the mesh intersects with a sphere
         * @param other Sphere to test intersection against
         * @return True if any triangle in the mesh intersects or is contained within the sphere
         *
         * Sphere intersection testing checks if any part of the mesh lies within
         * the spherical volume. This includes triangles that are fully contained,
         * partially intersecting, or touching the sphere boundary.
         *
         * Algorithm with BVH:
         * 1. Traverse BVH using AABB-sphere intersection for node pruning
         * 2. Test triangles in intersecting leaf nodes against sphere
         * 3. Early termination on first intersection found
         *
         * Triangle-sphere testing:
         * 1. Find closest point on triangle to sphere center
         * 2. Check if distance to closest point <= sphere radius
         *
         * @note Efficient for collision detection, proximity queries, and culling
         * @note BVH acceleration essential for real-time performance on detailed meshes
         * @see Triangle::Intersects(const Sphere&) for triangle-sphere testing
         * @see Triangle::ClosestPoint() for point-triangle distance calculations
         */
        bool Intersects(const Sphere& other) const;

    };
}