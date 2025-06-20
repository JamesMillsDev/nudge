#pragma once

#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Shapes/AABB.hpp"

namespace Nudge
{
    class Mesh;

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
    };
}