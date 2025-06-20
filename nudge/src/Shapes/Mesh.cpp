#include "Nudge/Shapes/Mesh.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

#include <list>

using std::list;

// Configuration: Use octree subdivision (8 children per node)
// Could be adjusted for different tree structures (binary = 2, quadtree = 4, etc.)
constexpr int BVH_CHILD_COUNT = 8;

namespace Nudge
{
	/**
	 * @brief Default constructor for BVH node
	 *
	 * Initializes node as a leaf with no children or triangles.
	 * All pointers are set to nullptr and counts to zero.
	 */
	BvhNode::BvhNode()
		: children{ nullptr }, numTriangles{ 0 }, triangles{ nullptr }
	{
	}

	/**
	 * @brief Recursively subdivides the BVH node using octree spatial partitioning
	 * @param mesh Pointer to the parent mesh containing triangle data
	 * @param depth Maximum recursion depth remaining (decremented each level)
	 *
	 * Algorithm:
	 * 1. Check termination conditions (depth limit, no triangles)
	 * 2. Create 8 child nodes representing octants of current bounds
	 * 3. Distribute triangles to children based on triangle-AABB intersection
	 * 4. Clear triangle data from current node (becomes internal node)
	 * 5. Recursively subdivide children
	 */
	void BvhNode::Split(Mesh* mesh, int depth)
	{
		// Termination condition: Maximum depth reached
		if (depth-- == 0)
		{
			return;
		}

		// Create children if this node has triangles and no children yet
		if (children == nullptr)
		{
			if (numTriangles > 0)
			{
				children = new BvhNode[BVH_CHILD_COUNT];

				// Calculate octant subdivision parameters
				const Vector3 c = bounds.origin;      // Current node center
				const Vector3 e = bounds.extents * 0.5f;  // Half-extents for children

				// Create 8 octant children with systematic offset pattern
				// Order: [front/back][top/bottom][left/right]
				children[0].bounds = Aabb(c + Vector3(-e.x, +e.y, -e.z), e);  // Front-top-left
				children[1].bounds = Aabb(c + Vector3(+e.x, +e.y, -e.z), e);  // Front-top-right
				children[2].bounds = Aabb(c + Vector3(-e.x, +e.y, +e.z), e);  // Back-top-left
				children[3].bounds = Aabb(c + Vector3(+e.x, +e.y, +e.z), e);  // Back-top-right
				children[4].bounds = Aabb(c + Vector3(-e.x, -e.y, -e.z), e);  // Front-bottom-left
				children[5].bounds = Aabb(c + Vector3(+e.x, -e.y, -e.z), e);  // Front-bottom-right
				children[6].bounds = Aabb(c + Vector3(-e.x, -e.y, +e.z), e);  // Back-bottom-left
				children[7].bounds = Aabb(c + Vector3(+e.x, -e.y, +e.z), e);  // Back-bottom-right
			}
		}

		// Distribute triangles to children if subdivision occurred
		if (children != nullptr && numTriangles > 0)
		{
			// Phase 1: Count triangles per child for memory allocation
			for (int i = 0; i < BVH_CHILD_COUNT; ++i)
			{
				BvhNode& child = children[i];
				children[i].numTriangles = 0;

				// Count triangles that intersect this child's bounding box
				for (int j = 0; j < numTriangles; ++j)
				{
					const Triangle& t = mesh->triangles[triangles[j]];

					if (t.Intersects(child.bounds))
					{
						child.numTriangles++;
					}
				}

				// Skip children with no triangles (optimization)
				if (child.numTriangles == 0)
				{
					continue;
				}

				// Allocate triangle index array for this child
				child.triangles = new int[child.numTriangles];

				// Phase 2: Assign triangle indices to child
				int index = 0;
				for (int j = 0; j < numTriangles; ++j)
				{
					const Triangle& t = mesh->triangles[triangles[j]];

					if (t.Intersects(child.bounds))
					{
						child.triangles[index++] = triangles[j];
					}
				}
			}

			// Convert this node from leaf to internal node
			// Clear triangle data as it's now distributed to children
			numTriangles = 0;
			delete[] triangles;
			triangles = nullptr;

			// Recursively subdivide all children
			for (int i = 0; i < BVH_CHILD_COUNT; ++i)
			{
				children[i].Split(mesh, depth);
			}
		}
	}

	/**
	 * @brief Recursively deallocates all memory associated with this BVH node
	 *
	 * Performs depth-first cleanup of the entire subtree rooted at this node.
	 * Must be called to prevent memory leaks when destroying the BVH.
	 *
	 * IMPORTANT: This must be called before the BvhNode destructor to ensure
	 * proper cleanup of dynamically allocated children and triangle arrays.
	 */
	void BvhNode::Free()
	{
		// Recursively free all children first (depth-first cleanup)
		if (children != nullptr)
		{
			for (int i = 0; i < BVH_CHILD_COUNT; ++i)
			{
				children[i].Free();
			}

			delete[] children;
			children = nullptr;
		}

		// Free triangle index array if this is a leaf node
		if (numTriangles != 0 && triangles != nullptr)
		{
			delete[] triangles;
			triangles = nullptr;
			numTriangles = 0;
		}
	}

	/**
	 * @brief Default constructor for mesh
	 *
	 * Initializes empty mesh with no triangles or acceleration structure.
	 */
	Mesh::Mesh()
		: numTriangles{ 0 }, values{ nullptr }, accelerator{ nullptr }
	{
	}

	/**
	 * @brief Builds BVH acceleration structure for the mesh
	 *
	 * Creates an octree-based BVH to accelerate spatial queries on the mesh.
	 * The structure enables fast ray-mesh intersection, collision detection,
	 * and spatial queries by hierarchically organizing triangles.
	 *
	 * Algorithm:
	 * 1. Calculate tight bounding box around all mesh vertices
	 * 2. Create root BVH node encompassing entire mesh
	 * 3. Initialize with all triangle indices
	 * 4. Recursively subdivide to depth of 3 levels
	 */
	void Mesh::Accelerate()
	{
		// Avoid rebuilding existing acceleration structure
		if (accelerator != nullptr)
		{
			return;
		}

		// Calculate mesh bounding box by examining all vertices
		// ASSUMPTION: vertices array contains numTriangles * 3 elements
		Vector3 min = vertices[0];
		Vector3 max = vertices[0];

		for (int i = 1; i < numTriangles * 3; ++i)
		{
			// Component-wise min/max calculation for tight bounds
			min.x = MathF::Min(vertices[i].x, min.x);
			min.y = MathF::Min(vertices[i].y, min.y);
			min.z = MathF::Min(vertices[i].z, min.z);
			max.x = MathF::Max(vertices[i].x, max.x);
			max.y = MathF::Max(vertices[i].y, max.y);
			max.z = MathF::Max(vertices[i].z, max.z);
		}

		// Create root BVH node encompassing entire mesh
		accelerator = new BvhNode;
		accelerator->bounds = Aabb::FromMinMax(min, max);
		accelerator->numTriangles = numTriangles;
		accelerator->triangles = new int[numTriangles];

		// Initialize root with all triangle indices (0, 1, 2, ..., numTriangles-1)
		for (int i = 0; i < numTriangles; ++i)
		{
			accelerator->triangles[i] = i;
		}

		// Begin recursive subdivision with maximum depth of 3
		// Depth 3 = up to 8^3 = 512 potential leaf nodes
		accelerator->Split(this, 3);
	}

	/**
 * @brief Tests if the mesh intersects with an Axis-Aligned Bounding Box
 * @param other AABB to test intersection against
 * @return True if any triangle in the mesh intersects the AABB
 *
 * Uses dual-path algorithm for optimal performance:
 * - Accelerated path: BVH traversal with spatial pruning (O(log n) average)
 * - Fallback path: Brute-force linear scan when no BVH available (O(n))
 *
 * The BVH traversal uses breadth-first search with spatial pruning to skip
 * entire subtrees that don't intersect the query AABB, dramatically reducing
 * the number of triangle tests required for large meshes.
 */
	bool Mesh::Intersects(const Aabb& other) const
	{
		// Fallback path: No acceleration structure available
		if (accelerator == nullptr)
		{
			// Linear scan through all triangles
			for (int i = 0; i < numTriangles; ++i)
			{
				if (triangles[i].Intersects(other))
				{
					return true;  // Early exit on first intersection
				}
			}
		}
		else
		{
			// Accelerated path: BVH traversal with spatial pruning
			// Use breadth-first traversal with explicit processing queue
			list<BvhNode*> toProcess;
			toProcess.emplace_front(accelerator);

			while (!toProcess.empty())
			{
				// Pop next node from processing queue
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				// Test triangles in leaf nodes
				if (iterator->numTriangles > 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						if (triangles[iterator->triangles[i]].Intersects(other))
						{
							return true;  // Early exit on intersection found
						}
					}
				}

				// Traverse children if this is an internal node
				if (iterator->children != nullptr)
				{
					// Process children in reverse order for consistent traversal
					for (int i = 8 - 1; i >= 0; --i)
					{
						// Spatial pruning: only traverse children that intersect query AABB
						if (other.Intersects(iterator->children[i].bounds))
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		return false;  // No intersection found
	}

	/**
	 * @brief Tests if the mesh intersects with an Oriented Bounding Box
	 * @param other OBB to test intersection against
	 * @return True if any triangle in the mesh intersects the OBB
	 *
	 * Similar to AABB intersection but handles arbitrarily oriented bounding boxes.
	 * The BVH traversal uses conservative AABB-OBB intersection tests for spatial
	 * pruning, then performs precise triangle-OBB intersection on candidate triangles.
	 *
	 * This is more computationally expensive than AABB testing due to the arbitrary
	 * orientation requiring SAT-based intersection calculations.
	 */
	bool Mesh::Intersects(const Obb& other) const
	{
		if (accelerator == nullptr)
		{
			// Brute-force: Test all triangles against OBB
			for (int i = 0; i < numTriangles; ++i)
			{
				if (triangles[i].Intersects(other))
				{
					return true;
				}
			}
		}
		else
		{
			// BVH traversal with OBB-AABB intersection pruning
			list<BvhNode*> toProcess;
			toProcess.emplace_front(accelerator);

			while (!toProcess.empty())
			{
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				// Test triangles in current node
				if (iterator->numTriangles > 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						if (triangles[iterator->triangles[i]].Intersects(other))
						{
							return true;
						}
					}
				}

				// Add children to processing queue with spatial pruning
				if (iterator->children != nullptr)
				{
					for (int i = 8 - 1; i >= 0; --i)
					{
						// Test query OBB against child node bounds for pruning
						if (other.Intersects(iterator->children[i].bounds))
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		return false;
	}

	/**
	 * @brief Tests if the mesh intersects with a plane
	 * @param other Plane to test intersection against
	 * @return True if any triangle crosses, touches, or lies on the plane
	 *
	 * Plane intersection testing is highly useful for:
	 * - Frustum culling in rendering systems
	 * - CSG (Constructive Solid Geometry) operations
	 * - Spatial partitioning and level-of-detail systems
	 *
	 * The BVH acceleration is particularly effective for plane tests since
	 * entire spatial regions can be quickly classified as being on one side
	 * of the plane or spanning across it.
	 */
	bool Mesh::Intersects(const Plane& other) const
	{
		if (accelerator == nullptr)
		{
			// Test all triangles against plane
			for (int i = 0; i < numTriangles; ++i)
			{
				if (triangles[i].Intersects(other))
				{
					return true;
				}
			}
		}
		else
		{
			// BVH traversal with plane-AABB intersection pruning
			list<BvhNode*> toProcess;
			toProcess.emplace_front(accelerator);

			while (!toProcess.empty())
			{
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				// Test triangles in leaf nodes
				if (iterator->numTriangles > 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						if (triangles[iterator->triangles[i]].Intersects(other))
						{
							return true;
						}
					}
				}

				// Traverse children with spatial pruning
				if (iterator->children != nullptr)
				{
					for (int i = 8 - 1; i >= 0; --i)
					{
						// Test plane against child bounds for spatial pruning
						if (other.Intersects(iterator->children[i].bounds))
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		return false;
	}

	/**
	 * @brief Tests if the mesh intersects with a sphere
	 * @param other Sphere to test intersection against
	 * @return True if any triangle intersects or is contained within the sphere
	 *
	 * Sphere intersection testing is essential for:
	 * - Collision detection in physics systems
	 * - Proximity queries and spatial analysis
	 * - Bounding sphere culling for rendering optimization
	 *
	 * The triangle-sphere intersection uses closest point calculations, making
	 * it more expensive than simpler shape tests but providing exact results
	 * for complex geometric scenarios.
	 */
	bool Mesh::Intersects(const Sphere& other) const
	{
		if (accelerator == nullptr)
		{
			// Linear search through all triangles
			for (int i = 0; i < numTriangles; ++i)
			{
				if (triangles[i].Intersects(other))
				{
					return true;
				}
			}
		}
		else
		{
			// BVH traversal with sphere-AABB intersection pruning
			list<BvhNode*> toProcess;
			toProcess.emplace_front(accelerator);

			while (!toProcess.empty())
			{
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				// Test triangles in current node
				if (iterator->numTriangles > 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						if (triangles[iterator->triangles[i]].Intersects(other))
						{
							return true;
						}
					}
				}

				// Add children with spatial pruning
				if (iterator->children != nullptr)
				{
					for (int i = 8 - 1; i >= 0; --i)
					{
						// Test sphere against child bounds for spatial pruning
						if (other.Intersects(iterator->children[i].bounds))
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		return false;
	}
}
