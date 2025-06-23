#include "Nudge/Ray.hpp"

#include "Nudge/RaycastHit.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/Mesh.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

#include <list>

using std::list;

namespace Nudge
{
	Ray Ray::FromPoints(const Vector3& from, const Vector3& to)
	{
		// Calculate direction vector and normalize it immediately
		return { from, (to - from).Normalized() };
	}

	Ray::Ray()
		: Ray(Vector3{ 0.f }, Vector3{ 0.f, 0.f, 1.f })
	{
		// Ensure direction is normalized
		Normalize();
	}

	Ray::Ray(const Vector3& origin, const Vector3& direction)
		: origin{ origin }, direction{ direction }
	{
		// Ensure direction is normalized
		Normalize();
	}

	void Ray::Normalize()
	{
		// Normalize the direction vector
		direction.Normalize();

		// Handle degenerate case where direction becomes zero
		if(direction.IsZero())
		{
			direction = { 0.f, 0.f, 1.f };  // Default to positive Z direction
		}
	}

	bool Ray::Contains(const Vector3& point) const
	{
		// Ray origin is always on the ray
		if (point == origin)
		{
			return true;
		}

		// Check if the normalized vector from origin to point is parallel to ray direction
		const Vector3 norm = (point - origin).Normalized();
		// Dot product of parallel unit vectors should be ±1, we want +1 (same direction)
		return MathF::IsNearZero(MathF::Abs(Vector3::Dot(norm, direction)) - 1.f);
	}

	Vector3 Ray::ClosestPoint(const Vector3& point) const
	{
		// Project the point onto the ray, clamping to ensure we don't go behind the origin
		const float t = MathF::Max(Vector3::Dot(point - origin, direction), 0.f);

		// Return the point on the ray at parameter t
		return origin + direction * t;
	}

	bool Ray::CastAgainst(const Aabb& other, RaycastHit* hit) const
	{
		// Initialize hit data to default values
		ResetHit(hit);

		// Get AABB bounds
		Vector3 min = other.Min();
		Vector3 max = other.Max();

		// Array to store intersection parameters for each face
		float t[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

		// Check if ray direction components are valid (non-zero) to avoid division by zero
		const bool dirXValid = !MathF::IsNearZero(direction.x);
		const bool dirYValid = !MathF::IsNearZero(direction.y);
		const bool dirZValid = !MathF::IsNearZero(direction.z);

		// Calculate intersection parameters for each pair of parallel faces
		t[0] = dirXValid ? (min.x - origin.x) / direction.x : 0.f;  // Left face
		t[1] = dirXValid ? (max.x - origin.x) / direction.x : 0.f;  // Right face
		t[2] = dirYValid ? (min.y - origin.y) / direction.y : 0.f;  // Bottom face
		t[3] = dirYValid ? (max.y - origin.y) / direction.y : 0.f;  // Top face
		t[4] = dirZValid ? (min.z - origin.z) / direction.z : 0.f;  // Near face
		t[5] = dirZValid ? (max.z - origin.z) / direction.z : 0.f;  // Far face

		// Find the maximum of the minimum intersections (entry point)
		const float tMin = MathF::Max(
			MathF::Max(
				MathF::Min(t[0], t[1]),  // Min X intersection
				MathF::Min(t[2], t[3])   // Min Y intersection
			),
			MathF::Min(t[4], t[5])       // Min Z intersection
		);

		// Find the minimum of the maximum intersections (exit point)
		const float tMax = MathF::Min(
			MathF::Min(
				MathF::Max(t[0], t[1]),  // Max X intersection
				MathF::Max(t[2], t[3])   // Max Y intersection
			),
			MathF::Max(t[4], t[5])       // Max Z intersection
		);

		// No intersection if ray misses box or box is behind ray
		if (tMax < 0.f || tMin > tMax)
		{
			return false;
		}

		// Use the closer intersection point, but prefer positive t values
		float tResult = tMin;
		if (tMin < 0.f)  // If entry point is behind ray origin
		{
			tResult = tMax;  // Use exit point instead
		}

		// Fill hit information if requested
		if (hit != nullptr)
		{
			hit->distance = tResult;
			hit->didHit = true;
			hit->point = origin + direction * tResult;

			// Face normals for each of the 6 faces
			static Vector3 normals[] =
			{
				{ -1.f, 0.f, 0.f }, { 1.f, 0.f, 0.f },  // X faces
				{ 0.f, -1.f, 0.f }, { 0.f, 1.f, 0.f },  // Y faces
				{ 0.f, 0.f, -1.f }, { 0.f, 0.f, 1.f }   // Z faces
			};

			// Find which face was hit by matching the intersection parameter
			for (int i = 0; i < 6; ++i)
			{
				if (MathF::Compare(tResult, t[i]))
				{
					hit->normal = normals[i];
				}
			}
		}

		return true;
	}

	float Ray::CastAgainst(const Mesh& other)
	{
		// Check if mesh has a BVH acceleration structure
		if (other.accelerator == nullptr)
		{
			// Brute force: test against all triangles
			for (int i = 0; i < other.numTriangles; ++i)
			{
				RaycastHit hit;
				CastAgainst(other.triangles[i], &hit);
				
				// Return first valid hit distance
				if (hit.distance >= 0.f)
				{
					return hit.distance;
				}
			}
		}
		else
		{
			// Use BVH for accelerated traversal
			list<BvhNode*> toProcess;
			toProcess.emplace_front(other.accelerator);

			// Breadth-first traversal of the BVH
			while (!toProcess.empty())
			{
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				// If this is a leaf node, test its triangles
				if (iterator->numTriangles >= 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						float r = CastAgainst(other.triangles[iterator->triangles[i]]);
						if (r >= 0)
						{
							return r;  // Return first hit
						}
					}
				}

				// If this is an internal node, test child bounding boxes
				if (iterator->children != nullptr)
				{
					// Test children in reverse order for consistent traversal
					for (int i = 8 - 1; i >= 0; --i)
					{
						RaycastHit hit;
						CastAgainst(iterator->children[i].bounds, &hit);

						// Add child to processing queue if ray hits its bounds
						if (hit.distance >= 0.f)
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		// No intersection found
		return -1.f;
	}

	bool Ray::CastAgainst(const Obb& other, RaycastHit* hit) const
	{
		// Initialize hit data to default values
		ResetHit(hit);

		// Vector from ray origin to OBB center
		const Vector3 p = other.origin - origin;

		// Get OBB's local axes
		const Vector3 x = other.orientation.GetColumn(0);
		const Vector3 y = other.orientation.GetColumn(1);
		const Vector3 z = other.orientation.GetColumn(2);

		// Project ray direction onto each OBB axis
		Vector3 f =
		{
			Vector3::Dot(x, direction),
			Vector3::Dot(y, direction),
			Vector3::Dot(z, direction)
		};

		// Project vector to OBB center onto each OBB axis
		const Vector3 e =
		{
			Vector3::Dot(x, p),
			Vector3::Dot(y, p),
			Vector3::Dot(z, p)
		};

		// Calculate intersection parameters for each pair of parallel faces
		float t[6] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
		for (int i = 0; i < 3; ++i)
		{
			// Handle case where ray is parallel to face pair
			if (MathF::IsNearZero(f[i]))
			{
				// Check if ray origin is outside the slab
				if (-e[i] - other.extents[i] > 0.f ||
					-e[i] + other.extents[i] < 0.f)
				{
					return false;  // Ray misses the OBB
				}

				f[i] = 0.00001f; // Avoid division by zero in next calculation
			}

			// Calculate intersection parameters for this axis
			t[i * 2 + 0] = (e[i] + other.extents[i]) / f[i];  // Near face
			t[i * 2 + 1] = (e[i] - other.extents[i]) / f[i];  // Far face
		}

		// Find entry and exit points using slab method
		const float tMin = MathF::Max(
			MathF::Max(
				MathF::Min(t[0], t[1]),  // Min X intersection
				MathF::Min(t[2], t[3])   // Min Y intersection
			),
			MathF::Min(t[4], t[5])       // Min Z intersection
		);

		const float tMax = MathF::Min(
			MathF::Min(
				MathF::Max(t[0], t[1]),  // Max X intersection
				MathF::Max(t[2], t[3])   // Max Y intersection
			),
			MathF::Max(t[4], t[5])       // Max Z intersection
		);

		// No intersection if ray misses box or box is behind ray
		if (tMax < 0.f || tMin > tMax)
		{
			return false;
		}

		// Use the closer intersection point, but prefer positive t values
		float tResult = tMin;
		if (tMin < 0.f)  // If entry point is behind ray origin
		{
			tResult = tMax;  // Use exit point instead
		}

		// Fill hit information if requested
		if (hit != nullptr)
		{
			hit->distance = tResult;
			hit->didHit = true;
			hit->point = origin + direction * tResult;

			// Local space normals for each of the 6 faces
			static Vector3 normals[] =
			{
				{ -1.f, 0.f, 0.f }, { 1.f, 0.f, 0.f },  // X faces
				{ 0.f, -1.f, 0.f }, { 0.f, 1.f, 0.f },  // Y faces
				{ 0.f, 0.f, -1.f }, { 0.f, 0.f, 1.f }   // Z faces
			};

			// Find which face was hit by matching the intersection parameter
			for (int i = 0; i < 6; ++i)
			{
				if (MathF::Compare(tResult, t[i]))
				{
					hit->normal = normals[i];
				}
			}
		}

		return true;
	}

	bool Ray::CastAgainst(const Plane& other, RaycastHit* hit) const
	{
		// Initialize hit data to default values
		ResetHit(hit);

		// Calculate ray direction dot plane normal
		const float nd = Vector3::Dot(direction, other.normal);
		// Calculate ray origin dot plane normal
		const float pn = Vector3::Dot(origin, other.normal);

		// Ray is parallel to plane or pointing away from it
		if (nd >= 0.f)
		{
			return false;
		}

		// Calculate intersection parameter
		const float t = (other.distance - pn) / nd;

		// Check if intersection is in front of ray origin
		if (t >= 0.f)
		{
			// Fill hit information if requested
			if (hit != nullptr)
			{
				hit->distance = t;
				hit->didHit = true;
				hit->point = origin + direction * t;
				hit->normal = other.normal.Normalized();
			}

			return true;
		}

		return false;
	}

	bool Ray::CastAgainst(const Sphere& other, RaycastHit* hit) const
	{
		// Initialize hit data to default values
		ResetHit(hit);

		// Vector from ray origin to sphere center
		const Vector3 e = other.origin - origin;
		const float rSqr = MathF::Squared(other.radius);
		const float eSqr = e.MagnitudeSqr();

		// Project sphere center vector onto ray direction
		const float a = Vector3::Dot(e, direction);
		const float aSqr = MathF::Squared(a);

		// Calculate squared distance from sphere center to ray
		const float bSqr = eSqr - aSqr;
		// Calculate half-chord length inside sphere
		const float f = MathF::Sqrt(rSqr - bSqr);

		// Distance to near intersection point
		float distance = a - f;

		// Check if ray actually intersects sphere (discriminant test)
		if (rSqr - (eSqr - bSqr) < 0.f)
		{
			return false;  // No intersection
		}

		// If ray origin is inside sphere, use far intersection
		if (eSqr < rSqr)
		{
			distance = a + f;
		}

		// Fill hit information if requested
		if (hit != nullptr)
		{
			hit->distance = distance;
			hit->didHit = true;
			hit->point = origin + direction * distance;
			// Normal points from sphere center to hit point
			hit->normal = (hit->point - other.origin).Normalized();
		}

		return true;
	}

	bool Ray::CastAgainst(const Triangle& tri, RaycastHit* hit) const
	{
		// First, test intersection with the triangle's plane
		const Plane plane = Plane::From(tri);
		RaycastHit planeHit;

		if (!CastAgainst(plane, &planeHit))
		{
			return false;  // Ray doesn't intersect the plane
		}

		// Calculate intersection point on the plane
		float t = planeHit.distance;
		const Vector3 result = origin + direction * t;

		// Check if intersection point is inside the triangle using barycentric coordinates
		const Vector3 barycentric = tri.Barycentric(result);

		// Point is inside triangle if all barycentric coordinates are non-negative
		// and they sum to 1
		if (barycentric.x >= 0.0f && barycentric.y >= 0.0f && barycentric.z >= 0.0f &&
			MathF::Compare(barycentric.x + barycentric.y + barycentric.z, 1.0f))
		{
			// Fill hit information if requested
			if (hit != nullptr)
			{
				hit->distance = t;
				hit->didHit = true;
				hit->point = origin + direction * t;
				hit->normal = plane.normal;
			}

			return true;
		}

		return false;  // Point is outside triangle
	}

	void Ray::ResetHit(RaycastHit* hit)
	{
		// Initialize hit structure to default "no hit" state
		if (hit != nullptr)
		{
			hit->distance = 0.f;
			hit->didHit = false;
			hit->normal = Vector3{ 0.f, 0.f, 1.f };  // Default normal
			hit->point = Vector3{ 0.f };             // Zero point
		}
	}
}