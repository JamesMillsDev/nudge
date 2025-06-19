#include "Nudge/Physics/Shapes/Ray.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/AABB.hpp"
#include "Nudge/Physics/Shapes/OBB.hpp"
#include "Nudge/Physics/Shapes/Plane.hpp"
#include "Nudge/Physics/Shapes/Sphere.hpp"
#include "Nudge/Physics/Shapes/Triangle.hpp"

using std::numeric_limits;

namespace Nudge
{
	/**
	 * @brief Creates a ray from two points
	 * @param from Starting point of the ray
	 * @param to Target point to aim the ray towards
	 * @return Ray with origin at 'from' and direction normalized towards 'to'
	 */
	Ray Ray::FromPoints(const Vector3& from, const Vector3& to)
	{
		return { from, (to - from).Normalized() };
	}

	/**
	 * @brief Default constructor - creates ray at origin pointing along positive Z-axis
	 */
	Ray::Ray()
		: Ray(Vector3{ 0.f }, Vector3{ 0.f, 0.f, 1.f })
	{
	}

	/**
	 * @brief Constructs a ray with specified origin and direction
	 * @param origin Starting point of the ray
	 * @param direction Direction vector of the ray (should be normalized)
	 */
	Ray::Ray(const Vector3& origin, const Vector3& direction)
		: origin{ origin }, direction{ direction }
	{
	}

	/**
	 * @brief Normalizes the ray's direction vector to unit length
	 */
	void Ray::Normalize()
	{
		direction.Normalize();
	}

	/**
	 * @brief Tests if a point lies on the ray
	 * @param point Point to test
	 * @return True if the point lies on the ray (within floating-point tolerance)
	 */
	bool Ray::Contains(const Vector3& point) const
	{
		// Ray origin is always on the ray
		if (point == origin)
		{
			return true;
		}

		// Check if the normalized vector from origin to point is parallel to ray direction
		const Vector3 norm = (point - origin).Normalized();
		return MathF::IsNearZero(Vector3::Dot(norm, direction));
	}

	/**
	 * @brief Finds the closest point on the ray to a given point
	 * @param point Reference point to find the closest approach to
	 * @return Point on the ray closest to the input point
	 */
	Vector3 Ray::ClosestPoint(const Vector3& point) const
	{
		// Project the point onto the ray, clamping to ensure we don't go behind the origin
		const float t = MathF::Max(Vector3::Dot(point - origin, direction), 0.f);

		return origin + direction * t;
	}

	/**
	 * @brief Performs ray-AABB intersection using the slab method
	 * @param other AABB to test intersection against
	 * @return Distance to intersection point, or -1 if no intersection
	 */
	float Ray::CastAgainst(const Aabb& other) const
	{
		// Get AABB bounds
		const Vector3 min = other.Min();
		const Vector3 max = other.Max();

		// Initialize intersection parameter range
		float tMin = 0.f;                               // Near intersection distance
		float tMax = numeric_limits<float>::infinity(); // Far intersection distance

		// Test intersection with each pair of parallel planes (slabs)
		for (int i = 0; i < 3; ++i)
		{
			if (MathF::IsNearZero(direction[i]))
			{
				// Ray is parallel to this slab - check if origin is within slab bounds
				if (origin[i] < min[i] || origin[i] > max[i])
				{
					return -1.f; // Ray misses the AABB entirely
				}
			}
			else
			{
				// Calculate intersection distances with both planes of this slab
				float t1 = (min[i] - origin[i]) / direction[i]; // Distance to min plane
				float t2 = (max[i] - origin[i]) / direction[i]; // Distance to max plane

				// Ensure t1 is the near plane and t2 is the far plane
				if (t1 > t2)
				{
					std::swap(t1, t2);
				}

				// Update the intersection parameter range
				tMin = MathF::Max(tMin, t1); // Latest entry point
				tMax = MathF::Min(tMax, t2); // Earliest exit point

				// No intersection if entry point is after exit point
				if (tMin > tMax)
				{
					return -1.f;
				}
			}
		}

		// No intersection if the AABB is entirely behind the ray origin
		if (tMax < 0.f)
		{
			return -1.f;
		}

		// Return the closest intersection point in front of the ray origin
		return tMin < 0.f ? tMax : tMin;
	}

	/**
	 * @brief Performs ray-OBB intersection using the separating axis theorem
	 * @param other OBB (Oriented Bounding Box) to test intersection against
	 * @return Distance to intersection point, or -1 if no intersection
	 */
	float Ray::CastAgainst(const Obb& other) const
	{
		// Get OBB's local coordinate system axes
		const Vector3 x = other.orientation.GetColumn(0); // OBB's local X-axis
		const Vector3 y = other.orientation.GetColumn(1); // OBB's local Y-axis
		const Vector3 z = other.orientation.GetColumn(2); // OBB's local Z-axis

		// Vector from ray origin to OBB center
		const Vector3 p = other.origin - origin;

		// Transform ray direction into OBB local space
		const Vector3 f =
		{
			Vector3::Dot(x, direction), // Ray direction component along OBB's X-axis
			Vector3::Dot(y, direction), // Ray direction component along OBB's Y-axis
			Vector3::Dot(z, direction)  // Ray direction component along OBB's Z-axis
		};

		// Transform ray origin relative to OBB center into OBB local space
		const Vector3 e =
		{
			Vector3::Dot(x, p), // Distance from ray origin to OBB center along OBB's X-axis
			Vector3::Dot(y, p), // Distance from ray origin to OBB center along OBB's Y-axis
			Vector3::Dot(z, p)  // Distance from ray origin to OBB center along OBB's Z-axis
		};

		float tMin = 0.0f;
		float tMax = std::numeric_limits<float>::infinity();

		// Test intersection with each pair of parallel planes (slabs) in OBB local space
		for (int i = 0; i < 3; ++i)
		{
			if (MathF::IsNearZero(f[i]))
			{
				// Ray is parallel to this slab - check if ray origin is within slab bounds
				if (MathF::Abs(e[i]) > other.extents[i])
				{
					return -1.f; // Ray misses the OBB entirely
				}
			}
			else
			{
				// Calculate intersection distances with both planes of this slab
				float t1 = (e[i] + other.extents[i]) / f[i]; // Distance to positive extent plane
				float t2 = (e[i] - other.extents[i]) / f[i]; // Distance to negative extent plane

				// Ensure t1 is the near plane and t2 is the far plane
				if (t1 > t2)
				{
					std::swap(t1, t2);
				}

				// Update the intersection parameter range
				tMin = MathF::Max(tMin, t1); // Latest entry point
				tMax = MathF::Min(tMax, t2); // Earliest exit point

				// No intersection if entry point is after exit point
				if (tMin > tMax)
				{
					return -1.f;
				}
			}
		}

		// No intersection if the OBB is entirely behind the ray origin
		if (tMax < 0.f)
		{
			return -1.f;
		}

		// Return the closest intersection point in front of the ray origin
		return tMin < 0.f ? tMax : tMin;
	}

	/**
	 * @brief Performs ray-plane intersection test
	 * @param other Plane to test intersection against
	 * @return Distance to intersection point, or -1 if no intersection
	 */
	float Ray::CastAgainst(const Plane& other) const
	{
		// Calculate dot product of ray direction with plane normal
		const float nd = Vector3::Dot(direction, other.normal);

		// Calculate dot product of ray origin with plane normal
		const float pn = Vector3::Dot(origin, other.normal);

		// Check if ray is parallel to plane or pointing away from it
		if (nd >= 0.f)
		{
			return -1.f; // No intersection (parallel or pointing away)
		}

		// Calculate intersection parameter using plane equation: dot(point, normal) = distance
		// Solve for t in: dot(origin + t * direction, normal) = distance
		const float t = (other.distance - pn) / nd;

		// Return intersection distance if it's in front of ray origin
		return t >= 0.f ? t : -1.f;
	}

	/**
	 * @brief Performs ray-sphere intersection using quadratic equation
	 * @param other Sphere to test intersection against
	 * @return Distance to intersection point, or -1 if no intersection
	 */
	float Ray::CastAgainst(const Sphere& other) const
	{
		// Vector from ray origin to sphere center
		const Vector3 e = other.origin - origin;

		// Precompute squared values to avoid repeated calculations
		const float rSqr = MathF::Squared(other.radius); // Sphere radius squared
		const float eSqr = e.MagnitudeSqr();             // Distance squared from ray origin to sphere center

		// Project sphere center onto ray direction
		const float a = Vector3::Dot(e, direction); // Distance along ray to the closest approach point
		const float aSqr = MathF::Squared(a);       // Squared distance along ray
		const float bSqr = eSqr - aSqr;             // Squared perpendicular distance from ray to sphere center

		// Calculate discriminant for intersection test
		const float discriminant = rSqr - bSqr;

		// No intersection if ray passes outside sphere
		if (discriminant < 0.f)
		{
			return -1.f;
		}

		// Distance from the closest approach point to intersection points
		const float f = MathF::Sqrt(discriminant);

		// Return appropriate intersection based on ray origin position relative to sphere
		return eSqr < rSqr ? a + f : a - f; // Inside sphere: far intersection, Outside sphere: near intersection
	}

	/**
	 * @brief Performs ray-triangle intersection using the two-phase approach
	 * @param tri Triangle to test intersection against
	 * @return Distance along ray to intersection point, or -1 if no intersection
	 *
	 * Algorithm:
	 * 1. First intersect ray with the triangle's plane
	 * 2. Then check if the intersection point lies within the triangle using barycentric coordinates
	 */
	float Ray::CastAgainst(const Triangle& tri) const
	{
		// Phase 1: Ray-Plane Intersection
		// Create a plane from the triangle and test intersection
		const Plane plane = Plane::From(tri);

		const float t = CastAgainst(plane);
		if (t < 0.f)
		{
			return t;  // No intersection with plane, or intersection is behind ray origin
		}

		// Phase 2: Point-in-Triangle Test
		// Calculate the intersection point on the plane
		const Vector3 result = origin + direction * t;

		// Convert world space point to barycentric coordinates relative to triangle
		const Vector3 barycentric = tri.Barycentric(result);

		// Explicit validation with sum check)
		if (barycentric.x >= 0.0f && barycentric.y >= 0.0f && barycentric.z >= 0.0f &&
			MathF::Compare(barycentric.x + barycentric.y + barycentric.z, 1.0f))
		{
			return t;
		}

		return -1.f;  // Point lies outside triangle bounds
	}
}
