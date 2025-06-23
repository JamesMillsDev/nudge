#include "Nudge/Ray.hpp"

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/Mesh.hpp"
#include "Nudge/Shapes/OBB.hpp"
#include "Nudge/Shapes/Plane.hpp"
#include "Nudge/Shapes/Sphere.hpp"
#include "Nudge/Shapes/Triangle.hpp"

#include <list>

#include "Nudge/RaycastHit.hpp"

using std::list;
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
		Normalize();
	}

	/**
	 * @brief Constructs a ray with specified origin and direction
	 * @param origin Starting point of the ray
	 * @param direction Direction vector of the ray (should be normalized)
	 */
	Ray::Ray(const Vector3& origin, const Vector3& direction)
		: origin{ origin }, direction{ direction }
	{
		Normalize();
	}

	/**
	 * @brief Normalizes the ray's direction vector to unit length
	 */
	void Ray::Normalize()
	{
		direction.Normalize();

		if(direction.IsZero())
		{
			direction = { 0.f, 0.f, 1.f };
		}
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
		return MathF::IsNearZero(MathF::Abs(Vector3::Dot(norm, direction)) - 1.f);
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

	bool Ray::CastAgainst(const Aabb& other, RaycastHit* hit) const
	{
		ResetHit(hit);

		Vector3 min = other.Min();
		Vector3 max = other.Max();

		float t[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

		const bool dirXValid = !MathF::IsNearZero(direction.x);
		const bool dirYValid = !MathF::IsNearZero(direction.y);
		const bool dirZValid = !MathF::IsNearZero(direction.z);

		t[0] = dirXValid ? (min.x - origin.x) / direction.x : 0.f;
		t[1] = dirXValid ? (max.x - origin.x) / direction.x : 0.f;
		t[2] = dirYValid ? (min.y - origin.y) / direction.y : 0.f;
		t[3] = dirYValid ? (max.y - origin.y) / direction.y : 0.f;
		t[4] = dirZValid ? (min.z - origin.z) / direction.z : 0.f;
		t[5] = dirZValid ? (max.z - origin.z) / direction.z : 0.f;

		const float tMin = MathF::Max(
			MathF::Max(
				MathF::Min(t[0], t[1]),
				MathF::Min(t[2], t[3])
			),
			MathF::Min(t[4], t[5])
		);

		const float tMax = MathF::Min(
			MathF::Min(
				MathF::Max(t[0], t[1]),
				MathF::Max(t[2], t[3])
			),
			MathF::Max(t[4], t[5])
		);

		if (tMax < 0.f || tMin > tMax)
		{
			return false;
		}

		float tResult = tMin;
		if (tMin < 0.f)
		{
			tResult = tMax;
		}

		if (hit != nullptr)
		{
			hit->distance = tResult;
			hit->didHit = true;
			hit->point = origin + direction * tResult;

			static Vector3 normals[] =
			{
				{ -1.f, 0.f, 0.f }, { 1.f, 0.f, 0.f },
				{ 0.f, -1.f, 0.f }, { 0.f, 1.f, 0.f },
				{ 0.f, 0.f, -1.f }, { 0.f, 0.f, 1.f }
			};

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
		if (other.accelerator == nullptr)
		{
			for (int i = 0; i < other.numTriangles; ++i)
			{
				RaycastHit hit;
				CastAgainst(other.triangles[i], &hit);
				
				if (hit.distance >= 0.f)
				{
					return hit.distance;
				}
			}
		}
		else
		{
			list<BvhNode*> toProcess;
			toProcess.emplace_front(other.accelerator);

			while (!toProcess.empty())
			{
				BvhNode* iterator = *toProcess.begin();
				toProcess.erase(toProcess.begin());

				if (iterator->numTriangles >= 0)
				{
					for (int i = 0; i < iterator->numTriangles; ++i)
					{
						float r = CastAgainst(other.triangles[iterator->triangles[i]]);
						if (r >= 0)
						{
							return r;
						}
					}
				}

				if (iterator->children != nullptr)
				{
					for (int i = 8 - 1; i >= 0; --i)
					{
						RaycastHit hit;
						CastAgainst(iterator->children[i].bounds, &hit);

						if (hit.distance >= 0.f)
						{
							toProcess.emplace_front(&iterator->children[i]);
						}
					}
				}
			}
		}

		return -1.f;
	}

	bool Ray::CastAgainst(const Obb& other, RaycastHit* hit) const
	{
		ResetHit(hit);

		const Vector3 p = other.origin - origin;

		const Vector3 x = other.orientation.GetColumn(0);
		const Vector3 y = other.orientation.GetColumn(1);
		const Vector3 z = other.orientation.GetColumn(2);

		Vector3 f =
		{
			Vector3::Dot(x, direction),
			Vector3::Dot(y, direction),
			Vector3::Dot(z, direction)
		};

		const Vector3 e =
		{
			Vector3::Dot(x, p),
			Vector3::Dot(y, p),
			Vector3::Dot(z, p)
		};

		float t[6] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };
		for (int i = 0; i < 3; ++i)
		{
			if (MathF::IsNearZero(f[i]))
			{
				if (-e[i] - other.extents[i] > 0.f ||
					-e[i] + other.extents[i] < 0.f)
				{
					return false;
				}

				f[i] = 0.00001f; // Avoid div by 0!
			}

			t[i * 2 + 0] = (e[i] + other.extents[i]) / f[i];
			t[i * 2 + 1] = (e[i] - other.extents[i]) / f[i];
		}

		const float tMin = MathF::Max(
			MathF::Max(
				MathF::Min(t[0], t[1]),
				MathF::Min(t[2], t[3])
			),
			MathF::Min(t[4], t[5])
		);

		const float tMax = MathF::Min(
			MathF::Min(
				MathF::Max(t[0], t[1]),
				MathF::Max(t[2], t[3])
			),
			MathF::Max(t[4], t[5])
		);

		if (tMax < 0.f || tMin > tMax)
		{
			return false;
		}

		float tResult = tMin;
		if (tMin < 0.f)
		{
			tResult = tMax;
		}

		if (hit != nullptr)
		{
			hit->distance = tResult;
			hit->didHit = true;
			hit->point = origin + direction * tResult;

			static Vector3 normals[] =
			{
				{ -1.f, 0.f, 0.f }, { 1.f, 0.f, 0.f },
				{ 0.f, -1.f, 0.f }, { 0.f, 1.f, 0.f },
				{ 0.f, 0.f, -1.f }, { 0.f, 0.f, 1.f }
			};

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
		ResetHit(hit);

		const float nd = Vector3::Dot(direction, other.normal);
		const float pn = Vector3::Dot(origin, other.normal);

		if (nd >= 0.f)
		{
			return false;
		}

		const float t = (other.distance - pn) / nd;

		if (t >= 0.f)
		{
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
		ResetHit(hit);

		const Vector3 e = other.origin - origin;
		const float rSqr = MathF::Squared(other.radius);
		const float eSqr = e.MagnitudeSqr();

		const float a = Vector3::Dot(e, direction);
		const float aSqr = MathF::Squared(a);

		const float bSqr = eSqr - aSqr;
		const float f = MathF::Sqrt(rSqr - bSqr);

		float distance = a - f;

		if (rSqr - (eSqr - bSqr) < 0.f)
		{
			return false;
		}

		if (eSqr < rSqr)
		{
			distance = a + f;
		}

		if (hit != nullptr)
		{
			hit->distance = distance;
			hit->didHit = true;
			hit->point = origin + direction * distance;
			hit->normal = (hit->point - other.origin).Normalized();
		}

		return true;
	}

	bool Ray::CastAgainst(const Triangle& tri, RaycastHit* hit) const
	{
		const Plane plane = Plane::From(tri);
		RaycastHit planeHit;

		if (!CastAgainst(plane, &planeHit))
		{
			return false;
		}

		float t = planeHit.distance;
		const Vector3 result = origin + direction * t;

		const Vector3 barycentric = tri.Barycentric(result);

		if (barycentric.x >= 0.0f && barycentric.y >= 0.0f && barycentric.z >= 0.0f &&
			MathF::Compare(barycentric.x + barycentric.y + barycentric.z, 1.0f))
		{
			if (hit != nullptr)
			{
				hit->distance = t;
				hit->didHit = true;
				hit->point = origin + direction * t;
				hit->normal = plane.normal;
			}

			return true;
		}

		return false;
	}

	void Ray::ResetHit(RaycastHit* hit)
	{
		if (hit != nullptr)
		{
			hit->distance = 0.f;
			hit->didHit = false;
			hit->normal = Vector3{ 0.f, 0.f, 1.f };
			hit->point = Vector3{ 0.f };
		}
	}
}
