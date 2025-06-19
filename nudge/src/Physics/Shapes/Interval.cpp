#include "Nudge/Physics/Shapes/Interval.hpp"

#include <algorithm>

#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Physics/Shapes/AABB.hpp"
#include "Nudge/Physics/Shapes/OBB.hpp"

namespace Nudge
{
	Interval Interval::Get(const Aabb& aabb, const Vector3& axis)
	{
		Vector3 min = aabb.Min();
		Vector3 max = aabb.Min();

		const Vector3 vertex[8] =
		{
			{ min.x, max.y, max.z },
			{ min.x, max.y, min.z },
			{ min.x, min.y, max.z },
			{ min.x, min.y, min.z },
			{ max.x, max.y, max.z },
			{ max.x, max.y, min.z },
			{ max.x, min.y, max.z },
			{ max.x, min.y, min.z }
		};

		Interval result;
		result.min = result.max = Vector3::Dot(axis, vertex[0]);

		for (int i = 1; i < 8; ++i)
		{
			float projection = Vector3::Dot(axis, vertex[i]);

			result.min = projection < result.min ? projection : result.min;
			result.max = projection > result.max ? projection : result.max;
		}

		return result;
	}

	Interval Interval::Get(const Obb& obb, const Vector3& axis)
	{
		Vector3 vertex[8];

		Vector3 origin = obb.origin;
		Vector3 extents = obb.extents;

		Vector3 axes[] = // OBB Axis 
		{
			obb.orientation.GetColumn(0),
			obb.orientation.GetColumn(1),
			obb.orientation.GetColumn(2)
		};

		vertex[0] = origin + axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
		vertex[1] = origin - axes[0] * extents[0] + axes[1] * extents[1] + axes[2] * extents[2];
		vertex[2] = origin + axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];
		vertex[3] = origin + axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
		vertex[4] = origin - axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
		vertex[5] = origin + axes[0] * extents[0] - axes[1] * extents[1] - axes[2] * extents[2];
		vertex[6] = origin - axes[0] * extents[0] + axes[1] * extents[1] - axes[2] * extents[2];
		vertex[7] = origin - axes[0] * extents[0] - axes[1] * extents[1] + axes[2] * extents[2];

		Interval result;
		result.min = result.max = Vector3::Dot(axis, vertex[0]);

		for (int i = 1; i < 8; ++i)
		{
			float projection = Vector3::Dot(axis, vertex[i]);

			result.min = projection < result.min ? projection : result.min;
			result.max = projection > result.max ? projection : result.max;
		}

		return result;
	}

	bool Interval::OverlapOnAxis(const Aabb& aabb, const Obb& obb, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(aabb, axis);
		const auto [bMin, bMax] = Get(obb, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	bool Interval::AabbObb(const Aabb& aabb, const Obb& obb)
	{
		Vector3 test[15] =
		{
			{ 1.f, 0.f, 0.f }, // AABB axis 1
			{ 0.f, 1.f, 0.f }, // AABB axis 2
			{ 0.f, 0.f, 1.f }, // AABB axis 3
			obb.orientation.GetColumn(0), // Obb axis 1
			obb.orientation.GetColumn(1), // Obb axis 2
			obb.orientation.GetColumn(2)  // Obb axis 3
		};

		for (int i = 0; i < 3; ++i)
		{
			test[6 + i * 3 + 0] = Vector3::Cross(test[i], test[0]);
			test[6 + i * 3 + 1] = Vector3::Cross(test[i], test[1]);
			test[6 + i * 3 + 2] = Vector3::Cross(test[i], test[2]);
		}

		return std::ranges::all_of(test, [&](const Vector3& axis) { return OverlapOnAxis(aabb, obb, axis); });
	}

	bool Interval::OverlapOnAxis(const Obb& a, const Obb& b, const Vector3& axis)
	{
		const auto [aMin, aMax] = Get(a, axis);
		const auto [bMin, bMax] = Get(b, axis);

		return bMin <= aMax && aMin <= bMax;
	}

	bool Interval::ObbObb(const Obb& a, const Obb& b)
	{
		Vector3 test[15] =
		{
			a.orientation.GetColumn(0),
			a.orientation.GetColumn(1),
			a.orientation.GetColumn(2),
			b.orientation.GetColumn(0),
			b.orientation.GetColumn(1),
			b.orientation.GetColumn(2),
		};

		for (int i = 0; i < 3; ++i)
		{
			test[6 + i * 3 + 0] = Vector3::Cross(test[i], test[0]);
			test[6 + i * 3 + 1] = Vector3::Cross(test[i], test[1]);
			test[6 + i * 3 + 2] = Vector3::Cross(test[i], test[2]);
		}

		return std::ranges::all_of(test, [&](const Vector3& axis) { return OverlapOnAxis(a, b, axis); });
	}
}
