#include "Nudge/Colliders/BoxCollider.hpp"

#include "Nudge/Maths/Quaternion.hpp"
#include "Nudge/Shapes/OBB.hpp"

namespace Nudge
{
	BoxCollider::BoxCollider()
		: Collider{ new Obb{ Vector3{ 0.f }, Vector3{ 1.f } } }
	{
	}

	void BoxCollider::SetOrigin(const Vector3& origin) const
	{
		As<Obb>()->origin = origin;
	}

	void BoxCollider::SetExtents(const Vector3& extents) const
	{
		As<Obb>()->extents = extents;
	}

	void BoxCollider::SetOrientation(const Vector3& axis, float angle) const
	{
		As<Obb>()->orientation = Quaternion::FromAxisAngle(axis, angle).ToMatrix3();
	}

	Vector3 BoxCollider::Origin() const
	{
		return As<Obb>()->origin;
	}

	Vector3 BoxCollider::Extents() const
	{
		return As<Obb>()->extents;
	}

	Matrix3 BoxCollider::Orientation() const
	{
		return As<Obb>()->orientation;
	}
}
