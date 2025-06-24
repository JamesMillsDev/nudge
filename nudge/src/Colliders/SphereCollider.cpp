#include "Nudge/Colliders/SphereCollider.hpp"

#include "Nudge/CollisionManifold.hpp"
#include "Nudge/Colliders/BoxCollider.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Shapes/AABB.hpp"
#include "Nudge/Shapes/Sphere.hpp"

namespace Nudge
{
	SphereCollider::SphereCollider()
		: Collider{ new Sphere{ Vector3{ 0.f }, 1.f } }
	{
	}

	void SphereCollider::SetOrigin(const Vector3& origin) const
	{
		As<Sphere>()->origin = origin;
	}

	void SphereCollider::SetRadius(float radius) const
	{
		As<Sphere>()->radius = radius;
	}

	Vector3 SphereCollider::Origin() const
	{
		return As<Sphere>()->origin;
	}

	float SphereCollider::Radius() const
	{
		return As<Sphere>()->radius;
	}

	CollisionManifold SphereCollider::FindCollisionFeatures(const SphereCollider* other) const
	{
		CollisionManifold manifold;
		manifold.Reset();

		const float r = Radius() + other->Radius();
		Vector3 d = other->Origin() - Origin();

		if(d.MagnitudeSqr() - MathF::Squared(r) > 0.f ||
			MathF::IsNearZero(d.MagnitudeSqr()))
		{
			return manifold;
		}

		manifold.colliding = true;
		manifold.depth = MathF::Abs(d.Magnitude() - Radius()) * .5f;

		d.Normalize();
		manifold.normal = d;

		const float dtp = Radius() - manifold.depth;
		Vector3 contact = Origin() + d * dtp;

		manifold.contacts.emplace_back(contact);

		return manifold;
	}

	CollisionManifold SphereCollider::FindCollisionFeatures(const BoxCollider* other) const
	{
		CollisionManifold manifold;
		manifold.Reset();

		const Vector3 closestPoint = other->GetShape<Aabb>()->ClosestPoint(Origin());
		const float distanceSqr = (closestPoint - Origin()).MagnitudeSqr();

		if(distanceSqr > MathF::Squared(Radius()))
		{
			return manifold;
		}

		Vector3 normal;
		if(MathF::IsNearZero(distanceSqr))
		{
			float mSqr = (closestPoint - other->Origin()).MagnitudeSqr();
			if(MathF::IsNearZero(mSqr))
			{
				return manifold;
			}

			normal = (closestPoint - other->Origin()).Normalized();
		}
		else
		{
			normal = (Origin() - closestPoint).Normalized();
		}

		const Vector3 outside = Origin() - normal * Radius();
		const float distance = (closestPoint - outside).Magnitude();

		manifold.colliding = true;
		manifold.contacts.emplace_back(closestPoint + (outside - closestPoint) * .5f);
		manifold.normal = normal;
		manifold.depth = distance * .5f;

		return manifold;
	}
}
