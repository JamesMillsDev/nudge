#include "Nudge/Physics/Shapes/Triangle.hpp"

namespace Nudge
{
	Triangle::Triangle()
		: Triangle{ Vector3{ -1.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, Vector3{ 1.f, 0.f, 0.f } }
	{
	}

	Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c)
		: a{ a }, b{ b }, c{ c }
	{
	}
}
