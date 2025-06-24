#include "Nudge/Colliders/Collider.hpp"

#include "Nudge/Shapes/Shape.hpp"

namespace Nudge
{
	Collider::Collider(Shape* shape)
		: m_shape{ shape }
	{
	}

	Collider::~Collider()
	{
		delete m_shape;
		m_shape = nullptr;
	}
}
