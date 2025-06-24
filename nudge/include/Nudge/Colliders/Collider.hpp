#pragma once

namespace Nudge
{
    class Shape;

    class Collider
    {
    public:
        template<class T>
        T const* GetShape() const;

    protected:
        Shape* m_shape;

    protected:
        Collider(Shape* shape);
        virtual ~Collider();

    protected:
        template<class T>
        T* As() const;

    };

    template <class T>
    T const* Collider::GetShape() const
    {
        return dynamic_cast<T*>(m_shape);
    }

    template <class T>
    T* Collider::As() const
    {
        return dynamic_cast<T*>(m_shape);
    }
}
