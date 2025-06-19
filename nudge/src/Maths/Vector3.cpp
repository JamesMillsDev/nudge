/**
 * @file Vector3.cpp
 * @brief Implementation of the Vector3 class for 3D vector mathematics.
 *
 * This file contains the implementation of all Vector3 class methods including
 * mathematical operations, utility functions, constructors, and operators.
 */

#include "nudge/Maths/Vector3.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Vector2.hpp"
#include "Nudge/Maths/Vector4.hpp"

#include <format>

using std::runtime_error;

namespace Nudge
{
	/**
	 * Calculates the dot product of two Vector3 vectors
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The dot product as a scalar value
	 */
	float Vector3::Dot(const Vector3& lhs, const Vector3& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
	}

	/**
	 * Calculates the Euclidean distance between two Vector3 points
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The distance between the two points
	 */
	float Vector3::Distance(const Vector3& lhs, const Vector3& rhs)
	{
		return MathF::Sqrt(DistanceSqr(lhs, rhs));
	}

	/**
	 * Calculates the squared distance between two Vector3 points
	 * More efficient than Distance() when you only need to compare distances
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The squared distance between the two points
	 */
	float Vector3::DistanceSqr(const Vector3& lhs, const Vector3& rhs)
	{
		return (lhs - rhs).MagnitudeSqr();
	}

	/**
	 * Calculates the angle of a vector from the positive Z-axis
	 * @param vec The vector to calculate the angle for
	 * @return The angle in radians
	 */
	float Vector3::AngleOf(const Vector3& vec)
	{
		const float mag = vec.Magnitude();

		if (MathF::Compare(mag, 0.f))
		{
			return 0.f;
		}

		return MathF::Acos(vec.z / mag);
	}

	/**
	 * Calculates the angle between two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return The angle between the vectors in radians
	 */
	float Vector3::AngleBetween(const Vector3& lhs, const Vector3& rhs)
	{
		const float dot = Dot(lhs, rhs);
		const float lengths = lhs.Magnitude() * rhs.Magnitude();

		if (lengths == 0.0f)
		{
			return 0.0f; // Handle zero vectors
		}

		// Clamp to handle floating-point precision errors
		float cosAngle = dot / lengths;
		cosAngle = MathF::Clamp(cosAngle, -1.0f, 1.0f);

		return MathF::Acos(cosAngle);
	}

	/**
	 * Performs linear interpolation between two Vector3 vectors
	 * @param a Start vector
	 * @param b End vector
	 * @param t Interpolation factor (0.0 to 1.0)
	 * @return The interpolated vector
	 */
	Vector3 Vector3::Lerp(const Vector3& a, const Vector3& b, float t)
	{
		t = MathF::Clamp01(t);

		return a * (1.f - t) + b * t;
	}

	/**
	 * Reflects a vector off a surface with the given normal
	 * @param inDirection The incoming direction vector
	 * @param norm The surface normal vector
	 * @return The reflected direction vector
	 */
	Vector3 Vector3::Reflect(const Vector3& inDirection, const Vector3& norm)
	{
		return inDirection - 2 * norm * Dot(inDirection, norm);
	}

	/**
	 * Calculates the cross product of two Vector3 vectors
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The cross product vector perpendicular to both input vectors
	 */
	Vector3 Vector3::Cross(const Vector3& lhs, const Vector3& rhs)
	{
		return Vector3
		{
			lhs.y * rhs.z - lhs.z * rhs.y,
			lhs.z * rhs.x - lhs.x * rhs.z,
			lhs.x * rhs.y - lhs.y * rhs.x
		};
	}

	/**
	 * Returns a vector with the minimum components of two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return Vector with minimum x, y, z components
	 */
	Vector3 Vector3::Min(const Vector3& lhs, const Vector3& rhs)
	{
		return Vector3{ MathF::Min(lhs.x, rhs.x), MathF::Min(lhs.y, rhs.y), MathF::Min(lhs.z, rhs.z) };
	}

	/**
	 * Returns a vector with the maximum components of two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return Vector with maximum x, y, z components
	 */
	Vector3 Vector3::Max(const Vector3& lhs, const Vector3& rhs)
	{
		return Vector3{ MathF::Max(lhs.x, rhs.x), MathF::Max(lhs.y, rhs.y), MathF::Max(lhs.z, rhs.z) };
	}

	/**
	 * Clamps a vector's components between minimum and maximum values
	 * @param value The vector to clamp
	 * @param min Minimum bounds vector
	 * @param max Maximum bounds vector
	 * @return The clamped vector
	 */
	Vector3 Vector3::Clamp(const Vector3& value, const Vector3& min, const Vector3& max)
	{
		return Vector3
		{
			MathF::Clamp(value.x, min.x, max.x),
			MathF::Clamp(value.y, min.y, max.y),
			MathF::Clamp(value.z, min.z, max.z)
		};
	}

	/**
	 * Returns a zero vector (0, 0, 0)
	 * @return Zero vector
	 */
	Vector3 Vector3::Zero()
	{
		return Vector3{ 0.f };
	}

	/**
	 * Returns a vector with all components set to 1 (1, 1, 1)
	 * @return One vector
	 */
	Vector3 Vector3::One()
	{
		return Vector3{ 1.f };
	}

	/**
	 * Returns a vector with all components set to 0.5 (0.5, 0.5, 0.5)
	 * @return Half vector
	 */
	Vector3 Vector3::Half()
	{
		return Vector3{ .5f };
	}

	/**
	 * Returns the unit vector along the X-axis (1, 0, 0)
	 * @return Unit X vector
	 */
	Vector3 Vector3::UnitX()
	{
		return Vector3{ 1.f, 0.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Y-axis (0, 1, 0)
	 * @return Unit Y vector
	 */
	Vector3 Vector3::UnitY()
	{
		return Vector3{ 0.f, 1.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Z-axis (0, 0, 1)
	 * @return Unit Z vector
	 */
	Vector3 Vector3::UnitZ()
	{
		return Vector3{ 0.f, 0.f, 1.f };
	}

	/**
	 * Default constructor - initializes to zero vector
	 */
	Vector3::Vector3()
		: Vector3{ 0.f }
	{
	}

	/**
	 * Constructor that sets all components to the same scalar value
	 * @param scalar Value to set for all components
	 */
	Vector3::Vector3(float scalar)
		: Vector3{ scalar, scalar, scalar }
	{
	}

	/**
	 * Constructor from Vector2 - Z component is set to 0
	 * @param vec Float2 vector to convert
	 */
	Vector3::Vector3(const Vector2& vec)
		: Vector3{ vec.x, vec.y, 0.f }
	{
	}

	/**
	 * Constructor from Vector4 - W component is discarded
	 * @param vec Float4 vector to convert
	 */
	Vector3::Vector3(const Vector4& vec)
		: Vector3{ vec.x, vec.y, vec.z }
	{
	}

	/**
	 * Constructor with explicit x, y, z components
	 * @param x X component
	 * @param y Y component
	 * @param z Z component
	 */
	Vector3::Vector3(const float x, const float y, const float z)
		: x{ x }, y{ y }, z{ z }
	{
	}

	/**
	 * Constructor from array of floats
	 * @param values Array containing x, y, z values
	 */
	Vector3::Vector3(float values[3])
		: Vector3{ values[0], values[1], values[2] }
	{
	}

	/**
	 * Copy constructor
	 * @param rhs Float3 to copy from
	 */
	Vector3::Vector3(const Vector3& rhs)
		: x{ rhs.x }, y{ rhs.y }, z{ rhs.z }
	{
	}

	/**
	 * Calculates the magnitude (length) of this vector
	 * @return The magnitude of the vector
	 */
	float Vector3::Magnitude() const
	{
		return MathF::Sqrt(MagnitudeSqr());
	}

	/**
	 * Calculates the squared magnitude of this vector
	 * More efficient than Magnitude() when you only need to compare lengths
	 * @return The squared magnitude of the vector
	 */
	float Vector3::MagnitudeSqr() const
	{
		return x * x + y * y + z * z;
	}

	/**
	 * Normalizes this vector to unit length (modifies the original vector)
	 */
	void Vector3::Normalize()
	{
		if (const float mag = Magnitude(); mag > 0.f)
		{
			x /= mag;
			y /= mag;
			z /= mag;
		}
		else
		{
			// Handle zero-length vector case
			x = 0.f;
			y = 0.f;
			z = 0.f;
		}
	}

	/**
	 * Returns a normalized copy of this vector (original vector unchanged)
	 * @return A normalized version of this vector
	 */
	Vector3 Vector3::Normalized() const
	{
		const float mag = Magnitude();

		return mag > 0.f ? Vector3{ x / mag, y / mag, z / mag } : Vector3{ 0.f };
	}

	/**
	 * Checks if this vector is approximately zero
	 * @return True if all components are near zero
	 */
	bool Vector3::IsZero() const
	{
		return MathF::IsNearZero(x) && MathF::IsNearZero(y) && MathF::IsNearZero(z);
	}

	/**
	 * Checks if this vector has unit length (magnitude approximately 1)
	 * @return True if the vector has unit length
	 */
	bool Vector3::IsUnit() const
	{
		return MathF::Compare(Magnitude(), 1.f);
	}

	/**
	 * Returns a string representation of this vector
	 * @return String in format "(x, y, z)"
	 */
	string Vector3::ToString() const
	{
		return std::format("({}, {}, {})", x, y, z);
	}

	/**
	 * Stream output operator for Vector3
	 * @param stream Output stream
	 * @param vec Vector to output
	 * @return Reference to the stream
	 */
	ostream& operator<<(ostream& stream, const Vector3& vec)
	{
		stream << vec.ToString();

		return stream;
	}

	/**
	 * Equality comparison operator (uses floating-point tolerance)
	 * @param rhs Vector to compare with
	 * @return True if vectors are approximately equal
	 */
	bool Vector3::operator==(const Vector3& rhs) const
	{
		return MathF::Compare(x, rhs.x) && MathF::Compare(y, rhs.y) && MathF::Compare(z, rhs.z);
	}

	/**
	 * Inequality comparison operator
	 * @param rhs Vector to compare with
	 * @return True if vectors are not approximately equal
	 */
	bool Vector3::operator!=(const Vector3& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y) || !MathF::Compare(z, rhs.z);
	}

	/**
	 * Vector addition operator
	 * @param rhs Vector to add
	 * @return Sum of the two vectors
	 */
	Vector3 Vector3::operator+(const Vector3& rhs) const
	{
		return Vector3{ x + rhs.x, y + rhs.y, z + rhs.z };
	}

	/**
	 * Vector addition assignment operator
	 * @param rhs Vector to add to this vector
	 * @return Reference to this vector after addition
	 */
	Vector3& Vector3::operator+=(const Vector3& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;

		return *this;
	}

	/**
	 * Vector subtraction operator
	 * @param rhs Vector to subtract
	 * @return Difference of the two vectors
	 */
	Vector3 Vector3::operator-(const Vector3& rhs) const
	{
		return Vector3{ x - rhs.x, y - rhs.y, z - rhs.z };
	}

	/**
	 * Vector subtraction assignment operator
	 * @param rhs Vector to subtract from this vector
	 * @return Reference to this vector after subtraction
	 */
	Vector3& Vector3::operator-=(const Vector3& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;

		return *this;
	}

	/**
	 * Scalar multiplication operator
	 * @param scalar Scalar value to multiply by
	 * @return Vector scaled by the scalar
	 */
	Vector3 Vector3::operator*(float scalar) const
	{
		return Vector3{ x * scalar, y * scalar, z * scalar };
	}

	/**
	 * Scalar multiplication assignment operator
	 * @param scalar Scalar value to multiply by
	 * @return Reference to this vector after scaling
	 */
	Vector3& Vector3::operator*=(float scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;

		return *this;
	}

	/**
	 * Scalar division operator
	 * @param scalar Scalar value to divide by
	 * @return Vector divided by the scalar
	 */
	Vector3 Vector3::operator/(float scalar) const
	{
		return Vector3{ x / scalar, y / scalar, z / scalar };
	}

	/**
	 * Scalar division assignment operator
	 * @param scalar Scalar value to divide by
	 * @return Reference to this vector after division
	 */
	Vector3& Vector3::operator/=(float scalar)
	{
		x /= scalar;
		y /= scalar;
		z /= scalar;

		return *this;
	}

	/**
	 * Unary negation operator (negates all components)
	 * @return Reference to this vector after negation
	 */
	Vector3& Vector3::operator-()
	{
		x = -x;
		y = -y;
		z = -z;

		return *this;
	}

	/**
	 * Array subscript operator for read access
	 * @param index Index (0=x, 1=y, 2=z)
	 * @return Component value at the specified index
	 * @throws runtime_error If index is out of bounds
	 */
	float Vector3::operator[](int index) const
	{
		switch (index)
		{
		case 0:
		{
			return x;
		}
		case 1:
		{
			return y;
		}
		case 2:
		{
			return z;
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * Assignment operator
	 * @param rhs Vector to assign from
	 * @return Reference to this vector after assignment
	 */
	Vector3& Vector3::operator=(const Vector3& rhs)
	{
		// Self-assignment check
		if (*this == rhs)
		{
			return *this;
		}

		x = rhs.x;
		y = rhs.y;
		z = rhs.z;

		return *this;
	}

	/**
	 * Global scalar multiplication operator (scalar * vector)
	 * @param lhs Scalar value
	 * @param rhs Vector to multiply
	 * @return Vector scaled by the scalar
	 */
	Vector3 operator*(float lhs, const Vector3& rhs)
	{
		return rhs * lhs;
	}

	/**
	 * Global scalar multiplication assignment operator (scalar *= vector)
	 * @param lhs Scalar value
	 * @param rhs Vector to multiply and modify
	 * @return Reference to the modified vector
	 */
	Vector3& operator*=(float lhs, Vector3& rhs)
	{
		rhs.x *= lhs;
		rhs.y *= lhs;
		rhs.z *= lhs;

		return rhs;
	}
}