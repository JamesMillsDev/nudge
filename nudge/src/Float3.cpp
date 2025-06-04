/**
 * @file Float3.cpp
 * @brief Implementation of the Float3 class for 3D vector mathematics.
 *
 * This file contains the implementation of all Float3 class methods including
 * mathematical operations, utility functions, constructors, and operators.
 */

#include "nudge/Float3.hpp"

#include <format>

#include "nudge/Float2.hpp"
#include "nudge/Float4.hpp"
#include "nudge/MathF.hpp"

using std::runtime_error;

namespace Nudge
{
	/**
	 * Calculates the dot product of two Float3 vectors
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The dot product as a scalar value
	 */
	float Float3::Dot(const Float3& lhs, const Float3& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
	}

	/**
	 * Calculates the Euclidean distance between two Float3 points
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The distance between the two points
	 */
	float Float3::Distance(const Float3& lhs, const Float3& rhs)
	{
		return sqrtf(DistanceSqr(lhs, rhs));
	}

	/**
	 * Calculates the squared distance between two Float3 points
	 * More efficient than Distance() when you only need to compare distances
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The squared distance between the two points
	 */
	float Float3::DistanceSqr(const Float3& lhs, const Float3& rhs)
	{
		return (lhs - rhs).MagnitudeSqr();
	}

	/**
	 * Calculates the angle of a vector from the positive Z-axis
	 * @param vec The vector to calculate the angle for
	 * @return The angle in radians
	 */
	float Float3::AngleOf(const Float3& vec)
	{
		const float mag = vec.Magnitude();

		if (MathF::Compare(mag, 0.f))
		{
			return 0.f;
		}

		return acos(vec.z / mag);
	}

	/**
	 * Calculates the angle between two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return The angle between the vectors in radians
	 */
	float Float3::AngleBetween(const Float3& lhs, const Float3& rhs)
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

		return acos(cosAngle);
	}

	/**
	 * Performs linear interpolation between two Float3 vectors
	 * @param a Start vector
	 * @param b End vector
	 * @param t Interpolation factor (0.0 to 1.0)
	 * @return The interpolated vector
	 */
	Float3 Float3::Lerp(const Float3& a, const Float3& b, float t)
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
	Float3 Float3::Reflect(const Float3& inDirection, const Float3& norm)
	{
		return inDirection - 2 * norm * Dot(inDirection, norm);
	}

	/**
	 * Calculates the cross product of two Float3 vectors
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The cross product vector perpendicular to both input vectors
	 */
	Float3 Float3::Cross(const Float3& lhs, const Float3& rhs)
	{
		return Float3
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
	Float3 Float3::Min(const Float3& lhs, const Float3& rhs)
	{
		return Float3{ fminf(lhs.x, rhs.x), fminf(lhs.y, rhs.y), fminf(lhs.z, rhs.z) };
	}

	/**
	 * Returns a vector with the maximum components of two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return Vector with maximum x, y, z components
	 */
	Float3 Float3::Max(const Float3& lhs, const Float3& rhs)
	{
		return Float3{ fmaxf(lhs.x, rhs.x), fmaxf(lhs.y, rhs.y), fmaxf(lhs.z, rhs.z) };
	}

	/**
	 * Clamps a vector's components between minimum and maximum values
	 * @param value The vector to clamp
	 * @param min Minimum bounds vector
	 * @param max Maximum bounds vector
	 * @return The clamped vector
	 */
	Float3 Float3::Clamp(const Float3& value, const Float3& min, const Float3& max)
	{
		return Float3
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
	Float3 Float3::Zero()
	{
		return Float3{ 0.f };
	}

	/**
	 * Returns a vector with all components set to 1 (1, 1, 1)
	 * @return One vector
	 */
	Float3 Float3::One()
	{
		return Float3{ 1.f };
	}

	/**
	 * Returns a vector with all components set to 0.5 (0.5, 0.5, 0.5)
	 * @return Half vector
	 */
	Float3 Float3::Half()
	{
		return Float3{ .5f };
	}

	/**
	 * Returns the unit vector along the X-axis (1, 0, 0)
	 * @return Unit X vector
	 */
	Float3 Float3::UnitX()
	{
		return Float3{ 1.f, 0.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Y-axis (0, 1, 0)
	 * @return Unit Y vector
	 */
	Float3 Float3::UnitY()
	{
		return Float3{ 0.f, 1.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Z-axis (0, 0, 1)
	 * @return Unit Z vector
	 */
	Float3 Float3::UnitZ()
	{
		return Float3{ 0.f, 0.f, 1.f };
	}

	/**
	 * Default constructor - initializes to zero vector
	 */
	Float3::Float3()
		: Float3{ 0.f }
	{
	}

	/**
	 * Constructor that sets all components to the same scalar value
	 * @param scalar Value to set for all components
	 */
	Float3::Float3(float scalar)
		: Float3{ scalar, scalar, scalar }
	{
	}

	/**
	 * Constructor from Float2 - Z component is set to 0
	 * @param vec Float2 vector to convert
	 */
	Float3::Float3(const Float2& vec)
		: Float3{ vec.x, vec.y, 0.f }
	{
	}

	/**
	 * Constructor from Float4 - W component is discarded
	 * @param vec Float4 vector to convert
	 */
	Float3::Float3(const Float4& vec)
		: Float3{ /*vec.x, vec.y, vec.z*/ }
	{
	}

	/**
	 * Constructor with explicit x, y, z components
	 * @param x X component
	 * @param y Y component
	 * @param z Z component
	 */
	Float3::Float3(const float x, const float y, const float z)
		: x{ x }, y{ y }, z{ z }
	{
	}

	/**
	 * Constructor from array of floats
	 * @param values Array containing x, y, z values
	 */
	Float3::Float3(float values[3])
		: Float3{ values[0], values[1], values[2] }
	{
	}

	/**
	 * Copy constructor
	 * @param rhs Float3 to copy from
	 */
	Float3::Float3(const Float3& rhs)
		: x{ rhs.x }, y{ rhs.y }, z{ rhs.z }
	{
	}

	/**
	 * Calculates the magnitude (length) of this vector
	 * @return The magnitude of the vector
	 */
	float Float3::Magnitude() const
	{
		return sqrtf(MagnitudeSqr());
	}

	/**
	 * Calculates the squared magnitude of this vector
	 * More efficient than Magnitude() when you only need to compare lengths
	 * @return The squared magnitude of the vector
	 */
	float Float3::MagnitudeSqr() const
	{
		return x * x + y * y + z * z;
	}

	/**
	 * Normalizes this vector to unit length (modifies the original vector)
	 */
	void Float3::Normalize()
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
	Float3 Float3::Normalized() const
	{
		const float mag = Magnitude();

		return mag > 0.f ? Float3{ x / mag, y / mag, z / mag } : Float3{ 0.f };
	}

	/**
	 * Checks if this vector is approximately zero
	 * @return True if all components are near zero
	 */
	bool Float3::IsZero() const
	{
		return MathF::IsNearZero(x) && MathF::IsNearZero(y) && MathF::IsNearZero(z);
	}

	/**
	 * Checks if this vector has unit length (magnitude approximately 1)
	 * @return True if the vector has unit length
	 */
	bool Float3::IsUnit() const
	{
		return MathF::Compare(Magnitude(), 1.f);
	}

	/**
	 * Returns a string representation of this vector
	 * @return String in format "(x, y, z)"
	 */
	string Float3::ToString() const
	{
		return std::format("({}, {}, {})", x, y, z);
	}

	/**
	 * Stream output operator for Float3
	 * @param stream Output stream
	 * @param vec Vector to output
	 * @return Reference to the stream
	 */
	ostream& operator<<(ostream& stream, const Float3& vec)
	{
		stream << vec.ToString();

		return stream;
	}

	/**
	 * Equality comparison operator (uses floating-point tolerance)
	 * @param rhs Vector to compare with
	 * @return True if vectors are approximately equal
	 */
	bool Float3::operator==(const Float3& rhs) const
	{
		return MathF::Compare(x, rhs.x) && MathF::Compare(y, rhs.y) && MathF::Compare(z, rhs.z);
	}

	/**
	 * Inequality comparison operator
	 * @param rhs Vector to compare with
	 * @return True if vectors are not approximately equal
	 */
	bool Float3::operator!=(const Float3& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y) || !MathF::Compare(z, rhs.z);
	}

	/**
	 * Vector addition operator
	 * @param rhs Vector to add
	 * @return Sum of the two vectors
	 */
	Float3 Float3::operator+(const Float3& rhs) const
	{
		return Float3{ x + rhs.x, y + rhs.y, z + rhs.z };
	}

	/**
	 * Vector addition assignment operator
	 * @param rhs Vector to add to this vector
	 * @return Reference to this vector after addition
	 */
	Float3& Float3::operator+=(const Float3& rhs)
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
	Float3 Float3::operator-(const Float3& rhs) const
	{
		return Float3{ x - rhs.x, y - rhs.y, z - rhs.z };
	}

	/**
	 * Vector subtraction assignment operator
	 * @param rhs Vector to subtract from this vector
	 * @return Reference to this vector after subtraction
	 */
	Float3& Float3::operator-=(const Float3& rhs)
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
	Float3 Float3::operator*(float scalar) const
	{
		return Float3{ x * scalar, y * scalar, z * scalar };
	}

	/**
	 * Scalar multiplication assignment operator
	 * @param scalar Scalar value to multiply by
	 * @return Reference to this vector after scaling
	 */
	Float3& Float3::operator*=(float scalar)
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
	Float3 Float3::operator/(float scalar) const
	{
		return Float3{ x / scalar, y / scalar, z / scalar };
	}

	/**
	 * Scalar division assignment operator
	 * @param scalar Scalar value to divide by
	 * @return Reference to this vector after division
	 */
	Float3& Float3::operator/=(float scalar)
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
	Float3& Float3::operator-()
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
	float Float3::operator[](int index) const
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
	Float3& Float3::operator=(const Float3& rhs)
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
	Float3 operator*(float lhs, const Float3& rhs)
	{
		return rhs * lhs;
	}

	/**
	 * Global scalar multiplication assignment operator (scalar *= vector)
	 * @param lhs Scalar value
	 * @param rhs Vector to multiply and modify
	 * @return Reference to the modified vector
	 */
	Float3& operator*=(float lhs, Float3& rhs)
	{
		rhs.x *= lhs;
		rhs.y *= lhs;
		rhs.z *= lhs;

		return rhs;
	}
}