/**
 * @file Vector4.cpp
 * @brief Implementation of the Vector4 class for 4D vector mathematics.
 *
 * This file contains the implementation of all Vector4 class methods including
 * mathematical operations, utility functions, constructors, and operators.
 */

#include "Nudge/Vector4.hpp"

#include <format>

#include "Nudge/MathF.hpp"
#include "Nudge/Vector2.hpp"
#include "Nudge/Vector3.hpp"

using std::runtime_error;

namespace Nudge
{
	/**
	 * Calculates the dot product of two Vector4 vectors
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The dot product as a scalar value
	 */
	float Vector4::Dot(const Vector4& lhs, const Vector4& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}

	/**
	 * Calculates the Euclidean distance between two Vector4 points
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The distance between the two points
	 */
	float Vector4::Distance(const Vector4& lhs, const Vector4& rhs)
	{
		return sqrtf(DistanceSqr(lhs, rhs));
	}

	/**
	 * Calculates the squared distance between two Vector4 points
	 * More efficient than Distance() when you only need to compare distances
	 * @param lhs First point
	 * @param rhs Second point
	 * @return The squared distance between the two points
	 */
	float Vector4::DistanceSqr(const Vector4& lhs, const Vector4& rhs)
	{
		return (lhs - rhs).MagnitudeSqr();
	}

	/**
	 * Calculates the angle of a vector from the positive Z-axis (using only x, y, z components)
	 * Note: This treats the Vector4 as a 3D vector for angle calculation, ignoring the w component
	 * @param vec The vector to calculate the angle for
	 * @return The angle in radians
	 */
	float Vector4::AngleOf(const Vector4& vec)
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
	float Vector4::AngleBetween(const Vector4& lhs, const Vector4& rhs)
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
	 * Performs linear interpolation between two Vector4 vectors
	 * @param a Start vector
	 * @param b End vector
	 * @param t Interpolation factor (0.0 to 1.0)
	 * @return The interpolated vector
	 */
	Vector4 Vector4::Lerp(const Vector4& a, const Vector4& b, float t)
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
	Vector4 Vector4::Reflect(const Vector4& inDirection, const Vector4& norm)
	{
		return inDirection - 2 * norm * Dot(inDirection, norm);
	}

	/**
	 * Calculates the cross product treating Vector4 as 3D vectors (ignoring w component)
	 * The resulting w component is set to 0
	 * @param lhs Left-hand side vector
	 * @param rhs Right-hand side vector
	 * @return The cross product vector with w = 0
	 */
	Vector4 Vector4::Cross(const Vector4& lhs, const Vector4& rhs)
	{
		return Vector4
		{
			lhs.y * rhs.z - lhs.z * rhs.y,
			lhs.z * rhs.x - lhs.x * rhs.z,
			lhs.x * rhs.y - lhs.y * rhs.x,
			0.f
		};
	}

	/**
	 * Returns a vector with the minimum components of two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return Vector with minimum x, y, z, w components
	 */
	Vector4 Vector4::Min(const Vector4& lhs, const Vector4& rhs)
	{
		return Vector4
		{
			fminf(lhs.x, rhs.x),
			fminf(lhs.y, rhs.y),
			fminf(lhs.z, rhs.z),
			fminf(lhs.w, rhs.w)
		};
	}

	/**
	 * Returns a vector with the maximum components of two vectors
	 * @param lhs First vector
	 * @param rhs Second vector
	 * @return Vector with maximum x, y, z, w components
	 */
	Vector4 Vector4::Max(const Vector4& lhs, const Vector4& rhs)
	{
		return Vector4
		{
			fmaxf(lhs.x, rhs.x),
			fmaxf(lhs.y, rhs.y),
			fmaxf(lhs.z, rhs.z),
			fmaxf(lhs.w, rhs.w)
		};
	}

	/**
	 * Clamps a vector's components between minimum and maximum values
	 * @param value The vector to clamp
	 * @param min Minimum bounds vector
	 * @param max Maximum bounds vector
	 * @return The clamped vector
	 */
	Vector4 Vector4::Clamp(const Vector4& value, const Vector4& min, const Vector4& max)
	{
		return Vector4
		{
			MathF::Clamp(value.x, min.x, max.x),
			MathF::Clamp(value.y, min.y, max.y),
			MathF::Clamp(value.z, min.z, max.z),
			MathF::Clamp(value.w, min.w, max.w)
		};
	}

	/**
	 * Returns a zero vector (0, 0, 0, 0)
	 * @return Zero vector
	 */
	Vector4 Vector4::Zero()
	{
		return Vector4{ 0.f };
	}

	/**
	 * Returns a vector with all components set to 1 (1, 1, 1, 1)
	 * @return One vector
	 */
	Vector4 Vector4::One()
	{
		return Vector4{ 1.f };
	}

	/**
	 * Returns a vector with all components set to 0.5 (0.5, 0.5, 0.5, 0.5)
	 * @return Half vector
	 */
	Vector4 Vector4::Half()
	{
		return Vector4{ .5f };
	}

	/**
	 * Returns the unit vector along the X-axis (1, 0, 0, 0)
	 * @return Unit X vector
	 */
	Vector4 Vector4::UnitX()
	{
		return Vector4{ 1.f, 0.f, 0.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Y-axis (0, 1, 0, 0)
	 * @return Unit Y vector
	 */
	Vector4 Vector4::UnitY()
	{
		return Vector4{ 0.f, 1.f, 0.f, 0.f };
	}

	/**
	 * Returns the unit vector along the Z-axis (0, 0, 1, 0)
	 * @return Unit Z vector
	 */
	Vector4 Vector4::UnitZ()
	{
		return Vector4{ 0.f, 0.f, 1.f, 0.f };
	}

	/**
	 * Default constructor - initializes to zero vector
	 */
	Vector4::Vector4()
		: Vector4{ 0.f }
	{
	}

	/**
	 * Constructor that sets all components to the same scalar value
	 * @param scalar Value to set for all components
	 */
	Vector4::Vector4(float scalar)
		: Vector4{ scalar, scalar, scalar, scalar }
	{
	}

	/**
	 * Constructor from Vector2 - Z and W components are set to 0
	 * @param vec Vector2 to convert
	 */
	Vector4::Vector4(const Vector2& vec)
		: Vector4{ vec.x, vec.y, 0.f, 0.f }
	{
	}

	/**
	 * Constructor from Vector3 - W component is set to 0
	 * @param vec Vector3 to convert
	 */
	Vector4::Vector4(const Vector3& vec)
		: Vector4{ vec.x, vec.y, vec.z, 0.f }
	{
	}

	/**
	 * Constructor with explicit x, y, z, w components
	 * @param x X component
	 * @param y Y component
	 * @param z Z component
	 * @param w W component
	 */
	Vector4::Vector4(float x, float y, float z, float w)
		: x{ x }, y{ y }, z{ z }, w{ w }
	{
	}

	/**
	 * Constructor from array of floats
	 * @param values Array containing x, y, z, w values
	 */
	Vector4::Vector4(float values[4])
		: Vector4{ values[0], values[1], values[2], values[3] }
	{
	}

	/**
	 * Copy constructor
	 * @param rhs Vector4 to copy from
	 */
	Vector4::Vector4(const Vector4& rhs)
		: x{ rhs.x }, y{ rhs.y }, z{ rhs.z }, w{ rhs.w }
	{
	}

	/**
	 * Calculates the magnitude (length) of this vector
	 * @return The magnitude of the vector
	 */
	float Vector4::Magnitude() const
	{
		return sqrtf(MagnitudeSqr());
	}

	/**
	 * Calculates the squared magnitude of this vector
	 * More efficient than Magnitude() when you only need to compare lengths
	 * @return The squared magnitude of the vector
	 */
	float Vector4::MagnitudeSqr() const
	{
		return x * x + y * y + z * z + w * w;
	}

	/**
	 * Normalizes this vector to unit length (modifies the original vector)
	 */
	void Vector4::Normalize()
	{
		if (const float mag = Magnitude(); mag > 0.f)
		{
			x /= mag;
			y /= mag;
			z /= mag;
			w /= mag;
		}
		else
		{
			// Handle zero-length vector case
			x = 0.f;
			y = 0.f;
			z = 0.f;
			w = 0.f;
		}
	}

	/**
	 * Returns a normalized copy of this vector (original vector unchanged)
	 * @return A normalized version of this vector
	 */
	Vector4 Vector4::Normalized() const
	{
		const float mag = Magnitude();

		return mag > 0.f ? Vector4{ x / mag, y / mag, z / mag, w / mag } : Vector4{ 0.f };
	}

	/**
	 * Checks if this vector is approximately zero
	 * @return True if all components are near zero
	 */
	bool Vector4::IsZero() const
	{
		return MathF::IsNearZero(x) && MathF::IsNearZero(y) && MathF::IsNearZero(z) && MathF::IsNearZero(w);
	}

	/**
	 * Checks if this vector has unit length (magnitude approx 1)
	 * @return True if the vector has unit length
	 */
	bool Vector4::IsUnit() const
	{
		return MathF::Compare(Magnitude(), 1.f);
	}

	/**
	 * Returns a string representation of this vector
	 * @return String in format "(x, y, z, w)"
	 */
	string Vector4::ToString() const
	{
		return std::format("({}, {}, {}, {})", x, y, z, w);
	}

	/**
	 * Stream output operator for Vector4
	 * @param stream Output stream
	 * @param vec Vector to output
	 * @return Reference to the stream
	 */
	ostream& operator<<(ostream& stream, const Vector4& vec)
	{
		stream << vec.ToString();

		return stream;
	}

	/**
	 * Equality comparison operator (uses floating-point tolerance)
	 * @param rhs Vector to compare with
	 * @return True if vectors are approximately equal
	 */
	bool Vector4::operator==(const Vector4& rhs) const
	{
		return MathF::Compare(x, rhs.x) && MathF::Compare(y, rhs.y) &&
			MathF::Compare(z, rhs.z) && MathF::Compare(w, rhs.w);
	}

	/**
	 * Inequality comparison operator
	 * Note: There appears to be a logical error in the original implementation
	 * @param rhs Vector to compare with
	 * @return True if vectors are not approximately equal
	 */
	bool Vector4::operator!=(const Vector4& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y) ||
			!MathF::Compare(z, rhs.z) || !MathF::Compare(w, rhs.w);
	}

	/**
	 * Vector addition operator
	 * @param rhs Vector to add
	 * @return Sum of the two vectors
	 */
	Vector4 Vector4::operator+(const Vector4& rhs) const
	{
		return Vector4{ x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w };
	}

	/**
	 * Vector addition assignment operator
	 * @param rhs Vector to add to this vector
	 * @return Reference to this vector after addition
	 */
	Vector4& Vector4::operator+=(const Vector4& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		w += rhs.w;

		return *this;
	}

	/**
	 * Vector subtraction operator
	 * @param rhs Vector to subtract
	 * @return Difference of the two vectors
	 */
	Vector4 Vector4::operator-(const Vector4& rhs) const
	{
		return Vector4{ x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w };
	}

	/**
	 * Vector subtraction assignment operator
	 * @param rhs Vector to subtract from this vector
	 * @return Reference to this vector after subtraction
	 */
	Vector4& Vector4::operator-=(const Vector4& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		w -= rhs.w;

		return *this;
	}

	/**
	 * Scalar multiplication operator
	 * @param scalar Scalar value to multiply by
	 * @return Vector scaled by the scalar
	 */
	Vector4 Vector4::operator*(float scalar) const
	{
		return Vector4{ x * scalar, y * scalar, z * scalar, w * scalar };
	}

	/**
	 * Scalar multiplication assignment operator
	 * @param scalar Scalar value to multiply by
	 * @return Reference to this vector after scaling
	 */
	Vector4& Vector4::operator*=(float scalar)
	{
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;

		return *this;
	}

	/**
	 * Scalar division operator
	 * @param scalar Scalar value to divide by
	 * @return Vector divided by the scalar
	 */
	Vector4 Vector4::operator/(float scalar) const
	{
		return Vector4{ x / scalar, y / scalar, z / scalar, w / scalar };
	}

	/**
	 * Scalar division assignment operator
	 * @param scalar Scalar value to divide by
	 * @return Reference to this vector after division
	 */
	Vector4& Vector4::operator/=(float scalar)
	{
		x /= scalar;
		y /= scalar;
		z /= scalar;
		w /= scalar;

		return *this;
	}

	/**
	 * Unary negation operator (negates all components)
	 * @return Reference to this vector after negation
	 */
	Vector4& Vector4::operator-()
	{
		x = -x;
		y = -y;
		z = -z;
		w = -w;

		return *this;
	}

	/**
	 * Array subscript operator for read access
	 * Note: Missing case for index 3 (w component) - this is a bug
	 * @param index Index (0=x, 1=y, 2=z, 3=w)
	 * @return Component value at the specified index
	 * @throws runtime_error If index is out of bounds
	 */
	float Vector4::operator[](int index) const
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
		case 3:
		{
			return w;  // This case is missing in the original code
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
	Vector4& Vector4::operator=(const Vector4& rhs)
	{
		// Self-assignment check
		if (*this == rhs)
		{
			return *this;
		}

		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;

		return *this;
	}

	/**
	 * Global scalar multiplication operator (scalar * vector)
	 * @param lhs Scalar value
	 * @param rhs Vector to multiply
	 * @return Vector scaled by the scalar
	 */
	Vector4 operator*(float lhs, const Vector4& rhs)
	{
		return rhs * lhs;
	}

	/**
	 * Global scalar multiplication assignment operator (scalar *= vector)
	 * @param lhs Scalar value
	 * @param rhs Vector to multiply and modify
	 * @return Reference to the modified vector
	 */
	Vector4& operator*=(float lhs, Vector4& rhs)
	{
		rhs.x *= lhs;
		rhs.y *= lhs;
		rhs.z *= lhs;
		rhs.w *= lhs;

		return rhs;
	}
}
