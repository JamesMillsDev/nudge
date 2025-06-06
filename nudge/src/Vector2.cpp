/**
 * @file Vector2.cpp
 * @brief Implementation of the Vector2 class for 2D vector mathematics.
 *
 * This file contains the implementation of all Vector2 class methods including
 * mathematical operations, utility functions, constructors, and operators.
 */

#include "Nudge/Vector2.hpp"

#include <format>
#include <stdexcept>

#include "Nudge/MathF.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Vector4.hpp"

using std::runtime_error;

namespace Nudge
{
	//========================================
	// Static Mathematical Operations
	//========================================

	/**
	 * @brief Calculates the dot product of two vectors.
	 *
	 * The dot product is calculated as: lhs.x * rhs.x + lhs.y * rhs.y
	 * This operation is commutative and returns zero for perpendicular vectors.
	 *
	 * @param lhs The left-hand side vector
	 * @param rhs The right-hand side vector
	 * @return The dot product as a scalar value
	 */
	float Vector2::Dot(const Vector2& lhs, const Vector2& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y;
	}

	/**
	 * @brief Calculates the Euclidean distance between two points.
	 *
	 * Uses the squared distance internally and applies square root for the final result.
	 * For performance-critical applications, consider using DistanceSqr() instead.
	 *
	 * @param lhs The first point
	 * @param rhs The second point
	 * @return The distance between the two points
	 */
	float Vector2::Distance(const Vector2& lhs, const Vector2& rhs)
	{
		return MathF::Sqrt(DistanceSqr(lhs, rhs));
	}

	/**
	 * @brief Calculates the squared distance between two points.
	 *
	 * More efficient than Distance() as it avoids the expensive square root operation.
	 * Useful for distance comparisons where the actual distance value isn't needed.
	 *
	 * @param lhs The first point
	 * @param rhs The second point
	 * @return The squared distance between the two points
	 */
	float Vector2::DistanceSqr(const Vector2& lhs, const Vector2& rhs)
	{
		return (lhs - rhs).MagnitudeSqr();
	}

	/**
	 * @brief Calculates the angle of a vector in radians.
	 *
	 * Uses atan2 to determine the angle from the positive X-axis to the vector.
	 * The angle is measured counter-clockwise from the positive X-axis.
	 *
	 * @param vec The vector to calculate the angle for
	 * @return The angle in radians, range [-PI, PI]
	 */
	float Vector2::AngleOf(const Vector2& vec)
	{
		return MathF::Atan2(vec.y, vec.x);
	}

	/**
	 * @brief Calculates the angle between two vectors in radians.
	 *
	 * Uses the dot product formula: cos(θ) = (a · b) / (|a| × |b|)
	 * Handles edge cases including zero-length vectors and floating-point precision errors.
	 *
	 * @param lhs The first vector
	 * @param rhs The second vector
	 * @return The angle between the vectors in radians, range [0, π]
	 */
	float Vector2::AngleBetween(const Vector2& lhs, const Vector2& rhs)
	{
		const float magProduct = lhs.Magnitude() * rhs.Magnitude();

		// Handle zero-length vectors
		if (MathF::Compare(magProduct, 0.f))
		{
			return 0.f;
		}

		// Clamp the cosine value to prevent floating-point precision errors
		// that could cause acos to return NaN
		return MathF::Acos(MathF::Clamp(Dot(lhs, rhs) / magProduct, -1.f, 1.f));
	}

	/**
	 * @brief Linearly interpolates between two vectors.
	 *
	 * Performs component-wise linear interpolation using the formula:
	 * result = a * (1 - t) + b * t
	 * The parameter t is automatically clamped to the range [0, 1].
	 *
	 * @param a The starting vector (returned when t = 0)
	 * @param b The ending vector (returned when t = 1)
	 * @param t The interpolation parameter, will be clamped to [0, 1]
	 * @return The interpolated vector
	 */
	Vector2 Vector2::Lerp(const Vector2& a, const Vector2& b, float t)
	{
		t = MathF::Clamp01(t);

		return a * (1.f - t) + b * t;
	}

	/**
	 * @brief Reflects a vector off a surface defined by a normal.
	 *
	 * Uses the reflection formula: R = I - 2 * N * (I · N)
	 * where I is the incident direction and N is the surface normal.
	 * The normal should be normalized for correct results.
	 *
	 * @param inDirection The incoming direction vector
	 * @param norm The surface normal vector (should be normalized)
	 * @return The reflected direction vector
	 */
	Vector2 Vector2::Reflect(const Vector2& inDirection, const Vector2& norm)
	{
		return inDirection - 2 * norm * Dot(inDirection, norm);
	}

	/**
	 * @brief Calculates a vector perpendicular to the input vector.
	 *
	 * Rotates the input vector 90 degrees clockwise by swapping components
	 * and negating the x component: (x, y) → (y, -x)
	 *
	 * @param vec The input vector
	 * @return A perpendicular vector (rotated 90 degrees clockwise)
	 */
	Vector2 Vector2::Perpendicular(const Vector2& vec)
	{
		return Vector2{ vec.y, -vec.x };
	}

	/**
	 * @brief Returns a vector with the minimum components of two vectors.
	 *
	 * Performs component-wise minimum selection:
	 * result.x = min(lhs.x, rhs.x), result.y = min(lhs.y, rhs.y)
	 *
	 * @param lhs The first vector
	 * @param rhs The second vector
	 * @return A vector containing the minimum x and y components
	 */
	Vector2 Vector2::Min(const Vector2& lhs, const Vector2& rhs)
	{
		return Vector2{ MathF::Min(lhs.x, rhs.x), MathF::Min(lhs.y, rhs.y) };
	}

	/**
	 * @brief Returns a vector with the maximum components of two vectors.
	 *
	 * Performs component-wise maximum selection:
	 * result.x = max(lhs.x, rhs.x), result.y = max(lhs.y, rhs.y)
	 *
	 * @param lhs The first vector
	 * @param rhs The second vector
	 * @return A vector containing the maximum x and y components
	 */
	Vector2 Vector2::Max(const Vector2& lhs, const Vector2& rhs)
	{
		return Vector2{ MathF::Max(lhs.x, rhs.x), MathF::Max(lhs.y, rhs.y) };
	}

	/**
	 * @brief Clamps a vector's components between minimum and maximum values.
	 *
	 * Performs component-wise clamping to ensure each component falls within
	 * the specified range: min ≤ component ≤ max
	 *
	 * @param value The vector to clamp
	 * @param min The minimum bounds for each component
	 * @param max The maximum bounds for each component
	 * @return The clamped vector
	 */
	Vector2 Vector2::Clamp(const Vector2& value, const Vector2& min, const Vector2& max)
	{
		return Vector2{ MathF::Clamp(value.x, min.x, max.x), MathF::Clamp(value.y, min.y, max.y) };
	}

	//========================================
	// Static Factory Methods
	//========================================

	/**
	 * @brief Creates a zero vector (0, 0).
	 * @return A vector with both components set to zero
	 */
	Vector2 Vector2::Zero()
	{
		return Vector2{ 0.f };
	}

	/**
	 * @brief Creates a vector with all components set to one (1, 1).
	 * @return A vector with both components set to one
	 */
	Vector2 Vector2::One()
	{
		return Vector2{ 1.f };
	}

	/**
	 * @brief Creates a vector with all components set to half (0.5, 0.5).
	 * @return A vector with both components set to 0.5
	 */
	Vector2 Vector2::Half()
	{
		return Vector2{ .5f };
	}

	/**
	 * @brief Creates a unit vector along the X-axis (1, 0).
	 * @return A unit vector pointing in the positive X direction
	 */
	Vector2 Vector2::UnitX()
	{
		return { 1.f, 0.f };
	}

	/**
	 * @brief Creates a unit vector along the Y-axis (0, 1).
	 * @return A unit vector pointing in the positive Y direction
	 */
	Vector2 Vector2::UnitY()
	{
		return { 0.f, 1.f };
	}

	//========================================
	// Constructors
	//========================================

	/**
	 * @brief Default constructor. Creates a zero vector.
	 *
	 * Delegates to the scalar constructor with value 0.0f.
	 */
	Vector2::Vector2()
		: Vector2{ 0.f }
	{
	}

	/**
	 * @brief Constructs a vector with both components set to the same scalar value.
	 *
	 * Delegates to the two-parameter constructor with both values set to scalar.
	 *
	 * @param scalar The value to set for both x and y components
	 */
	Vector2::Vector2(const float scalar)
		: Vector2{ scalar, scalar }
	{
	}

	/**
	 * @brief Constructs a vector with specified x and y components.
	 *
	 * This is the primary constructor that directly initializes the member variables.
	 *
	 * @param x The x component
	 * @param y The y component
	 */
	Vector2::Vector2(const float x, const float y)
		: x{ x }, y{ y }
	{
	}

	/**
	 * @brief Constructs a vector from an array of two float values.
	 *
	 * Assumes the array contains at least 2 elements: [x, y].
	 * Delegates to the two-parameter constructor.
	 *
	 * @param values Array containing [x, y] values
	 */
	Vector2::Vector2(float values[2])
		: Vector2{ values[0], values[1] }
	{
	}

	/**
	 * Constructor from Vector3 - Z component is discarded
	 * @param vec Vector3 to convert, only x and y components are used
	 */
	Vector2::Vector2(const Vector3& vec)
		: Vector2{ vec.x, vec.y }
	{
	}

	/**
	 * Constructor from Vector4 - Z and W components are discarded
	 * @param vec Vector4 to convert, only x and y components are used
	 */
	Vector2::Vector2(const Vector4& vec)
		: Vector2{ vec.x, vec.y }
	{
	}

	/**
	 * @brief Copy constructor.
	 *
	 * Creates a new vector with the same component values as the source vector.
	 *
	 * @param rhs The vector to copy from
	 */
	Vector2::Vector2(const Vector2& rhs)
		: x{ rhs.x }, y{ rhs.y }
	{
	}

	//========================================
	// Instance Methods
	//========================================

	/**
	 * @brief Calculates the magnitude (length) of the vector.
	 *
	 * Uses the Pythagorean theorem: √(x² + y²)
	 * For performance-critical applications, consider using MagnitudeSqr().
	 *
	 * @return The magnitude of the vector
	 */
	float Vector2::Magnitude() const
	{
		return MathF::Sqrt(MagnitudeSqr());
	}

	/**
	 * @brief Calculates the squared magnitude of the vector.
	 *
	 * More efficient than Magnitude() as it avoids the expensive square root operation.
	 * Useful for magnitude comparisons where the actual magnitude value isn't needed.
	 *
	 * @return The squared magnitude (x² + y²)
	 */
	float Vector2::MagnitudeSqr() const
	{
		return x * x + y * y;
	}

	/**
	 * @brief Normalizes this vector to unit length in-place.
	 *
	 * Divides each component by the vector's magnitude to create a unit vector.
	 * If the vector is zero-length, both components are set to zero.
	 * This method modifies the current vector.
	 */
	void Vector2::Normalize()
	{
		if (const float mag = Magnitude(); mag > 0.f)
		{
			x /= mag;
			y /= mag;
		}
		else
		{
			// Handle zero-length vector case
			x = 0.f;
			y = 0.f;
		}
	}

	/**
	 * @brief Returns a normalized copy of this vector.
	 *
	 * Creates a new vector with the same direction but unit length.
	 * The original vector remains unchanged.
	 * Returns a zero vector if the original vector has zero length.
	 *
	 * @return A unit vector in the same direction, or zero vector if original is zero-length
	 */
	Vector2 Vector2::Normalized() const
	{
		const float mag = Magnitude();

		return mag > 0.f ? Vector2{ x / mag, y / mag } : Vector2{ 0.f };
	}

	/**
	 * @brief Checks if the vector is approximately zero.
	 *
	 * Uses MathF::IsNearZero() to handle floating-point precision issues.
	 * Returns true if both components are within the tolerance of zero.
	 *
	 * @return True if both components are close to zero within floating-point tolerance
	 */
	bool Vector2::IsZero() const
	{
		return MathF::IsNearZero(x) && MathF::IsNearZero(y);
	}

	/**
	 * @brief Checks if the vector has unit length.
	 *
	 * Uses MathF::Compare() to check if the magnitude is approximately 1.0
	 * within floating-point tolerance.
	 *
	 * @return True if the magnitude is approximately 1.0 within floating-point tolerance
	 */
	bool Vector2::IsUnit() const
	{
		return MathF::Compare(Magnitude(), 1.f);
	}

	/**
	 * @brief Converts the vector to a string representation.
	 *
	 * Creates a formatted string in the format "(x, y)" using std::format.
	 * Useful for debugging and logging purposes.
	 *
	 * @return A formatted string representation of the vector
	 */
	string Vector2::ToString() const
	{
		return std::format("({}, {})", x, y);
	}

	//========================================
	// Operators
	//========================================

	/**
	 * @brief Stream insertion operator for easy printing.
	 *
	 * Allows Vector2 objects to be directly output to streams using the << operator.
	 * Uses the ToString() method for formatting.
	 *
	 * @param stream The output stream
	 * @param vec The vector to output
	 * @return Reference to the output stream for chaining
	 */
	ostream& operator<<(ostream& stream, const Vector2& vec)
	{
		stream << vec.ToString();

		return stream;
	}

	/**
	 * @brief Equality comparison operator.
	 *
	 * Compares both components using MathF::Compare() to handle floating-point
	 * precision issues. Two vectors are considered equal if both their x and y
	 * components are approximately equal within tolerance.
	 *
	 * @param rhs The vector to compare with
	 * @return True if vectors are approximately equal within floating-point tolerance
	 */
	bool Vector2::operator==(const Vector2& rhs) const
	{
		return MathF::Compare(x, rhs.x) && MathF::Compare(y, rhs.y);
	}

	/**
	 * @brief Inequality comparison operator.
	 *
	 * Returns true if the vectors are not approximately equal.
	 * Uses logical OR to check if either component differs significantly.
	 *
	 * @param rhs The vector to compare with
	 * @return True if vectors are not approximately equal
	 */
	bool Vector2::operator!=(const Vector2& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y);
	}

	/**
	 * @brief Vector addition operator.
	 *
	 * Performs component-wise addition: (x1, y1) + (x2, y2) = (x1+x2, y1+y2)
	 * Creates and returns a new vector; the original vectors remain unchanged.
	 *
	 * @param rhs The vector to add
	 * @return The sum of the two vectors
	 */
	Vector2 Vector2::operator+(const Vector2& rhs) const
	{
		return Vector2{ x + rhs.x, y + rhs.y };
	}

	/**
	 * @brief Vector addition assignment operator.
	 *
	 * Adds the right-hand side vector to this vector in-place.
	 * Includes self-assignment check for safety (though mathematically unnecessary).
	 *
	 * @param rhs The vector to add to this vector
	 * @return Reference to this vector after addition
	 */
	Vector2& Vector2::operator+=(const Vector2& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x += rhs.x;
		y += rhs.y;

		return *this;
	}

	/**
	 * @brief Vector subtraction operator.
	 *
	 * Performs component-wise subtraction: (x1, y1) - (x2, y2) = (x1-x2, y1-y2)
	 * Creates and returns a new vector; the original vectors remain unchanged.
	 *
	 * @param rhs The vector to subtract
	 * @return The difference of the two vectors
	 */
	Vector2 Vector2::operator-(const Vector2& rhs) const
	{
		return Vector2{ x - rhs.x, y - rhs.y };
	}

	/**
	 * @brief Vector subtraction assignment operator.
	 *
	 * Subtracts the right-hand side vector from this vector in-place.
	 * Includes self-assignment check for safety (results in zero vector).
	 *
	 * @param rhs The vector to subtract from this vector
	 * @return Reference to this vector after subtraction
	 */
	Vector2& Vector2::operator-=(const Vector2& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		x -= rhs.x;
		y -= rhs.y;

		return *this;
	}

	/**
	 * @brief Scalar multiplication operator.
	 *
	 * Multiplies each component by the scalar value: (x, y) * s = (x*s, y*s)
	 * Creates and returns a new vector; the original vector remains unchanged.
	 *
	 * @param scalar The scalar value to multiply by
	 * @return The scaled vector
	 */
	Vector2 Vector2::operator*(const float scalar) const
	{
		return Vector2{ x * scalar, y * scalar };
	}

	/**
	 * @brief Scalar multiplication assignment operator.
	 *
	 * Multiplies this vector by the scalar value in-place.
	 * Modifies the current vector and returns a reference to it.
	 *
	 * @param scalar The scalar value to multiply this vector by
	 * @return Reference to this vector after scaling
	 */
	Vector2& Vector2::operator*=(const float scalar)
	{
		x *= scalar;
		y *= scalar;

		return *this;
	}

	/**
	 * @brief Scalar division operator.
	 *
	 * Divides each component by the scalar value: (x, y) / s = (x/s, y/s)
	 * Creates and returns a new vector; the original vector remains unchanged.
	 * Note: No division by zero check is performed.
	 *
	 * @param scalar The scalar value to divide by
	 * @return The scaled vector
	 */
	Vector2 Vector2::operator/(const float scalar) const
	{
		return Vector2{ x / scalar, y / scalar };
	}

	/**
	 * @brief Scalar division assignment operator.
	 *
	 * Divides this vector by the scalar value in-place.
	 * Modifies the current vector and returns a reference to it.
	 * Note: No division by zero check is performed.
	 *
	 * @param scalar The scalar value to divide this vector by
	 * @return Reference to this vector after scaling
	 */
	Vector2& Vector2::operator/=(const float scalar)
	{
		x /= scalar;
		y /= scalar;

		return *this;
	}

	/**
	 * @brief Unary negation operator (modifies the vector in-place).
	 *
	 * Negates both components of the vector: (x, y) becomes (-x, -y)
	 * This operator modifies the current vector rather than creating a new one.
	 *
	 * @return Reference to this vector after negation
	 */
	Vector2& Vector2::operator-()
	{
		x = -x;
		y = -y;

		return *this;
	}

	/**
	 * @brief Component access operator.
	 *
	 * Provides array-like access to vector components:
	 * - Index 0 returns the x component
	 * - Index 1 returns the y component
	 * - Any other index throws a runtime_error
	 *
	 * @param index The component index (0 for x, 1 for y)
	 * @return The component value at the specified index
	 * @throws std::runtime_error if index is out of bounds
	 */
	float Vector2::operator[](const int index) const
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
			default:
				{
					throw runtime_error("Index out of bounds!");
				}
		}
	}

	/**
	 * @brief Copy assignment operator.
	 *
	 * Assigns the values of another vector to this vector.
	 * Includes self-assignment check for efficiency and safety.
	 *
	 * @param rhs The vector to copy from
	 * @return Reference to this vector after assignment
	 */
	Vector2& Vector2::operator=(const Vector2& rhs)
	{
		// Self-assignment check
		if (*this == rhs)
		{
			return *this;
		}

		x = rhs.x;
		y = rhs.y;

		return *this;
	}

	//========================================
	// Global Operators
	//========================================

	/**
	 * @brief Global scalar multiplication operator (scalar * vector).
	 *
	 * Enables multiplication with the scalar on the left-hand side: s * v
	 * Delegates to the member operator* for consistency.
	 *
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply
	 * @return The scaled vector
	 */
	Vector2 operator*(const float lhs, const Vector2& rhs)
	{
		return rhs * lhs;
	}

	/**
	 * @brief Global scalar multiplication assignment operator.
	 *
	 * Multiplies a vector by a scalar and returns the result.
	 * Note: This creates a new vector rather than modifying the input.
	 *
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply (passed by value)
	 * @return The scaled vector
	 */
	Vector2& operator*=(const float lhs, Vector2& rhs)
	{
		rhs.x *= lhs;
		rhs.y *= lhs;

		return rhs;
	}
}
