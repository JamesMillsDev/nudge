#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

namespace Nudge
{
	/**
	 * @brief A 2D floating-point vector class providing mathematical operations and utilities.
	 *
	 * The Float2 class represents a 2-dimensional vector with floating-point components.
	 * It provides comprehensive mathematical operations including dot product, distance calculations,
	 * normalization, interpolation, and standard arithmetic operations.
	 *
	 * The class uses a union to allow access to components as either x/y coordinates or r/g color values.
	 */
	class Float2
	{
	public:
		union
		{
			struct
			{
				float x; ///< X component of the vector
				float y; ///< Y component of the vector
			};

			struct
			{
				float r; ///< Red component (alternative access to x)
				float g; ///< Green component (alternative access to y)
			};
		};

	public:
		// Static Mathematical Operations

		/**
		 * @brief Calculates the dot product of two vectors.
		 * @param lhs The left-hand side vector
		 * @param rhs The right-hand side vector
		 * @return The dot product as a scalar value
		 */
		static float Dot(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Calculates the Euclidean distance between two points.
		 * @param lhs The first point
		 * @param rhs The second point
		 * @return The distance between the two points
		 */
		static float Distance(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Calculates the squared distance between two points.
		 * @param lhs The first point
		 * @param rhs The second point
		 * @return The squared distance (avoids expensive square root calculation)
		 */
		static float DistanceSqr(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Calculates the angle of a vector in radians.
		 * @param vec The vector to calculate the angle for
		 * @return The angle in radians from the positive X-axis, range [-π, π]
		 */
		static float AngleOf(const Float2& vec);

		/**
		 * @brief Calculates the angle between two vectors in radians.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return The angle between the vectors in radians, range [0, π]
		 */
		static float AngleBetween(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Linearly interpolates between two vectors.
		 * @param a The starting vector
		 * @param b The ending vector
		 * @param t The interpolation parameter, clamped to [0, 1]
		 * @return The interpolated vector
		 */
		static Float2 Lerp(const Float2& a, const Float2& b, float t);

		/**
		 * @brief Reflects a vector off a surface defined by a normal.
		 * @param inDirection The incoming direction vector
		 * @param norm The surface normal vector
		 * @return The reflected direction vector
		 */
		static Float2 Reflect(const Float2& inDirection, const Float2& norm);

		/**
		 * @brief Calculates a vector perpendicular to the input vector.
		 * @param vec The input vector
		 * @return A perpendicular vector (rotated 90 degrees clockwise)
		 */
		static Float2 Perpendicular(const Float2& vec);

		/**
		 * @brief Returns a vector with the minimum components of two vectors.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return A vector containing the minimum x and y components
		 */
		static Float2 Min(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Returns a vector with the maximum components of two vectors.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return A vector containing the maximum x and y components
		 */
		static Float2 Max(const Float2& lhs, const Float2& rhs);

		/**
		 * @brief Clamps a vector's components between minimum and maximum values.
		 * @param value The vector to clamp
		 * @param min The minimum bounds
		 * @param max The maximum bounds
		 * @return The clamped vector
		 */
		static Float2 Clamp(const Float2& value, const Float2& min, const Float2& max);

		// Static Factory Methods

		/**
		 * @brief Creates a zero vector (0, 0).
		 * @return A vector with both components set to zero
		 */
		static Float2 Zero();

		/**
		 * @brief Creates a vector with all components set to one (1, 1).
		 * @return A vector with both components set to one
		 */
		static Float2 One();

		/**
		 * @brief Creates a vector with all components set to half (0.5, 0.5).
		 * @return A vector with both components set to 0.5
		 */
		static Float2 Half();

		/**
		 * @brief Creates a unit vector along the X-axis (1, 0).
		 * @return A unit vector pointing in the positive X direction
		 */
		static Float2 UnitX();

		/**
		 * @brief Creates a unit vector along the Y-axis (0, 1).
		 * @return A unit vector pointing in the positive Y direction
		 */
		static Float2 UnitY();

	public:
		// Constructors and Destructor

		/**
		 * @brief Default constructor. Creates a zero vector.
		 */
		Float2();

		/**
		 * @brief Constructs a vector with both components set to the same scalar value.
		 * @param scalar The value to set for both x and y components
		 */
		explicit Float2(float scalar);

		/**
		 * @brief Constructs a vector with specified x and y components.
		 * @param x The x component
		 * @param y The y component
		 */
		Float2(float x, float y);

		/**
		 * @brief Constructs a vector from an array of two float values.
		 * @param values Array containing [x, y] values
		 */
		explicit Float2(float values[2]);

		/**
		 * @brief Copy constructor.
		 * @param rhs The vector to copy from
		 */
		Float2(const Float2& rhs);

	public:
		// Instance Methods

		/**
		 * @brief Calculates the magnitude (length) of the vector.
		 * @return The magnitude of the vector
		 */
		float Magnitude() const;

		/**
		 * @brief Calculates the squared magnitude of the vector.
		 * @return The squared magnitude (avoids expensive square root calculation)
		 */
		float MagnitudeSqr() const;

		/**
		 * @brief Normalizes this vector to unit length in-place.
		 * If the vector is zero-length, it remains unchanged.
		 */
		void Normalize();

		/**
		 * @brief Returns a normalized copy of this vector.
		 * @return A unit vector in the same direction, or zero vector if original is zero-length
		 */
		Float2 Normalized() const;

		/**
		 * @brief Checks if the vector is approximately zero.
		 * @return True if both components are close to zero within floating-point tolerance
		 */
		bool IsZero() const;

		/**
		 * @brief Checks if the vector has unit length.
		 * @return True if the magnitude is approximately 1.0 within floating-point tolerance
		 */
		bool IsUnit() const;

		/**
		 * @brief Converts the vector to a string representation.
		 * @return A formatted string representation of the vector
		 */
		string ToString() const;

	public:
		// Operators

		/**
		 * @brief Stream insertion operator for easy printing.
		 * @param stream The output stream
		 * @param vec The vector to output
		 * @return Reference to the output stream
		 */
		friend ostream& operator<<(ostream& stream, const Float2& vec);

		/**
		 * @brief Equality comparison operator.
		 * @param rhs The vector to compare with
		 * @return True if vectors are approximately equal within floating-point tolerance
		 */
		bool operator==(const Float2& rhs) const;

		/**
		 * @brief Inequality comparison operator.
		 * @param rhs The vector to compare with
		 * @return True if vectors are not approximately equal
		 */
		bool operator!=(const Float2& rhs) const;

		/**
		 * @brief Vector addition operator.
		 * @param rhs The vector to add
		 * @return The sum of the two vectors
		 */
		Float2 operator+(const Float2& rhs) const;

		/**
		 * @brief Vector addition assignment operator.
		 * @param rhs The vector to add to this vector
		 * @return Reference to this vector after addition
		 */
		Float2& operator+=(const Float2& rhs);

		/**
		 * @brief Vector subtraction operator.
		 * @param rhs The vector to subtract
		 * @return The difference of the two vectors
		 */
		Float2 operator-(const Float2& rhs) const;

		/**
		 * @brief Vector subtraction assignment operator.
		 * @param rhs The vector to subtract from this vector
		 * @return Reference to this vector after subtraction
		 */
		Float2& operator-=(const Float2& rhs);

		/**
		 * @brief Scalar multiplication operator.
		 * @param scalar The scalar value to multiply by
		 * @return The scaled vector
		 */
		Float2 operator*(float scalar) const;

		/**
		 * @brief Scalar multiplication assignment operator.
		 * @param scalar The scalar value to multiply this vector by
		 * @return Reference to this vector after scaling
		 */
		Float2& operator*=(float scalar);

		/**
		 * @brief Scalar division operator.
		 * @param scalar The scalar value to divide by
		 * @return The scaled vector
		 */
		Float2 operator/(float scalar) const;

		/**
		 * @brief Scalar division assignment operator.
		 * @param scalar The scalar value to divide this vector by
		 * @return Reference to this vector after scaling
		 */
		Float2& operator/=(float scalar);

		/**
		 * @brief Unary negation operator (modifies the vector in-place).
		 * @return Reference to this vector after negation
		 */
		Float2& operator-();

		/**
		 * @brief Component access operator.
		 * @param index The component index (0 for x, 1 for y)
		 * @return The component value at the specified index
		 * @throws std::runtime_error if index is out of bounds
		 */
		float operator[](int index) const;

		/**
		 * @brief Copy assignment operator.
		 * @param rhs The vector to copy from
		 * @return Reference to this vector after assignment
		 */
		Float2& operator=(const Float2& rhs);
	};

	// Global Operators

	/**
	 * @brief Global scalar multiplication operator (scalar * vector).
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply
	 * @return The scaled vector
	 */
	Float2 operator*(float lhs, const Float2& rhs);

	/**
	 * @brief Global scalar multiplication assignment operator.
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply
	 * @return The scaled vector
	 */
	Float2& operator*=(float lhs, Float2 rhs);
}