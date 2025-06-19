#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

namespace Nudge
{
	class Vector2;
	class Vector4;

	/**
	 * @brief A 3D floating-point vector class providing mathematical operations and utilities.
	 *
	 * The Vector3 class represents a 3-dimensional vector with floating-point components.
	 * It provides comprehensive mathematical operations including dot product, distance calculations,
	 * normalization, interpolation, and standard arithmetic operations.
	 *
	 * The class uses a union to allow access to components as either x/y/z coordinates or r/g/b color values.
	 */
	class Vector3
	{
	public:
		union
		{
			struct
			{
				float x; ///< X component of the vector
				float y; ///< Y component of the vector
				float z; ///< Z component of the vector
			};

			struct
			{
				float r; ///< Red component (alternative access to x)
				float g; ///< Green component (alternative access to y)
				float b; ///< Blue component (alternative access to z)
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
		static float Dot(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Calculates the Euclidean distance between two points.
		 * @param lhs The first point
		 * @param rhs The second point
		 * @return The distance between the two points
		 */
		static float Distance(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Calculates the squared distance between two points.
		 * @param lhs The first point
		 * @param rhs The second point
		 * @return The squared distance (avoids expensive square root calculation)
		 */
		static float DistanceSqr(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Calculates the angle of a vector in radians.
		 * @param vec The vector to calculate the angle for
		 * @return The angle in radians from the positive X-axis, range [-PI, PI]
		 */
		static float AngleOf(const Vector3& vec);

		/**
		 * @brief Calculates the angle between two vectors in radians.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return The angle between the vectors in radians, range [0, PI]
		 */
		static float AngleBetween(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Linearly interpolates between two vectors.
		 * @param a The starting vector
		 * @param b The ending vector
		 * @param t The interpolation parameter, clamped to [0, 1]
		 * @return The interpolated vector
		 */
		static Vector3 Lerp(const Vector3& a, const Vector3& b, float t);

		/**
		 * @brief Reflects a vector off a surface defined by a normal.
		 * @param inDirection The incoming direction vector
		 * @param norm The surface normal vector
		 * @return The reflected direction vector
		 */
		static Vector3 Reflect(const Vector3& inDirection, const Vector3& norm);

		/**
		 * Calculates the cross product of two Vector3 vectors
		 * The cross product produces a vector perpendicular to both input vectors,
		 * with magnitude equal to the area of the parallelogram formed by the two vectors.
		 * The direction follows the right-hand rule.
		 * @param lhs Left-hand side vector
		 * @param rhs Right-hand side vector
		 * @return The cross product vector perpendicular to both input vectors
		 */
		static Vector3 Cross(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Returns a vector with the minimum components of two vectors.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return A vector containing the minimum x and y components
		 */
		static Vector3 Min(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Returns a vector with the maximum components of two vectors.
		 * @param lhs The first vector
		 * @param rhs The second vector
		 * @return A vector containing the maximum x and y components
		 */
		static Vector3 Max(const Vector3& lhs, const Vector3& rhs);

		/**
		 * @brief Clamps a vector's components between minimum and maximum values.
		 * @param value The vector to clamp
		 * @param min The minimum bounds
		 * @param max The maximum bounds
		 * @return The clamped vector
		 */
		static Vector3 Clamp(const Vector3& value, const Vector3& min, const Vector3& max);

		// Static Factory Methods

		/**
		 * @brief Creates a zero vector (0, 0, 0).
		 * @return A vector with both components set to zero
		 */
		static Vector3 Zero();

		/**
		 * @brief Creates a vector with all components set to one (1, 1, 1).
		 * @return A vector with both components set to one
		 */
		static Vector3 One();

		/**
		 * @brief Creates a vector with all components set to half (0.5, 0.5, 0.5).
		 * @return A vector with both components set to 0.5
		 */
		static Vector3 Half();

		/**
		 * @brief Creates a unit vector along the X-axis (1, 0, 0).
		 * @return A unit vector pointing in the positive X direction
		 */
		static Vector3 UnitX();

		/**
		 * @brief Creates a unit vector along the Y-axis (0, 1, 0).
		 * @return A unit vector pointing in the positive Y direction
		 */
		static Vector3 UnitY();

		/**
		 * @brief Creates a unit vector along the Z-axis (0, 0, 1).
		 * @return A unit vector pointing in the positive Z direction
		 */
		static Vector3 UnitZ();

	public:
		// Constructors and Destructor

		/**
		 * @brief Default constructor. Creates a zero vector.
		 */
		Vector3();

		/**
		 * @brief Constructs a vector with both components set to the same scalar value.
		 * @param scalar The value to set for both x, y and z components
		 */
		explicit Vector3(float scalar);

		/**
		 * @brief Constructs a new vector one-dimension higher from the passed one
		 * @param vec The vector to be upgraded from
		 */
		explicit Vector3(const Vector2& vec);

		/**
		 * @brief Constructs a new vector one-dimension lower from the passed one
		 * @param vec The vector to be downgraded from
		 */
		explicit Vector3(const Vector4& vec);

		/**
		 * @brief Constructs a vector with specified x, y and z components.
		 * @param x The x component
		 * @param y The y component
		 * @param z The z component
		 */
		Vector3(float x, float y, float z);

		/**
		 * @brief Constructs a vector from an array of three float values.
		 * @param values Array containing [x, y, z] values
		 */
		explicit Vector3(float values[3]);

		/**
		 * @brief Copy constructor.
		 * @param rhs The vector to copy from
		 */
		Vector3(const Vector3& rhs);

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
		Vector3 Normalized() const;

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
		friend ostream& operator<<(ostream& stream, const Vector3& vec);

		/**
		 * @brief Equality comparison operator.
		 * @param rhs The vector to compare with
		 * @return True if vectors are approximately equal within floating-point tolerance
		 */
		bool operator==(const Vector3& rhs) const;

		/**
		 * @brief Inequality comparison operator.
		 * @param rhs The vector to compare with
		 * @return True if vectors are not approximately equal
		 */
		bool operator!=(const Vector3& rhs) const;

		/**
		 * @brief Vector addition operator.
		 * @param rhs The vector to add
		 * @return The sum of the two vectors
		 */
		Vector3 operator+(const Vector3& rhs) const;

		/**
		 * @brief Vector addition assignment operator.
		 * @param rhs The vector to add to this vector
		 * @return Reference to this vector after addition
		 */
		Vector3& operator+=(const Vector3& rhs);

		/**
		 * @brief Vector subtraction operator.
		 * @param rhs The vector to subtract
		 * @return The difference of the two vectors
		 */
		Vector3 operator-(const Vector3& rhs) const;

		/**
		 * @brief Vector subtraction assignment operator.
		 * @param rhs The vector to subtract from this vector
		 * @return Reference to this vector after subtraction
		 */
		Vector3& operator-=(const Vector3& rhs);

		/**
		 * @brief Scalar multiplication operator.
		 * @param scalar The scalar value to multiply by
		 * @return The scaled vector
		 */
		Vector3 operator*(float scalar) const;

		/**
		 * @brief Scalar multiplication assignment operator.
		 * @param scalar The scalar value to multiply this vector by
		 * @return Reference to this vector after scaling
		 */
		Vector3& operator*=(float scalar);

		/**
		 * @brief Scalar division operator.
		 * @param scalar The scalar value to divide by
		 * @return The scaled vector
		 */
		Vector3 operator/(float scalar) const;

		/**
		 * @brief Scalar division assignment operator.
		 * @param scalar The scalar value to divide this vector by
		 * @return Reference to this vector after scaling
		 */
		Vector3& operator/=(float scalar);

		/**
		 * @brief Unary negation operator (modifies the vector in-place).
		 * @return Reference to this vector after negation
		 */
		Vector3& operator-();

		/**
		 * @brief Component access operator.
		 * @param index The component index (0 for x, 1 for y, 2 for z)
		 * @return The component value at the specified index
		 * @throws std::runtime_error if index is out of bounds
		 */
		float operator[](int index) const;

		/**
		 * @brief Copy assignment operator.
		 * @param rhs The vector to copy from
		 * @return Reference to this vector after assignment
		 */
		Vector3& operator=(const Vector3& rhs);
	};

	// Global Operators

	/**
	 * @brief Global scalar multiplication operator (scalar * vector).
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply
	 * @return The scaled vector
	 */
	Vector3 operator*(float lhs, const Vector3& rhs);

	/**
	 * @brief Global scalar multiplication assignment operator.
	 * @param lhs The scalar value
	 * @param rhs The vector to multiply
	 * @return The scaled vector
	 */
	Vector3& operator*=(float lhs, Vector3& rhs);
}