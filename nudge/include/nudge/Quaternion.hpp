#pragma once

namespace Nudge
{
	class Vector3;
	class Matrix3;
	class Matrix4;

	/**
	 * @brief Represents a quaternion for 3D rotations
	 *
	 * Quaternions provide a compact and efficient way to represent rotations in 3D space.
	 * They avoid gimbal lock and provide smooth interpolation between rotations.
	 */
	class Quaternion
	{
	public:
		/** @brief Quaternion components: x, y, z (imaginary), w (real) */
		float x, y, z, w;

	public:
		/**
		 * @brief Returns an identity quaternion (no rotation)
		 * @return Identity quaternion representing no rotation
		 */
		static Quaternion Identity();

		/**
		 * @brief Creates quaternion from axis and angle
		 * @param axis The rotation axis (should be normalized)
		 * @param degrees The rotation angle in degrees
		 * @return Quaternion representing the rotation
		 */
		static Quaternion FromAxisAngle(const Vector3& axis, float degrees);

		/**
		 * @brief Creates quaternion from Euler angles
		 * @param euler Euler angles (pitch, yaw, roll) in degrees
		 * @return Quaternion representing the rotation
		 */
		static Quaternion FromEuler(const Vector3& euler);

		/**
		 * @brief Creates quaternion from 3x3 rotation matrix
		 * @param matrix 3x3 rotation matrix
		 * @return Quaternion representing the same rotation
		 */
		static Quaternion FromMatrix(const Matrix3& matrix);

		/**
		 * @brief Creates quaternion from 4x4 transformation matrix
		 * @param matrix 4x4 transformation matrix (rotation part is extracted)
		 * @return Quaternion representing the rotation component
		 */
		static Quaternion FromMatrix(const Matrix4& matrix);

		/**
		 * @brief Creates quaternion representing rotation from one vector to another
		 * @param from Source vector
		 * @param to Target vector
		 * @return Quaternion that rotates from vector to to vector
		 */
		static Quaternion FromToRotation(Vector3 from, Vector3 to);

		/**
		 * @brief Creates quaternion for looking in a direction with specified up vector
		 * @param forward The forward direction vector
		 * @param up The up direction vector
		 * @return Quaternion representing the look rotation
		 */
		static Quaternion LookRotation(Vector3 forward, Vector3 up);

		/**
		 * @brief Calculates dot product of two quaternions
		 * @param lhs First quaternion
		 * @param rhs Second quaternion
		 * @return Dot product value
		 */
		static float Dot(const Quaternion& lhs, const Quaternion& rhs);

		/**
		 * @brief Linear interpolation between two quaternions (clamped t)
		 * @param a Start quaternion
		 * @param b End quaternion
		 * @param t Interpolation parameter (clamped to [0,1])
		 * @return Interpolated quaternion
		 */
		static Quaternion Lerp(const Quaternion& a, Quaternion b, float t);

		/**
		 * @brief Linear interpolation between two quaternions (unclamped t)
		 * @param a Start quaternion
		 * @param b End quaternion
		 * @param t Interpolation parameter (not clamped)
		 * @return Interpolated quaternion
		 */
		static Quaternion LerpUnclamped(const Quaternion& a, Quaternion b, float t);

		/**
		 * @brief Spherical linear interpolation between two quaternions (clamped t)
		 * @param a Start quaternion
		 * @param b End quaternion
		 * @param t Interpolation parameter (clamped to [0,1])
		 * @return Spherically interpolated quaternion
		 */
		static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t);

		/**
		 * @brief Spherical linear interpolation between two quaternions (unclamped t)
		 * @param a Start quaternion
		 * @param b End quaternion
		 * @param t Interpolation parameter (not clamped)
		 * @return Spherically interpolated quaternion
		 */
		static Quaternion SlerpUnclamped(const Quaternion& a, const Quaternion& b, float t);

	public:
		/**
		 * @brief Default constructor - creates identity quaternion
		 */
		Quaternion();

		/**
		 * @brief Constructor with explicit x, y, z, w components
		 * @param x X component (imaginary part)
		 * @param y Y component (imaginary part)
		 * @param z Z component (imaginary part)
		 * @param w W component (real part)
		 */
		Quaternion(float x, float y, float z, float w);

		/**
		 * @brief Constructor from axis and angle
		 * @param axis The rotation axis (should be normalized)
		 * @param degrees The rotation angle in degrees
		 */
		Quaternion(Vector3 axis, float degrees);

		/**
		 * @brief Copy constructor
		 * @param rhs Quaternion to copy from
		 */
		Quaternion(const Quaternion& rhs);

	public:
		/**
		 * @brief Converts quaternion to Euler angles
		 * @return Vector3 containing Euler angles (pitch, yaw, roll) in degrees
		 */
		Vector3 Euler() const;

		/**
		 * @brief Converts quaternion to 3x3 rotation matrix
		 * @return 3x3 rotation matrix representing the same rotation
		 */
		Matrix3 ToMatrix3() const;

		/**
		 * @brief Converts quaternion to 4x4 transformation matrix
		 * @return 4x4 transformation matrix with rotation component
		 */
		Matrix4 ToMatrix4() const;

		/**
		 * @brief Returns the magnitude (length) of the quaternion
		 * @return Magnitude of the quaternion
		 */
		float Magnitude() const;

		/**
		 * @brief Returns the squared magnitude of the quaternion
		 * @return Squared magnitude (avoids expensive sqrt calculation)
		 */
		float MagnitudeSqr() const;

		/**
		 * @brief Normalizes this quaternion to unit length in-place
		 */
		void Normalize();

		/**
		 * @brief Returns a normalized copy of this quaternion
		 * @return Normalized quaternion (unit length)
		 */
		Quaternion Normalized() const;

	public:
		/**
		 * @brief Adds two quaternions component-wise
		 * @param rhs Right-hand side quaternion
		 * @return Sum of the two quaternions
		 */
		Quaternion operator+(const Quaternion& rhs) const;

		/**
		 * @brief Multiplies two quaternions (rotation composition)
		 * @param rhs Right-hand side quaternion
		 * @return Product quaternion representing combined rotation
		 */
		Quaternion operator*(const Quaternion& rhs) const;

		/**
		 * @brief Rotates a vector by this quaternion
		 * @param rhs Vector to rotate
		 * @return Rotated vector
		 */
		Vector3 operator*(const Vector3& rhs) const;

		/**
		 * @brief Scales quaternion by scalar value
		 * @param rhs Scalar multiplier
		 * @return Scaled quaternion
		 */
		Quaternion operator*(float rhs) const;

		/**
		 * @brief Negates the quaternion (same rotation, opposite representation)
		 * @return Reference to this quaternion after negation
		 */
		Quaternion& operator-();

		/**
		 * @brief Tests equality between two quaternions
		 * @param rhs Right-hand side quaternion to compare
		 * @return True if quaternions are equal, false otherwise
		 */
		bool operator==(const Quaternion& rhs) const;

		/**
		 * @brief Tests inequality between two quaternions
		 * @param rhs Right-hand side quaternion to compare
		 * @return True if quaternions are not equal, false otherwise
		 */
		bool operator!=(const Quaternion& rhs) const;

	};

	/**
	 * @brief Scales quaternion by scalar value (scalar * quaternion)
	 * @param lhs Scalar multiplier
	 * @param rhs Quaternion to scale
	 * @return Scaled quaternion
	 */
	Quaternion operator*(float lhs, const Quaternion& rhs);
}