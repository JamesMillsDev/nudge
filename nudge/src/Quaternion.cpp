/**
 * @file Quaternion.cpp
 * @brief Implementation of the Quaternion class for 3D rotations
 * @author JamesMillsAIE
 * @date 2025-06-06
 */

#include "Nudge/Quaternion.hpp"

#include "Nudge/MathF.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Matrix4.hpp"
#include "nudge/Vector3.hpp"

namespace Nudge
{
	/**
	 * @brief Returns an identity quaternion representing no rotation
	 * @return Identity quaternion (0, 0, 0, 1)
	 */
	Quaternion Quaternion::Identity()
	{
		return Quaternion{ 0.f, 0.f, 0.f, 1.f };
	}

	/**
	 * @brief Creates quaternion from axis and angle using the axis-angle constructor
	 * @param axis The rotation axis (should be normalized)
	 * @param degrees The rotation angle in degrees
	 * @return Quaternion representing the rotation
	 */
	Quaternion Quaternion::FromAxisAngle(const Vector3& axis, float degrees)
	{
		return Quaternion{ axis, degrees };
	}

	/**
	 * @brief Creates quaternion from Euler angles using ZYX rotation order
	 * @param euler Euler angles (pitch=X, yaw=Y, roll=Z) in degrees
	 * @return Quaternion representing the combined rotation
	 * @note Uses ZYX order (Roll x Yaw x Pitch) to match Matrix3::Rotation
	 */
	Quaternion Quaternion::FromEuler(const Vector3& euler)
	{
		// Convert degrees to radians
		const float pitch = MathF::Radians(euler.x);
		const float yaw = MathF::Radians(euler.y);
		const float roll = MathF::Radians(euler.z);

		// Calculate half-angle trigonometric values for efficiency
		const float halfPitchCos = MathF::Cos(pitch * .5f);
		const float halfPitchSin = MathF::Sin(pitch * .5f);

		const float halfYawCos = MathF::Cos(yaw * .5f);
		const float halfYawSin = MathF::Sin(yaw * .5f);

		const float halfRollCos = MathF::Cos(roll * .5f);
		const float halfRollSin = MathF::Sin(roll * .5f);

		// ZYX order: Roll x Yaw x Pitch (to match Matrix3::Rotation)
		return Quaternion
		{
			halfRollCos * halfYawCos * halfPitchSin - halfRollSin * halfYawSin * halfPitchCos, // x
			halfRollCos * halfYawSin * halfPitchCos + halfRollSin * halfYawCos * halfPitchSin, // y
			halfRollSin * halfYawCos * halfPitchCos - halfRollCos * halfYawSin * halfPitchSin, // z
			halfRollCos * halfYawCos * halfPitchCos + halfRollSin * halfYawSin * halfPitchSin  // w
		};
	}

	/**
	 * @brief Creates quaternion from 3x3 rotation matrix
	 * @param matrix 3x3 rotation matrix
	 * @return Quaternion representing the same rotation
	 * @note Currently returns identity quaternion (not implemented)
	 */
	Quaternion Quaternion::FromMatrix(const Matrix3& matrix)
	{
		return { };
	}

	/**
	 * @brief Creates quaternion from 4x4 transformation matrix
	 * @param matrix 4x4 transformation matrix (rotation part is extracted)
	 * @return Quaternion representing the rotation component
	 * @note Currently returns identity quaternion (not implemented)
	 */
	Quaternion Quaternion::FromMatrix(const Matrix4& matrix)
	{
		return { };
	}

	/**
	 * @brief Creates quaternion representing rotation from one vector to another
	 * @param from Source vector (will be normalized)
	 * @param to Target vector (will be normalized)
	 * @return Quaternion that rotates from vector to to vector
	 * @note Handles special cases for aligned and opposite vectors
	 */
	Quaternion Quaternion::FromToRotation(Vector3 from, Vector3 to)
	{
		from.Normalize();
		to.Normalize();
		float dot = Vector3::Dot(from, to);

		// Vectors are already aligned (dot product approximately 1)
		if (dot >= 0.999999f)
		{
			return Quaternion(0, 0, 0, 1); // Identity
		}

		// Vectors are opposite (dot product approximately -1)
		if (dot <= -0.999999f)
		{
			// Find any perpendicular vector for 180-degree rotation
			Vector3 axis = Vector3::Cross(Vector3(1, 0, 0), from);
			if (axis.MagnitudeSqr() < 0.000001f)
			{
				axis = Vector3::Cross(Vector3(0, 1, 0), from);
			}

			axis.Normalize();
			return Quaternion(axis.x, axis.y, axis.z, 0); // 180 degree rotation
		}

		// General case: use cross product for axis and calculate w component
		Vector3 cross = Vector3::Cross(from, to);
		float w = sqrt((from.MagnitudeSqr() * to.MagnitudeSqr()) + dot);

		Quaternion q(cross.x, cross.y, cross.z, w);
		return q.Normalized();
	}

	/**
	 * @brief Creates quaternion for looking in a direction with specified up vector
	 * @param forward The forward direction vector (will be normalized)
	 * @param up The up direction vector (used to calculate right vector)
	 * @return Quaternion representing the look rotation
	 * @note Constructs orthonormal basis and converts to quaternion using Shepperd's method
	 */
	Quaternion Quaternion::LookRotation(Vector3 forward, Vector3 up)
	{
		forward.Normalize();

		// Construct orthonormal basis vectors
		Vector3 right = Vector3::Cross(up, forward).Normalized();
		up = Vector3::Cross(forward, right);

		// Create rotation matrix from basis vectors
		Matrix3 m =
		{
			right.x, up.x, forward.x,
			right.y, up.y, forward.y,
			right.z, up.z, forward.z
		};

		// Convert rotation matrix to quaternion using Shepperd's method
		float trace = m[0][0] + m[1][1] + m[2][2];

		float s, x, y, z, w;

		// Choose the largest diagonal element to avoid numerical instability
		if (trace > 0)
		{
			s = MathF::Sqrt(trace + 1.f) * 2.f;
			w = .25f * s;
			x = (m[2][1] - m[1][2]) / s;
			y = (m[0][2] - m[2][0]) / s;
			z = (m[1][0] - m[0][1]) / s;
		}
		else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
		{
			s = MathF::Sqrt(1.f + m[0][0] - m[1][1] - m[2][2]) * 2.f;
			w = (m[2][1] - m[1][2]) / s;
			x = .25f * s;
			y = (m[0][1] + m[1][0]) / s;
			z = (m[0][2] + m[2][0]) / s;
		}
		else if (m[1][1] > m[2][2])
		{
			s = MathF::Sqrt(1.f + m[1][1] - m[0][0] - m[2][2]) * 2.f;
			w = (m[0][2] - m[2][0]) / s;
			x = (m[0][1] + m[1][0]) / s;
			y = .25f * s;
			z = (m[2][1] - m[1][2]) / s;
		}
		else
		{
			s = MathF::Sqrt(1.f + m[2][2] - m[0][0] - m[1][1]) * 2.f;
			w = (m[1][0] - m[0][1]) / s;
			x = (m[0][2] + m[2][0]) / s;
			y = (m[1][2] + m[2][1]) / s;
			z = .25f * s;
		}

		return { x, y, z, w };
	}

	/**
	 * @brief Calculates dot product of two quaternions
	 * @param lhs First quaternion
	 * @param rhs Second quaternion
	 * @return Dot product (sum of component-wise products)
	 */
	float Quaternion::Dot(const Quaternion& lhs, const Quaternion& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
	}

	/**
	 * @brief Linear interpolation between two quaternions with clamped parameter
	 * @param a Start quaternion
	 * @param b End quaternion
	 * @param t Interpolation parameter (clamped to [0,1])
	 * @return Normalized interpolated quaternion
	 * @note Handles quaternion double-cover by negating b if dot product is negative
	 */
	Quaternion Quaternion::Lerp(const Quaternion& a, Quaternion b, float t)
	{
		// Handle quaternion double-cover (q and -q represent same rotation)
		if (Dot(a, b) < 0)
		{
			b = -b;
		}

		t = MathF::Clamp01(t);

		return (a * (1.f - t) + b * t).Normalized();
	}

	/**
	 * @brief Linear interpolation between two quaternions with unclamped parameter
	 * @param a Start quaternion
	 * @param b End quaternion
	 * @param t Interpolation parameter (not clamped)
	 * @return Interpolated quaternion (not normalized)
	 * @note Handles quaternion double-cover by negating b if dot product is negative
	 */
	Quaternion Quaternion::LerpUnclamped(const Quaternion& a, Quaternion b, const float t)
	{
		// Handle quaternion double-cover
		if (Dot(a, b) < 0)
		{
			b = -b;
		}

		return (a * (1.f - t) + b * t);
	}

	/**
	 * @brief Spherical linear interpolation between two quaternions with clamped parameter
	 * @param a Start quaternion
	 * @param b End quaternion
	 * @param t Interpolation parameter (clamped to [0,1])
	 * @return Spherically interpolated quaternion
	 * @note Falls back to linear interpolation if angle is too small to avoid division by zero
	 */
	Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t)
	{
		t = MathF::Clamp01(t);

		// Calculate angle between quaternions
		float angle = MathF::Acos(MathF::Abs(Dot(a, b)));
		float sinAngle = MathF::Sin(angle);

		// Use spherical interpolation if angle is significant
		if (sinAngle > MathF::epsilon)
		{
			float factor1 = MathF::Sin((1.f - t) * angle) / sinAngle;
			float factor2 = MathF::Sin(t * angle) / sinAngle;

			return a * factor1 + b * factor2;
		}

		// Fall back to linear interpolation for small angles
		return Lerp(a, b, t);
	}

	/**
	 * @brief Spherical linear interpolation between two quaternions with unclamped parameter
	 * @param a Start quaternion
	 * @param b End quaternion
	 * @param t Interpolation parameter (not clamped)
	 * @return Spherically interpolated quaternion
	 * @note Falls back to linear interpolation if angle is too small to avoid division by zero
	 */
	Quaternion Quaternion::SlerpUnclamped(const Quaternion& a, const Quaternion& b, const float t)
	{
		// Calculate angle between quaternions
		float angle = MathF::Acos(MathF::Abs(Dot(a, b)));
		float sinAngle = MathF::Sin(angle);

		// Use spherical interpolation if angle is significant
		if (sinAngle > MathF::epsilon)
		{
			float factor1 = MathF::Sin((1.f - t) * angle) / sinAngle;
			float factor2 = MathF::Sin(t * angle) / sinAngle;

			return a * factor1 + b * factor2;
		}

		// Fall back to linear interpolation for small angles
		return Lerp(a, b, t);
	}

	/**
	 * @brief Default constructor - creates identity quaternion
	 * @note Delegates to parameterized constructor with identity values
	 */
	Quaternion::Quaternion()
		: Quaternion{ 0.f, 0.f, 0.f, 1.f }
	{
	}

	/**
	 * @brief Constructor with explicit x, y, z, w components
	 * @param x X component (imaginary part)
	 * @param y Y component (imaginary part)
	 * @param z Z component (imaginary part)
	 * @param w W component (real part)
	 */
	Quaternion::Quaternion(float x, float y, float z, float w)
		: x{ x }, y{ y }, z{ z }, w{ w }
	{
	}

	/**
	 * @brief Constructor from axis and angle
	 * @param axis The rotation axis (will be normalized)
	 * @param degrees The rotation angle in degrees
	 * @note Converts to half-angle representation using trigonometric functions
	 */
	Quaternion::Quaternion(Vector3 axis, float degrees)
		: Quaternion{ }
	{
		const float theta = MathF::Radians(degrees);
		axis.Normalize();

		// Calculate half-angle trigonometric values
		const float halfCos = MathF::Cos(theta / 2.f);
		const float halfSin = MathF::Sin(theta / 2.f);

		// Set quaternion components
		w = halfCos;
		x = axis.x * halfSin;
		y = axis.y * halfSin;
		z = axis.z * halfSin;
	}

	/**
	 * @brief Copy constructor
	 * @param rhs Quaternion to copy from
	 * @note Uses compiler-generated default implementation
	 */
	Quaternion::Quaternion(const Quaternion& rhs) = default;

	/**
	 * @brief Converts quaternion to Euler angles
	 * @return Vector3 containing Euler angles (roll=X, pitch=Y, yaw=Z) in degrees
	 * @note Uses XYZ rotation order and handles gimbal lock singularities
	 */
	Vector3 Quaternion::Euler() const
	{
		// Calculate roll (rotation around X-axis)
		float sinRCosP = 2.f * (w * x + y * z);
		float cosRCosP = 1.f - 2.f * (x * x + y * y);
		float roll = MathF::Atan2(sinRCosP, cosRCosP);

		// Calculate pitch (rotation around Y-axis) with gimbal lock protection
		float sinP = 2.f * (w * y - z * x);
		float pitch;
		if (MathF::Abs(sinP) >= 1.f)
		{
			pitch = (sinP >= 0.f ? 1.f : -1.f) * MathF::pi / 2.f; // Use 90 degrees if out of range
		}
		else
		{
			pitch = MathF::Asin(sinP);
		}

		// Calculate yaw (rotation around Z-axis)
		float sinYCosP = 2.f * (w * z + x * y);
		float cosyCosP = 1.f - 2.f * (y * y + z * z);
		float yaw = MathF::Atan2(sinYCosP, cosyCosP);

		return Vector3
		{
			MathF::Degrees(roll),  // Z
			MathF::Degrees(pitch), // X
			MathF::Degrees(yaw),   // Y  
		};
	}

	/**
	 * @brief Converts quaternion to 3x3 rotation matrix
	 * @return 3x3 rotation matrix representing the same rotation
	 * @note Uses optimized formula to avoid repeated calculations
	 */
	Matrix3 Quaternion::ToMatrix3() const
	{
		return Matrix3
		{
			1.f - 2.f * (MathF::Squared(y) + MathF::Squared(z)), // m00
			2.f * (x * y - w * z),                               // m01
			2.f * (x * z + w * y),                               // m02
			2.f * (x * y + w * z),                               // m10
			1.f - 2.f * (MathF::Squared(x) + MathF::Squared(z)), // m11
			2.f * (y * z - w * x),                               // m12
			2.f * (x * z - w * y),                               // m20
			2.f * (y * z + w * x),                               // m21
			1.f - 2.f * (MathF::Squared(x) + MathF::Squared(y))  // m22
		};
	}

	/**
	 * @brief Converts quaternion to 4x4 transformation matrix
	 * @return 4x4 transformation matrix with rotation component
	 * @note Delegates to Matrix4 constructor that takes Matrix3
	 */
	Matrix4 Quaternion::ToMatrix4() const
	{
		return Matrix4
		{
			ToMatrix3()
		};
	}

	/**
	 * @brief Returns the magnitude (length) of the quaternion
	 * @return Magnitude of the quaternion
	 * @note Calculates square root of sum of squared components
	 */
	float Quaternion::Magnitude() const
	{
		return MathF::Sqrt(MagnitudeSqr());
	}

	/**
	 * @brief Returns the squared magnitude of the quaternion
	 * @return Squared magnitude (avoids expensive sqrt calculation)
	 * @note More efficient then Magnitude() when only comparison is needed
	 */
	float Quaternion::MagnitudeSqr() const
	{
		return x * x + y * y + z * z + w * w;
	}

	/**
	 * @brief Normalizes this quaternion to unit length in-place
	 * @note Sets quaternion to zero if magnitude is zero to avoid division by zero
	 */
	void Quaternion::Normalize()
	{
		float mag = Magnitude();

		if (mag > 0)
		{
			x /= mag;
			y /= mag;
			z /= mag;
			w /= mag;
		}
		else
		{
			// Handle zero quaternion case
			x = 0.f;
			y = 0.f;
			z = 0.f;
			w = 0.f;
		}
	}

	/**
	 * @brief Returns a normalized copy of this quaternion
	 * @return Normalized quaternion (unit length)
	 * @note Returns zero quaternion if original magnitude is zero
	 */
	Quaternion Quaternion::Normalized() const
	{
		float mag = Magnitude();

		return mag > 0.f ? Quaternion{ x / mag, y / mag, z / mag, w / mag } : Quaternion{ 0.f, 0.f, 0.f, 0.f };
	}

	/**
	 * @brief Adds two quaternions component-wise
	 * @param rhs Right-hand side quaternion
	 * @return Sum of the two quaternions
	 * @note Component-wise addition (not rotation composition)
	 */
	Quaternion Quaternion::operator+(const Quaternion& rhs) const
	{
		return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w };
	}

	/**
	 * @brief Multiplies two quaternions (rotation composition)
	 * @param rhs Right-hand side quaternion
	 * @return Product quaternion representing combined rotation
	 * @note Uses Hamilton's quaternion multiplication formula
	 */
	Quaternion Quaternion::operator*(const Quaternion& rhs) const
	{
		// Hamilton's quaternion multiplication formula
		return Quaternion
		{
			w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y, // x component
			w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x, // y component  
			w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w, // z component
			w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z  // w component
		};
	}

	/**
	 * @brief Rotates a vector by this quaternion
	 * @param rhs Vector to rotate
	 * @return Rotated vector
	 * @note Uses optimized rotation formula: v' = q * v * q^-1
	 */
	Vector3 Quaternion::operator*(const Vector3& rhs) const
	{
		// Precompute squared components for efficiency
		float ww = w * w;
		float xx = x * x;
		float yy = y * y;
		float zz = z * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;

		// Apply rotation using optimized formula
		return Vector3
		{
			(ww + xx - yy - zz) * rhs.x + 2.f * (xy - wz) * rhs.y + 2.f * (xz + wy) * rhs.z,
			2.f * (xy + wz) * rhs.x + (ww - xx + yy - zz) * rhs.y + 2.f * (yz - wx) * rhs.z,
			2.f * (xz - wy) * rhs.x + 2.f * (yz + wx) * rhs.y + (ww - xx - yy + zz) * rhs.z
		};
	}

	/**
	 * @brief Scales quaternion by scalar value
	 * @param rhs Scalar multiplier
	 * @return Scaled quaternion
	 * @note Scales all components uniformly
	 */
	Quaternion Quaternion::operator*(float rhs) const
	{
		return { x * rhs, y * rhs, z * rhs, w * rhs };
	}

	/**
	 * @brief Negates the quaternion (same rotation, opposite representation)
	 * @return Reference to this quaternion after negation
	 * @note Quaternions q and -q represent the same rotation
	 */
	Quaternion& Quaternion::operator-()
	{
		x = -x;
		y = -y;
		z = -z;
		w = -w;

		return *this;
	}

	/**
	 * @brief Tests equality between two quaternions
	 * @param rhs Right-hand side quaternion to compare
	 * @return True if quaternions are equal within epsilon tolerance, false otherwise
	 * @note Uses epsilon comparison for floating-point tolerance
	 */
	bool Quaternion::operator==(const Quaternion& rhs) const
	{
		return MathF::Compare(x, rhs.x, MathF::epsilon) && MathF::Compare(y, rhs.y, MathF::epsilon) &&
			MathF::Compare(z, rhs.z, MathF::epsilon) && MathF::Compare(w, rhs.w, MathF::epsilon);
	}

	/**
	 * @brief Tests inequality between two quaternions
	 * @param rhs Right-hand side quaternion to compare
	 * @return True if quaternions are not equal, false otherwise
	 * @note Uses epsilon comparison for floating-point tolerance
	 */
	bool Quaternion::operator!=(const Quaternion& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y) ||
			!MathF::Compare(z, rhs.z) || !MathF::Compare(w, rhs.w);
	}

	/**
	 * @brief Scales quaternion by scalar value (scalar * quaternion)
	 * @param lhs Scalar multiplier
	 * @param rhs Quaternion to scale
	 * @return Scaled quaternion
	 * @note Commutative operation - delegates to quaternion * scalar
	 */
	Quaternion operator*(float lhs, const Quaternion& rhs)
	{
		return rhs * lhs;
	}
}