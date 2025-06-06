#include "Nudge/Quaternion.hpp"

#include "Nudge/MathF.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Matrix4.hpp"
#include "nudge/Vector3.hpp"

namespace Nudge
{
	Quaternion Quaternion::Identity()
	{
		return Quaternion{ 0.f, 0.f, 0.f, 1.f };
	}

	Quaternion Quaternion::FromAxisAngle(const Vector3& axis, float degrees)
	{
		return Quaternion{ axis, degrees };
	}

	Quaternion Quaternion::FromEuler(const Vector3& euler)
	{
		const float pitch = MathF::Radians(euler.x);
		const float yaw = MathF::Radians(euler.y);
		const float roll = MathF::Radians(euler.z);

		const float halfPitchCos = MathF::Cos(pitch * .5f);
		const float halfPitchSin = MathF::Sin(pitch * .5f);

		const float halfYawCos = MathF::Cos(yaw * .5f);
		const float halfYawSin = MathF::Sin(yaw * .5f);

		const float halfRollCos = MathF::Cos(roll * .5f);
		const float halfRollSin = MathF::Sin(roll * .5f);

		// XYZ order: Pitch x Yaw x Roll
		return Quaternion
		{
			halfPitchSin * halfYawCos * halfRollCos + halfPitchCos * halfYawSin * halfRollSin,
			halfPitchCos * halfYawSin * halfRollCos - halfPitchSin * halfYawCos * halfRollSin,
			halfPitchCos * halfYawCos * halfRollSin + halfPitchSin * halfYawSin * halfRollCos,
			halfPitchCos * halfYawCos * halfRollCos - halfPitchSin * halfYawSin * halfRollSin,
		};
	}

	Quaternion Quaternion::FromMatrix(const Matrix3& matrix)
	{
		return { };
	}

	Quaternion Quaternion::FromMatrix(const Matrix4& matrix)
	{
		return { };
	}

	Quaternion Quaternion::FromToRotation(const Vector3& from, const Vector3& to)
	{
		return { };
	}

	Quaternion Quaternion::LookRotation(const Vector3& forward, const Vector3& up)
	{
		return { };
	}

	Quaternion Quaternion::Lerp(Quaternion a, Quaternion b, float t)
	{
		return { };
	}

	Quaternion Quaternion::LerpUnclamped(Quaternion a, Quaternion b, float t)
	{
		return { };
	}

	Quaternion Quaternion::Slerp(Quaternion a, Quaternion b, float t)
	{
		return { };
	}

	Quaternion Quaternion::SlerpUnclamped(Quaternion a, Quaternion b, float t)
	{
		return { };
	}

	Quaternion::Quaternion()
		: Quaternion{ 0.f, 0.f, 0.f, 1.f }
	{
	}

	Quaternion::Quaternion(float x, float y, float z, float w)
		: x{ x }, y{ y }, z{ z }, w{ w }
	{
	}

	Quaternion::Quaternion(Vector3 axis, float degrees)
		: Quaternion{ }
	{
		const float theta = MathF::Radians(degrees);
		axis.Normalize();

		const float halfCos = MathF::Cos(theta / 2.f);
		const float halfSin = MathF::Sin(theta / 2.f);

		w = halfCos;
		x = axis.x * halfSin;
		y = axis.y * halfSin;
		z = axis.z * halfSin;
	}

	Quaternion::Quaternion(const Quaternion& rhs) = default;

	Vector3 Quaternion::Euler() const
	{
		return Vector3
		{
			MathF::Degrees(MathF::Atan2(2.f * (w * x - y * z), 1.f - 2.f * (MathF::Squared(x) + MathF::Squared(y)))),
			MathF::Degrees(MathF::Asin(MathF::Clamp(2.f * (w * y - z * x), -1.f, 1.f))),
			MathF::Degrees(MathF::Atan2(2.f * (w * z - x * y), 1.f - 2.f * (MathF::Squared(y) + MathF::Squared(z))))
		};
	}

	Matrix3 Quaternion::ToMatrix3() const
	{
		return Matrix3
		{
			1.f - 2.f * (MathF::Squared(y) + MathF::Squared(z)),
			2.f * (x * y - w * z),
			2.f * (x * z + w * y),
			2.f * (x * y + w * z),
			1.f - 2.f * (MathF::Squared(x) + MathF::Squared(z)),
			2.f * (y * z - w * x),
			2.f * (x * z - w * y),
			2.f * (y * z + w * x),
			1.f - 2.f * (MathF::Squared(x) + MathF::Squared(y))
		};
	}

	Matrix4 Quaternion::ToMatrix4() const
	{
		return { };
	}

	Quaternion Quaternion::operator*(const Quaternion& rhs)
	{
		return { };
	}

	Vector3 Quaternion::operator*(const Vector3& rhs) const
	{
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

		return Vector3
		{
			(ww + xx - yy - zz) * rhs.x + 2.f * (xy - wz) * rhs.y + 2.f * (xz + wy) * rhs.z,
			2.f * (xy + wz) * rhs.x + (ww - xx + yy - zz) * rhs.y + 2.f * (yz - wx) * rhs.z,
			2.f * (xz - wy) * rhs.x + 2.f * (yz + wx) * rhs.y + (ww - xx - yy + zz) * rhs.z
		};
	}

	bool Quaternion::operator==(const Quaternion& rhs) const
	{
		return MathF::Compare(x, rhs.x, MathF::epsilon) && MathF::Compare(y, rhs.y, MathF::epsilon) &&
		       MathF::Compare(w, rhs.w, MathF::epsilon) && MathF::Compare(w, rhs.w, MathF::epsilon);
	}

	bool Quaternion::operator!=(const Quaternion& rhs) const
	{
		return !MathF::Compare(x, rhs.x) || !MathF::Compare(y, rhs.y) ||
		       !MathF::Compare(z, rhs.z) || !MathF::Compare(w, rhs.w);
	}
}
