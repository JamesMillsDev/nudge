#pragma once

namespace Nudge
{
	class Vector3;
	class Matrix3;
	class Matrix4;

	class Quaternion
	{
	public:
		float x, y, z, w;

	public:
		static Quaternion Identity();
		static Quaternion FromAxisAngle(const Vector3& axis, float degrees);
		static Quaternion FromEuler(const Vector3& euler);
		static Quaternion FromMatrix(const Matrix3& matrix);
		static Quaternion FromMatrix(const Matrix4& matrix);
		static Quaternion FromToRotation(const Vector3& from, const Vector3& to);
		static Quaternion LookRotation(const Vector3& forward, const Vector3& up);

		static Quaternion Lerp(Quaternion a, Quaternion b, float t);
		static Quaternion LerpUnclamped(Quaternion a, Quaternion b, float t);
		static Quaternion Slerp(Quaternion a, Quaternion b, float t);
		static Quaternion SlerpUnclamped(Quaternion a, Quaternion b, float t);

	public:
		Quaternion();
		Quaternion(float x, float y, float z, float w);
		Quaternion(Vector3 axis, float degrees);
		Quaternion(const Quaternion& rhs);

	public:
		Vector3 Euler() const;
		Matrix3 ToMatrix3() const;
		Matrix4 ToMatrix4() const;

	public:
		Quaternion operator*(const Quaternion& rhs);
		Vector3 operator*(const Vector3& rhs) const;

		bool operator==(const Quaternion& rhs) const;
		bool operator!=(const Quaternion& rhs) const;

	};
}
