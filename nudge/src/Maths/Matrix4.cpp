/**
 * @file Matrix4.cpp
 * @brief Implementation of the Matrix4 class for 4x4 matrix operations
 *
 * This file contains the implementation of a 4x4 matrix class with support for
 * common mathematical operations including transformations, matrix arithmetic,
 * and linear algebra operations. The matrix uses column-major storage order.
 */

#include "Nudge/Maths/Matrix4.hpp"
#include "Nudge/Maths/MathF.hpp"
#include "Nudge/Maths/Matrix3.hpp"
#include "Nudge/Maths/Vector3.hpp"
#include "Nudge/Maths/Vector4.hpp"

#include <format>

using std::runtime_error;

namespace Nudge
{
	// ===== Static Factory Methods =====

	/**
	 * @brief Creates a 4x4 identity matrix
	 * @return Matrix4 Identity matrix with 1s on the diagonal and 0s elsewhere
	 */
	Matrix4 Matrix4::Identity()
	{
		return Matrix4{ 1.f };
	}

	/**
	 * @brief Creates a 4x4 zero matrix
	 * @return Matrix4 Matrix with all elements set to 0
	 */
	Matrix4 Matrix4::Zero()
	{
		return Matrix4{ 0.f };
	}

	/**
	 * @brief Creates a 4x4 scale transformation matrix
	 * @param sx Scale factor along X-axis
	 * @param sy Scale factor along Y-axis
	 * @param sz Scale factor along Z-axis
	 * @return Matrix4 Scale transformation matrix
	 */
	Matrix4 Matrix4::Scale(float sx, float sy, float sz)
	{
		return Matrix4
		{
			sx, 0.f, 0.f, 0.f,
			0.f, sy, 0.f, 0.f,
			0.f, 0.f, sz, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a 4x4 scale transformation matrix from a Vector3
	 * @param scale Vector containing scale factors for each axis
	 * @return Matrix4 Scale transformation matrix
	 */
	Matrix4 Matrix4::Scale(const Vector3& scale)
	{
		return Scale(scale.x, scale.y, scale.z);
	}

	/**
	 * @brief Creates a rotation matrix around the X-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix4 Rotation matrix for X-axis rotation
	 */
	Matrix4 Matrix4::RotationX(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		// Standard X-axis rotation matrix
		return Matrix4
		{
			1.f, 0.f, 0.f, 0.f,
			0.f, cosTheta, -sinTheta, 0.f,
			0.f, sinTheta, cosTheta, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a rotation matrix around the Y-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix4 Rotation matrix for Y-axis rotation
	 */
	Matrix4 Matrix4::RotationY(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		// Standard Y-axis rotation matrix
		return Matrix4
		{
			cosTheta, 0.f, sinTheta, 0.f,
			0.f, 1.f, 0.f, 0.f,
			-sinTheta, 0.f, cosTheta, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a rotation matrix around the Z-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix4 Rotation matrix for Z-axis rotation
	 */
	Matrix4 Matrix4::RotationZ(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		// Standard Z-axis rotation matrix
		return Matrix4
		{
			cosTheta, -sinTheta, 0.f, 0.f,
			sinTheta, cosTheta, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a composite rotation matrix from Euler angles
	 * @param euler Vector containing rotation angles (x, y, z) in degrees
	 * @return Matrix4 Composite rotation matrix (Z * Y * X order)
	 */
	Matrix4 Matrix4::Rotation(const Vector3& euler)
	{
		// Apply rotations in Z-Y-X order (common for game engines)
		return RotationZ(euler.z) * RotationY(euler.y) * RotationX(euler.x);
	}

	/**
	 * @brief Creates a rotation matrix around an arbitrary axis using Rodrigues' formula
	 * @param axis The axis of rotation (will be normalized)
	 * @param degrees Rotation angle in degrees
	 * @return Matrix4 Rotation matrix for arbitrary axis rotation
	 */
	Matrix4 Matrix4::Rotation(const Vector3& axis, float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		const float oneMinusCos = 1 - cosTheta;

		// Normalize the rotation axis
		const Vector3 n = axis.Normalized();

		// Rodrigues' rotation formula in matrix form
		return Matrix4
		{
			MathF::Squared(n.x) * oneMinusCos + cosTheta,
			n.y * n.x * oneMinusCos - n.z * sinTheta,
			n.z * n.x * oneMinusCos + n.y * sinTheta,
			0.f,
			n.y * n.x * oneMinusCos + n.z * sinTheta,
			MathF::Squared(n.y) * oneMinusCos + cosTheta,
			n.z * n.y * oneMinusCos - n.x * sinTheta,
			0.f,
			n.x * n.z * oneMinusCos - n.y * sinTheta,
			n.y * n.z * oneMinusCos + n.x * sinTheta,
			MathF::Squared(n.z) * oneMinusCos + cosTheta,
			0.f,
			0.f,
			0.f,
			0.f,
			1.f
		};
	}

	/**
	 * @brief Creates a 4x4 translation matrix
	 * @param tx Translation along X-axis
	 * @param ty Translation along Y-axis
	 * @param tz Translation along Z-axis
	 * @return Matrix4 Translation matrix
	 */
	Matrix4 Matrix4::Translation(float tx, float ty, float tz)
	{
		// Translation values go in the 4th column (m14, m24, m34)
		return Matrix4
		{
			1.f, 0.f, 0.f, tx,
			0.f, 1.f, 0.f, ty,
			0.f, 0.f, 1.f, tz,
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a 4x4 translation matrix from a Vector3
	 * @param translation Vector containing translation values
	 * @return Matrix4 Translation matrix
	 */
	Matrix4 Matrix4::Translation(const Vector3& translation)
	{
		return Translation(translation.x, translation.y, translation.z);
	}

	/**
	 * @brief Creates a view matrix for camera positioning (currently unimplemented)
	 * @param eye Camera position
	 * @param target Target position to look at
	 * @param up Up vector for camera orientation
	 * @return Matrix4 View matrix (currently returns default constructed matrix)
	 */
	Matrix4 Matrix4::LookAt(const Vector3& eye, const Vector3& target, const Vector3& up)
	{
		Vector3 forward = (target - eye).Normalized();
		Vector3 right = Vector3::Cross(forward, up);
		Vector3 correctedUp = Vector3::Cross(right, forward);

		return
		{
			right.x, correctedUp.x, -forward.x, -Vector3::Dot(right, eye),
			right.y, correctedUp.y, -forward.y, -Vector3::Dot(correctedUp, eye),
			right.z, correctedUp.z, -forward.z, Vector3::Dot(forward, eye),
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a perspective projection matrix (currently unimplemented)
	 * @param fovY Field of view in Y direction (degrees)
	 * @param aspectRatio Aspect ratio (width/height)
	 * @param nearPlane Near clipping plane distance
	 * @param farPlane Far clipping plane distance
	 * @return Matrix4 Perspective projection matrix (currently returns default constructed matrix)
	 */
	Matrix4 Matrix4::Perspective(float fovY, float aspectRatio, float nearPlane, float farPlane)
	{
		float f = 1.f / MathF::Tan(fovY / 2.f);
		float aspect = aspectRatio;
		float near = nearPlane;
		float far = farPlane;

		return Matrix4
		{
			f / aspect, 0.f, 0.f, 0.f,
			0.f, f, 0.f, 0.f,
			0.f, 0.f, (far + near) / (near - far), (2.f * far * near) / (near - far),
			0.f, 0.f, -1.f, 0.f
		};
	}

	/**
	 * @brief Creates an orthographic projection matrix (currently unimplemented)
	 * @param left Left clipping plane
	 * @param right Right clipping plane
	 * @param bottom Bottom clipping plane
	 * @param top Top clipping plane
	 * @param nearPlane Near clipping plane
	 * @param farPlane Far clipping plane
	 * @return Matrix4 Orthographic projection matrix (currently returns default constructed matrix)
	 */
	Matrix4 Matrix4::Orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane)
	{
		float near = nearPlane;
		float far = farPlane;

		return Matrix4
		{
			2.f / (right - left), 0.f, 0.f, -(right + left) / (right - left),
			0.f, 2.f / (top - bottom), 0.f, -(top + bottom) / (top - bottom),
			0.f, 0.f, -2.f / (far - near), -(far + near) / (far - near),
			0.f, 0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a combined transformation matrix from Translation, Rotation, and Scale
	 * @param translation Translation vector
	 * @param rotation Rotation Euler angles in degrees
	 * @param scale Scale factors
	 * @return Matrix4 Combined TRS transformation matrix
	 */
	Matrix4 Matrix4::TRS(const Vector3& translation, const Vector3& rotation, const Vector3& scale)
	{
		// Apply transformations in T * R * S order
		return Translation(translation) * Rotation(rotation) * Scale(scale);
	}

	// ===== Constructors =====

	/**
	 * @brief Default constructor - creates an identity matrix
	 */
	Matrix4::Matrix4()
		: Matrix4{ 1.f }
	{
	}

	/**
	 * @brief Scalar constructor - creates a matrix with scalar value on diagonal
	 * @param scalar Value to place on the main diagonal
	 */
	Matrix4::Matrix4(float scalar)
		: Matrix4{
			scalar, 0.f, 0.f, 0.f,
			0.f, scalar, 0.f, 0.f,
			0.f, 0.f, scalar, 0.f,
			0.f, 0.f, 0.f, scalar
		}
	{
	}

	/**
	 * @brief Element-wise constructor (row-major parameter order, column-major storage)
	 * @param m11,m12,m13,m14,m21,m22,m23,m24,m31,m32,m33,m34,m41,m42,m43,m44 Matrix elements
	 * @note Parameters are in intuitive row-major order but stored in column-major format
	 */
	Matrix4::Matrix4(float m11, float m12, float m13, float m14, float m21, float m22, float m23, float m24, float m31,
		float m32, float m33, float m34, float m41, float m42, float m43, float m44)
		: m11{ m11 }, m21{ m21 }, m31{ m31 }, m41{ m41 }, m12{ m12 }, m22{ m22 }, m32{ m32 }, m42{ m42 }, m13{ m13 },
		m23{ m23 }, m33{ m33 }, m43{ m43 }, m14{ m14 }, m24{ m24 }, m34{ m34 }, m44{ m44 }
	{
	}

	/**
	 * @brief Constructor from four column vectors
	 * @param col1,col2,col3,col4 Column vectors for the matrix
	 */
	Matrix4::Matrix4(const Vector4& col1, const Vector4& col2, const Vector4& col3, const Vector4& col4)
		: Matrix4{
			col1.x, col2.x, col3.x, col4.x,
			col1.y, col2.y, col3.y, col4.y,
			col1.z, col2.z, col3.z, col4.z,
			col1.w, col2.w, col3.w, col4.w
		}
	{
	}

	/**
	 * @brief Constructor from array (column-major layout)
	 * @param values Array of 16 floats in column-major order
	 */
	Matrix4::Matrix4(float values[16])
		: Matrix4{
			values[0], values[4], values[8], values[12],   // First row from columns
			values[1], values[5], values[9], values[13],   // Second row from columns
			values[2], values[6], values[10], values[14],  // Third row from columns
			values[3], values[7], values[11], values[15]   // Fourth row from columns
		}
	{
	}

	/**
	 * @brief Constructor from Matrix3 (extends to 4x4 with identity row/column)
	 * @param matrix 3x3 matrix to extend to 4x4
	 */
	Matrix4::Matrix4(const Matrix3& matrix)
		: Matrix4{
			matrix.m11, matrix.m12, matrix.m13, 0.f,
			matrix.m21, matrix.m22, matrix.m23, 0.f,
			matrix.m31, matrix.m32, matrix.m33, 0.f,
			0.f, 0.f, 0.f, 1.f
		}
	{
	}

	/**
	 * @brief Copy constructor
	 * @param rhs Matrix to copy from
	 */
	Matrix4::Matrix4(const Matrix4& rhs)
		: Matrix4{
			rhs.m11, rhs.m12, rhs.m13, rhs.m14,
			rhs.m21, rhs.m22, rhs.m23, rhs.m24,
			rhs.m31, rhs.m32, rhs.m33, rhs.m34,
			rhs.m41, rhs.m42, rhs.m43, rhs.m44
		}
	{
	}

	// ===== Matrix Operations =====

	/**
	 * @brief Calculates the determinant of the matrix using cofactor expansion
	 * @return float The determinant value
	 */
	float Matrix4::Determinant() const
	{
		// Cofactor expansion along the first row
		return m11 * (
			m22 * (m33 * m44 - m34 * m43) -
			m23 * (m32 * m44 - m34 * m42) +
			m24 * (m32 * m43 - m33 * m42)
			) -
			m12 * (
				m21 * (m33 * m44 - m34 * m43) -
				m23 * (m31 * m44 - m34 * m41) +
				m24 * (m31 * m43 - m33 * m41)
				) +
			m13 * (
				m21 * (m32 * m44 - m34 * m42) -
				m22 * (m31 * m44 - m34 * m41) +
				m24 * (m31 * m42 - m32 * m41)
				) -
			m14 * (
				m21 * (m32 * m43 - m33 * m42) -
				m22 * (m31 * m43 - m33 * m41) +
				m23 * (m31 * m42 - m32 * m41)
				);
	}

	/**
	 * @brief Returns the transpose of this matrix
	 * @return Matrix4 Transposed matrix (rows become columns)
	 */
	Matrix4 Matrix4::Transposed() const
	{
		return Matrix4
		{
			m11, m21, m31, m41,
			m12, m22, m32, m42,
			m13, m23, m33, m43,
			m14, m24, m34, m44
		};
	}

	/**
	 * @brief Transposes this matrix in-place
	 */
	void Matrix4::Transpose()
	{
		// Swap symmetric elements across the main diagonal
		std::swap(m12, m21); // Swap (0,1) with (1,0)
		std::swap(m13, m31); // Swap (0,2) with (2,0)
		std::swap(m14, m41); // Swap (0,3) with (3,0)
		std::swap(m23, m32); // Swap (1,2) with (2,1)
		std::swap(m24, m42); // Swap (1,3) with (3,1)
		std::swap(m34, m43); // Swap (2,3) with (3,2)
	}

	/**
	 * @brief Calculates the cofactor matrix
	 * @return Matrix4 The cofactor matrix where each element is (-1)^(i+j) * minor
	 */
	Matrix4 Matrix4::Cofactor() const
	{
		// Lambda function to calculate 3x3 determinant for cofactor calculation
		auto calculate3X3Det = [](float a11, float a12, float a13,
			float a21, float a22, float a23,
			float a31, float a32, float a33)
			{
				return a11 * (a22 * a33 - a23 * a32)
					- a12 * (a21 * a33 - a23 * a31)
					+ a13 * (a21 * a32 - a22 * a31);
			};

		// Calculate cofactors with alternating signs (+/-)
		return Matrix4
		{
			// Row 1 - signs: + - + -
			+calculate3X3Det(m22, m23, m24, m32, m33, m34, m42, m43, m44), // C11
			-calculate3X3Det(m21, m23, m24, m31, m33, m34, m41, m43, m44), // C12
			+calculate3X3Det(m21, m22, m24, m31, m32, m34, m41, m42, m44), // C13
			-calculate3X3Det(m21, m22, m23, m31, m32, m33, m41, m42, m43), // C14

			// Row 2 - signs: - + - +
			-calculate3X3Det(m12, m13, m14, m32, m33, m34, m42, m43, m44), // C21
			+calculate3X3Det(m11, m13, m14, m31, m33, m34, m41, m43, m44), // C22
			-calculate3X3Det(m11, m12, m14, m31, m32, m34, m41, m42, m44), // C23
			+calculate3X3Det(m11, m12, m13, m31, m32, m33, m41, m42, m43), // C24

			// Row 3 - signs: + - + -
			+calculate3X3Det(m12, m13, m14, m22, m23, m24, m42, m43, m44), // C31
			-calculate3X3Det(m11, m13, m14, m21, m23, m24, m41, m43, m44), // C32
			+calculate3X3Det(m11, m12, m14, m21, m22, m24, m41, m42, m44), // C33
			-calculate3X3Det(m11, m12, m13, m21, m22, m23, m41, m42, m43), // C34

			// Row 4 - signs: - + - +
			-calculate3X3Det(m12, m13, m14, m22, m23, m24, m32, m33, m34), // C41
			+calculate3X3Det(m11, m13, m14, m21, m23, m24, m31, m33, m34), // C42
			-calculate3X3Det(m11, m12, m14, m21, m22, m24, m31, m32, m34), // C43
			+calculate3X3Det(m11, m12, m13, m21, m22, m23, m31, m32, m33)  // C44
		};
	}

	/**
	 * @brief Calculates the adjugate (adjoint) matrix
	 * @return Matrix4 The adjugate matrix (transpose of cofactor matrix)
	 */
	Matrix4 Matrix4::Adjugate() const
	{
		return Cofactor().Transposed();
	}

	/**
	 * @brief Calculates the inverse matrix with optimizations for special cases
	 * @return Matrix4 The inverse matrix
	 * @throws runtime_error If the matrix is not invertible (determinant is zero)
	 */
	Matrix4 Matrix4::Inverse() const
	{
		const float det = Determinant();
		if (MathF::IsNearZero(det))
		{
			throw runtime_error("Matrix is not invertible!");
		}

		// Optimization: Check for pure rotation (orthogonal matrix)
		if (IsOrthogonal())
		{
			// For orthogonal matrices, inverse = transpose
			return Transposed();
		}

		// Optimization: Check for pure translation matrix
		if (MathF::IsNearZero(m11 - 1.0f) && MathF::IsNearZero(m22 - 1.0f) &&
			MathF::IsNearZero(m33 - 1.0f) && MathF::IsNearZero(m44 - 1.0f) &&
			MathF::IsNearZero(m12) && MathF::IsNearZero(m13) &&
			MathF::IsNearZero(m21) && MathF::IsNearZero(m23) &&
			MathF::IsNearZero(m31) && MathF::IsNearZero(m32) &&
			MathF::IsNearZero(m41) && MathF::IsNearZero(m42) && MathF::IsNearZero(m43))
		{
			// For translation matrices, just negate the translation components
			return Translation(-m14, -m24, -m34);
		}

		Matrix4 adj = Adjugate();

		// General case: inverse = (1/determinant) * adjugate
		return (1.0f / det) * adj;
	}

	// ===== Matrix Properties =====

	/**
	 * @brief Checks if this matrix is an identity matrix
	 * @param tolerance Floating-point comparison tolerance
	 * @return bool True if matrix is identity within tolerance
	 */
	bool Matrix4::IsIdentity(float tolerance) const
	{
		return MathF::Compare(m11, 1.f, tolerance) && MathF::IsNearZero(m12, tolerance) &&
			MathF::IsNearZero(m13, tolerance) && MathF::IsNearZero(m14, tolerance) &&
			MathF::IsNearZero(m21, tolerance) && MathF::Compare(m22, 1.f, tolerance) &&
			MathF::IsNearZero(m23, tolerance) && MathF::IsNearZero(m24, tolerance) &&
			MathF::IsNearZero(m31, tolerance) && MathF::IsNearZero(m32, tolerance) && // Fixed: was m23
			MathF::Compare(m33, 1.f, tolerance) && MathF::IsNearZero(m34, tolerance) &&
			MathF::IsNearZero(m41, tolerance) && MathF::IsNearZero(m42, tolerance) &&
			MathF::IsNearZero(m43, tolerance) && MathF::Compare(m44, 1.f, tolerance);
	}

	/**
	 * @brief Checks if this matrix is a zero matrix
	 * @param tolerance Floating-point comparison tolerance
	 * @return bool True if all elements are zero within tolerance
	 */
	bool Matrix4::IsZero(float tolerance) const
	{
		return MathF::IsNearZero(m11, tolerance) && MathF::IsNearZero(m12, tolerance) &&
			MathF::IsNearZero(m13, tolerance) && MathF::IsNearZero(m14, tolerance) &&
			MathF::IsNearZero(m21, tolerance) && MathF::IsNearZero(m22, tolerance) &&
			MathF::IsNearZero(m23, tolerance) && MathF::IsNearZero(m24, tolerance) &&
			MathF::IsNearZero(m31, tolerance) && MathF::IsNearZero(m32, tolerance) && // Fixed: was m23
			MathF::IsNearZero(m33, tolerance) && MathF::IsNearZero(m34, tolerance) &&
			MathF::IsNearZero(m41, tolerance) && MathF::IsNearZero(m42, tolerance) &&
			MathF::IsNearZero(m43, tolerance) && MathF::IsNearZero(m44, tolerance);
	}

	/**
	 * @brief Checks if this matrix is orthogonal (M^T * M = I)
	 * @return bool True if matrix is orthogonal
	 */
	bool Matrix4::IsOrthogonal() const
	{
		// A matrix is orthogonal if its transpose equals its inverse
		return (Transposed() * (*this)).IsIdentity();
	}

	// ===== Transform Decomposition =====

	/**
	 * @brief Extracts translation component from transformation matrix
	 * @return Vector3 Translation vector from the 4th column
	 */
	Vector3 Matrix4::GetTranslation() const
	{
		// Translation is stored in the 4th column (m14, m24, m34)
		return Vector3{ m14, m24, m34 };
	}

	/**
	 * @brief Extracts scale component from transformation matrix
	 * @return Vector3 Scale vector calculated from column magnitudes
	 */
	Vector3 Matrix4::GetScale() const
	{
		// Scale factors are the magnitudes of the first three column vectors
		return Vector3
		{
			GetColumn(0).Magnitude(),
			GetColumn(1).Magnitude(),
			GetColumn(2).Magnitude()
		};
	}

	/**
	 * @brief Extracts rotation component as 3x3 matrix
	 * @return Matrix3 3x3 rotation matrix (upper-left 3x3 portion)
	 * @note This assumes uniform scale or pre-normalized rotation matrix
	 */
	Matrix3 Matrix4::GetRotation() const
	{
		// Rotation is the upper-left 3x3 submatrix
		return Matrix3
		{
			m11, m12, m13,
			m21, m22, m23,
			m31, m32, m33
		};
	}

	// ===== Column/Row Access =====

	/**
	 * @brief Gets a column vector from the matrix
	 * @param index Column index (0, 1, 2, or 3)
	 * @return Vector4 The specified column as a vector
	 * @throws runtime_error If index is out of bounds
	 */
	Vector4 Matrix4::GetColumn(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector4{ m11, m21, m31, m41 };
		}
		case 1:
		{
			return Vector4{ m12, m22, m32, m42 };
		}
		case 2:
		{
			return Vector4{ m13, m23, m33, m43 };
		}
		case 3:
		{
			return Vector4{ m14, m24, m34, m44 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * @brief Sets a column in the matrix
	 * @param index Column index (0, 1, 2, or 3)
	 * @param column Vector to set as the column
	 * @throws runtime_error If index is out of bounds
	 */
	void Matrix4::SetColumn(int index, const Vector4& column)
	{
		switch (index)
		{
		case 0:
		{
			m11 = column.x;
			m21 = column.y;
			m31 = column.z;
			m41 = column.w;
			break;
		}
		case 1:
		{
			m12 = column.x;
			m22 = column.y;
			m32 = column.z;
			m42 = column.w;
			break;
		}
		case 2:
		{
			m13 = column.x;
			m23 = column.y;
			m33 = column.z;
			m43 = column.w;
			break;
		}
		case 3:
		{
			m14 = column.x;
			m24 = column.y;
			m34 = column.z;
			m44 = column.w;
			break;
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * @brief Gets a row vector from the matrix
	 * @param index Row index (0, 1, 2, or 3)
	 * @return Vector4 The specified row as a vector
	 * @throws runtime_error If index is out of bounds
	 */
	Vector4 Matrix4::GetRow(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector4{ m11, m12, m13, m14 };
		}
		case 1:
		{
			return Vector4{ m21, m22, m23, m24 };
		}
		case 2:
		{
			return Vector4{ m31, m32, m33, m34 };
		}
		case 3:
		{
			return Vector4{ m41, m42, m43, m44 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * @brief Sets a row in the matrix
	 * @param index Row index (0, 1, 2, or 3)
	 * @param row Vector to set as the row
	 * @throws runtime_error If index is out of bounds
	 */
	void Matrix4::SetRow(int index, const Vector4& row)
	{
		switch (index)
		{
		case 0:
		{
			m11 = row.x;
			m12 = row.y;
			m13 = row.z;
			m14 = row.w;
			break;
		}
		case 1:
		{
			m21 = row.x;
			m22 = row.y;
			m23 = row.z;
			m24 = row.w;
			break;
		}
		case 2:
		{
			m31 = row.x;
			m32 = row.y;
			m33 = row.z;
			m34 = row.w;
			break;
		}
		case 3:
		{
			m41 = row.x;
			m42 = row.y;
			m43 = row.z;
			m44 = row.w;
			break;
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	// ===== String Representation =====

	/**
	 * @brief Converts the matrix to a formatted string representation
	 * @return string Formatted string showing the matrix layout
	 */
	string Matrix4::ToString() const
	{
		return std::format(
			"[\n\t{}, {}, {}, {},\n\t{}, {}, {}, {},\n\t {}, {}, {}, {},\n\t{}, {}, {}, {}\n]",
			m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, m41, m42, m43, m44
		);
	}

	/**
	 * @brief Stream output operator for Matrix4
	 * @param stream Output stream
	 * @param matrix Matrix to output
	 * @return ostream& Reference to the stream
	 */
	ostream& operator<<(ostream& stream, const Matrix4& matrix)
	{
		stream << matrix.ToString();

		return stream;
	}

	// ===== Comparison Operators =====

	/**
	 * @brief Equality comparison operator
	 * @param rhs Matrix to compare with
	 * @return bool True if matrices are equal within floating-point tolerance
	 */
	bool Matrix4::operator==(const Matrix4& rhs) const
	{
		return MathF::Compare(m11, rhs.m11) && MathF::Compare(m21, rhs.m21) && MathF::Compare(m31, rhs.m31) &&
			MathF::Compare(m41, rhs.m41) &&
			MathF::Compare(m12, rhs.m12) && MathF::Compare(m22, rhs.m22) && MathF::Compare(m32, rhs.m32) &&
			MathF::Compare(m42, rhs.m42) &&
			MathF::Compare(m13, rhs.m13) && MathF::Compare(m23, rhs.m23) && MathF::Compare(m33, rhs.m33) &&
			MathF::Compare(m43, rhs.m43) &&
			MathF::Compare(m14, rhs.m14) && MathF::Compare(m24, rhs.m24) && MathF::Compare(m34, rhs.m34) &&
			MathF::Compare(m44, rhs.m44);
	}

	/**
	 * @brief Inequality comparison operator
	 * @param rhs Matrix to compare with
	 * @return bool True if matrices are not equal
	 */
	bool Matrix4::operator!=(const Matrix4& rhs) const
	{
		return !MathF::Compare(m11, rhs.m11) || !MathF::Compare(m21, rhs.m21) || !MathF::Compare(m31, rhs.m31) || !
			MathF::Compare(m41, rhs.m41) ||
			!MathF::Compare(m12, rhs.m12) || !MathF::Compare(m22, rhs.m22) || !MathF::Compare(m32, rhs.m32) || !
			MathF::Compare(m42, rhs.m42) ||
			!MathF::Compare(m13, rhs.m13) || !MathF::Compare(m23, rhs.m23) || !MathF::Compare(m33, rhs.m33) || !
			MathF::Compare(m43, rhs.m43) ||
			!MathF::Compare(m14, rhs.m14) || !MathF::Compare(m24, rhs.m24) || !MathF::Compare(m34, rhs.m34) || !
			MathF::Compare(m44, rhs.m44);
	}

	// ===== Arithmetic Operators =====

	/**
	 * @brief Matrix multiplication operator
	 * @param rhs Matrix to multiply with
	 * @return Matrix4 Result of matrix multiplication (this * rhs)
	 */
	Matrix4 Matrix4::operator*(const Matrix4& rhs) const
	{
		// Standard matrix multiplication: C[i][j] = sum(A[i][k] * B[k][j])
		return
		{
			// First row of result
			m11 * rhs.m11 + m12 * rhs.m21 + m13 * rhs.m31 + m14 * rhs.m41,
			m11 * rhs.m12 + m12 * rhs.m22 + m13 * rhs.m32 + m14 * rhs.m42,
			m11 * rhs.m13 + m12 * rhs.m23 + m13 * rhs.m33 + m14 * rhs.m43,
			m11 * rhs.m14 + m12 * rhs.m24 + m13 * rhs.m34 + m14 * rhs.m44,

			// Second row of result
			m21 * rhs.m11 + m22 * rhs.m21 + m23 * rhs.m31 + m24 * rhs.m41,
			m21 * rhs.m12 + m22 * rhs.m22 + m23 * rhs.m32 + m24 * rhs.m42,
			m21 * rhs.m13 + m22 * rhs.m23 + m23 * rhs.m33 + m24 * rhs.m43,
			m21 * rhs.m14 + m22 * rhs.m24 + m23 * rhs.m34 + m24 * rhs.m44,

			// Third row of result
			m31 * rhs.m11 + m32 * rhs.m21 + m33 * rhs.m31 + m34 * rhs.m41,
			m31 * rhs.m12 + m32 * rhs.m22 + m33 * rhs.m32 + m34 * rhs.m42,
			m31 * rhs.m13 + m32 * rhs.m23 + m33 * rhs.m33 + m34 * rhs.m43,
			m31 * rhs.m14 + m32 * rhs.m24 + m33 * rhs.m34 + m34 * rhs.m44,

			// Fourth row of result
			m41 * rhs.m11 + m42 * rhs.m21 + m43 * rhs.m31 + m44 * rhs.m41,
			m41 * rhs.m12 + m42 * rhs.m22 + m43 * rhs.m32 + m44 * rhs.m42,
			m41 * rhs.m13 + m42 * rhs.m23 + m43 * rhs.m33 + m44 * rhs.m43,
			m41 * rhs.m14 + m42 * rhs.m24 + m43 * rhs.m34 + m44 * rhs.m44
		};
	}

	/**
	 * @brief Scalar multiplication operator
	 * @param scalar Value to multiply all elements by
	 * @return Matrix4 Result of scalar multiplication
	 */
	Matrix4 Matrix4::operator*(float scalar) const
	{
		return Matrix4
		{
			m11 * scalar, m12 * scalar, m13 * scalar, m14 * scalar,
			m21 * scalar, m22 * scalar, m23 * scalar, m24 * scalar,
			m31 * scalar, m32 * scalar, m33 * scalar, m34 * scalar,
			m41 * scalar, m42 * scalar, m43 * scalar, m44 * scalar
		};
	}

	/**
	 * @brief Scalar division operator
	 * @param scalar Value to divide all elements by
	 * @return Matrix4 Result of scalar division
	 * @throws runtime_error If scalar is zero or near-zero
	 */
	Matrix4 Matrix4::operator/(float scalar) const
	{
		if (MathF::IsNearZero(scalar))
		{
			throw runtime_error("Division by zero!");
		}

		return Matrix4
		{
			m11 / scalar, m12 / scalar, m13 / scalar, m14 / scalar,
			m21 / scalar, m22 / scalar, m23 / scalar, m24 / scalar,
			m31 / scalar, m32 / scalar, m33 / scalar, m34 / scalar,
			m41 / scalar, m42 / scalar, m43 / scalar, m44 / scalar
		};
	}

	/**
	 * @brief Matrix-Vector4 multiplication (transforms 4D vector)
	 * @param rhs Vector4 to transform
	 * @return Vector4 Transformed vector
	 */
	Vector4 Matrix4::operator*(const Vector4& rhs) const
	{
		return Vector4
		{
			m11 * rhs.x + m12 * rhs.y + m13 * rhs.z + m14 * rhs.w,
			m21 * rhs.x + m22 * rhs.y + m23 * rhs.z + m24 * rhs.w,
			m31 * rhs.x + m32 * rhs.y + m33 * rhs.z + m34 * rhs.w,
			m41 * rhs.x + m42 * rhs.y + m43 * rhs.z + m44 * rhs.w
		};
	}

	/**
	 * @brief Matrix-Vector3 multiplication (transforms 3D vector with implicit w=1)
	 * @param rhs Vector3 to transform
	 * @return Vector3 Transformed vector (translation is applied)
	 */
	Vector3 Matrix4::operator*(const Vector3& rhs) const
	{
		// Treats Vector3 as Vector4 with w=1 (applies translation)
		return Vector3
		{
			m11 * rhs.x + m12 * rhs.y + m13 * rhs.z + m14,
			m21 * rhs.x + m22 * rhs.y + m23 * rhs.z + m24,
			m31 * rhs.x + m32 * rhs.y + m33 * rhs.z + m34,
		};
	}

	/**
	 * @brief Subscript operator for column access
	 * @param index Column index (0, 1, 2, or 3)
	 * @return Vector4 The specified column as a vector
	 */
	Vector4 Matrix4::operator[](int index) const
	{
		return GetColumn(index);
	}

	/**
	 * @brief Assignment operator
	 * @param rhs Matrix to assign from
	 * @return Matrix4& Reference to this matrix
	 */
	Matrix4& Matrix4::operator=(const Matrix4& rhs)
	{
		// Self-assignment check
		if (*this == rhs)
		{
			return *this;
		}

		// Copy all elements
		m11 = rhs.m11;
		m12 = rhs.m12;
		m13 = rhs.m13;
		m14 = rhs.m14;
		m21 = rhs.m21;
		m22 = rhs.m22;
		m23 = rhs.m23;
		m24 = rhs.m24;
		m31 = rhs.m31;
		m32 = rhs.m32;
		m33 = rhs.m33;
		m34 = rhs.m34;
		m41 = rhs.m41;
		m42 = rhs.m42;
		m43 = rhs.m43;
		m44 = rhs.m44;

		return *this;
	}

	/**
	 * @brief Free function for scalar-matrix multiplication (scalar * matrix)
	 * @param scalar Value to multiply by
	 * @param rhs Matrix to multiply
	 * @return Matrix4 Result of scalar multiplication
	 */
	Matrix4 operator*(float scalar, const Matrix4& rhs)
	{
		return rhs * scalar;
	}
}