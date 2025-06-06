/**
 * @file Matrix3.cpp
 * @brief Implementation of the Matrix3 class for 3x3 matrix operations
 *
 * This file contains the implementation of a 3x3 matrix class with support for
 * common mathematical operations including transformations, matrix arithmetic,
 * and linear algebra operations. The matrix uses column-major storage order.
 */

#include "nudge/Matrix3.hpp"

#include <format>

#include "nudge/MathF.hpp"
#include "Nudge/Matrix2.hpp"
#include "Nudge/Vector2.hpp"
#include "nudge/Vector3.hpp"

using std::runtime_error;

namespace Nudge
{
	// ===== Static Factory Methods =====

	/**
	 * @brief Creates a 3x3 identity matrix
	 * @return Matrix3 Identity matrix with 1s on the diagonal and 0s elsewhere
	 */
	Matrix3 Matrix3::Identity()
	{
		return Matrix3{ 1.f };
	}

	/**
	 * @brief Creates a 3x3 zero matrix
	 * @return Matrix3 Matrix with all elements set to 0
	 */
	Matrix3 Matrix3::Zero()
	{
		return Matrix3{ 0.f };
	}

	/**
	 * @brief Creates a 3x3 scale transformation matrix
	 * @param sx Scale factor along X-axis
	 * @param sy Scale factor along Y-axis
	 * @param sz Scale factor along Z-axis
	 * @return Matrix3 Scale transformation matrix
	 */
	Matrix3 Matrix3::Scale(float sx, float sy, float sz)
	{
		return Matrix3
		{
			sx, 0.f, 0.f,
			0.f, sy, 0.f,
			0.f, 0.f, sz
		};
	}

	/**
	 * @brief Creates a 3x3 scale transformation matrix from a Vector3
	 * @param scale Vector containing scale factors for each axis
	 * @return Matrix3 Scale transformation matrix
	 */
	Matrix3 Matrix3::Scale(const Vector3& scale)
	{
		return Scale(scale.x, scale.y, scale.z);
	}

	/**
	 * @brief Creates a rotation matrix around the X-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix3 Rotation matrix for X-axis rotation
	 */
	Matrix3 Matrix3::RotationX(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		return Matrix3
		{
			1.f, 0.f, 0.f,
			0.f, cosTheta, -sinTheta,
			0.f, sinTheta, cosTheta
		};
	}

	/**
	 * @brief Creates a rotation matrix around the Y-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix3 Rotation matrix for Y-axis rotation
	 */
	Matrix3 Matrix3::RotationY(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		return Matrix3
		{
			cosTheta, 0.f, sinTheta,
			0.f, 1.f, 0.f,
			-sinTheta, 0.f, cosTheta
		};
	}

	/**
	 * @brief Creates a rotation matrix around the Z-axis
	 * @param degrees Rotation angle in degrees
	 * @return Matrix3 Rotation matrix for Z-axis rotation
	 */
	Matrix3 Matrix3::RotationZ(float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		return Matrix3
		{
			cosTheta, -sinTheta, 0.f,
			sinTheta, cosTheta, 0.f,
			0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a composite rotation matrix from Euler angles
	 * @param euler Vector containing rotation angles (x, y, z) in degrees
	 * @return Matrix3 Composite rotation matrix (Z * Y * X order)
	 */
	Matrix3 Matrix3::Rotation(const Vector3& euler)
	{
		return RotationZ(euler.z) * RotationY(euler.y) * RotationX(euler.x);
	}

	/**
	 * @brief Creates a rotation matrix around an arbitrary axis using Rodrigues' formula
	 * @param axis The axis of rotation (will be normalized)
	 * @param degrees Rotation angle in degrees
	 * @return Matrix3 Rotation matrix for arbitrary axis rotation
	 */
	Matrix3 Matrix3::Rotation(const Vector3& axis, float degrees)
	{
		const float theta = MathF::Radians(degrees);
		const float cosTheta = MathF::Cos(theta);
		const float sinTheta = MathF::Sin(theta);

		const float oneMinusCos = 1 - cosTheta;

		const Vector3 n = axis.Normalized();

		return Matrix3
		{
			MathF::Squared(n.x) * oneMinusCos + cosTheta,
			n.x * n.y * oneMinusCos - n.z * sinTheta,
			n.x * n.z * oneMinusCos + n.y * sinTheta,
			n.y * n.x * oneMinusCos + n.z * sinTheta,
			MathF::Squared(n.y) * oneMinusCos + cosTheta,
			n.y * n.z * oneMinusCos - n.x * sinTheta,
			n.z * n.x * oneMinusCos - n.y * sinTheta,
			n.z * n.y * oneMinusCos + n.x * sinTheta,
			MathF::Squared(n.z) * oneMinusCos + cosTheta
		};
	}

	/**
	 * @brief Creates a 2D translation matrix (homogeneous coordinates)
	 * @param tx Translation along X-axis
	 * @param ty Translation along Y-axis
	 * @return Matrix3 Translation matrix for 2D transformations
	 */
	Matrix3 Matrix3::Translation(float tx, float ty)
	{
		return Matrix3
		{
			1.f, 0.f, tx,
			0.f, 1.f, ty,
			0.f, 0.f, 1.f
		};
	}

	/**
	 * @brief Creates a 2D translation matrix from a Vector2
	 * @param translation Vector containing translation values
	 * @return Matrix3 Translation matrix for 2D transformations
	 */
	Matrix3 Matrix3::Translation(const Vector2& translation)
	{
		return Translation(translation.x, translation.y);
	}

	// ===== Constructors =====

	/**
	 * @brief Default constructor - creates an identity matrix
	 */
	Matrix3::Matrix3()
		: Matrix3{ 1.f }
	{
	}

	/**
	 * @brief Scalar constructor - creates a matrix with the scalar on the diagonal
	 * @param scalar Value to place on the diagonal (identity matrix if scalar = 1)
	 */
	Matrix3::Matrix3(float scalar)
		: Matrix3{ scalar, 0.f, 0.f, 0.f, scalar, 0.f, 0.f, 0.f, scalar }
	{
	}

	/**
	 * @brief Element-wise constructor (column-major order)
	 * @param m11,m12,m13,m21,m22,m23,m31,m32,m33 Matrix elements in column-major order
	 * @note Parameters are in column-major order: col1(m11,m21,m31), col2(m12,m22,m32), col3(m13,m23,m33)
	 */
	Matrix3::Matrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33)
		: m11{ m11 }, m21{ m21 }, m31{ m31 }, m12{ m12 }, m22{ m22 }, m32{ m32 }, m13{ m13 }, m23{ m23 }, m33{ m33 }
	{
	}

	/**
	 * @brief Constructor from three column vectors
	 * @param col1 First column vector
	 * @param col2 Second column vector
	 * @param col3 Third column vector
	 */
	Matrix3::Matrix3(const Vector3& col1, const Vector3& col2, const Vector3& col3)
		: Matrix3{ col1.x, col2.x, col3.x, col1.y, col2.y, col3.y, col1.z, col2.z, col3.z }
	{
	}

	/**
	 * @brief Constructor from array (column-major layout)
	 * @param values Array of 9 floats in column-major order
	 */
	Matrix3::Matrix3(float values[9])
		: Matrix3{
			values[0], values[3], values[6], // m11, m12, m13
			values[1], values[4], values[7], // m21, m22, m23
			values[2], values[5], values[8]  // m31, m32, m33
		}
	{
	}

	/**
	 * @brief Constructor from Matrix2 (extends to 3x3 with identity in bottom-right)
	 * @param matrix 2x2 matrix to extend
	 */
	Matrix3::Matrix3(const Matrix2& matrix)
		: Matrix3{ matrix.m11, matrix.m12, 0.f, matrix.m21, matrix.m22, 0.f, 0.f, 0.f, 1.f }
	{
	}

	/**
	 * @brief Copy constructor
	 * @param rhs Matrix to copy from
	 */
	Matrix3::Matrix3(const Matrix3& rhs)
		: Matrix3{ rhs.m11, rhs.m12, rhs.m13, rhs.m21, rhs.m22, rhs.m23, rhs.m31, rhs.m32, rhs.m33 }
	{
	}

	// ===== Matrix Operations =====

	/**
	 * @brief Calculates the determinant of the matrix
	 * @return float The determinant value
	 */
	float Matrix3::Determinant() const
	{
		return m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31);
	}

	/**
	 * @brief Returns the transpose of this matrix
	 * @return Matrix3 Transposed matrix (rows become columns)
	 */
	Matrix3 Matrix3::Transposed() const
	{
		return Matrix3
		{
			m11, m21, m31, // Row 1 becomes Column 1
			m12, m22, m32, // Row 2 becomes Column 2  
			m13, m23, m33  // Row 3 becomes Column 3
		};
	}

	/**
	 * @brief Transposes this matrix in-place
	 */
	void Matrix3::Transpose()
	{
		std::swap(m12, m21);
		std::swap(m13, m31);
		std::swap(m23, m32);
	}

	/**
	 * @brief Calculates the cofactor matrix
	 * @return Matrix3 The cofactor matrix where each element is (-1)^(i+j) * minor
	 */
	Matrix3 Matrix3::Cofactor() const
	{
		return Matrix3
		{
			+(m22 * m33 - m23 * m32), // C11
			-(m21 * m33 - m23 * m31), // C12
			+(m21 * m32 - m22 * m31), // C13
			-(m12 * m33 - m13 * m32), // C21
			+(m11 * m33 - m13 * m31), // C22
			-(m11 * m32 - m12 * m31), // C23
			+(m12 * m23 - m13 * m22), // C31
			-(m11 * m23 - m13 * m21), // C32
			+(m11 * m22 - m12 * m21)  // C33
		};
	}

	/**
	 * @brief Calculates the adjugate (adjoint) matrix
	 * @return Matrix3 The adjugate matrix (transpose of cofactor matrix)
	 */
	Matrix3 Matrix3::Adjugate() const
	{
		return Cofactor().Transposed();
	}

	/**
	 * @brief Calculates the inverse of this matrix
	 * @return Matrix3 The inverse matrix
	 * @throws runtime_error If the matrix is not invertible (determinant is zero)
	 */
	Matrix3 Matrix3::Inverse() const
	{
		const float det = Determinant();

		if (MathF::IsNearZero(det))
		{
			throw runtime_error("Matrix is not invertible!");
		}

		return 1 / det * Adjugate();
	}

	// ===== Matrix Properties =====

	/**
	 * @brief Checks if this matrix is an identity matrix
	 * @param tolerance Floating-point comparison tolerance
	 * @return bool True if matrix is identity within tolerance
	 */
	bool Matrix3::IsIdentity(float tolerance) const
	{
		return MathF::Compare(m11, 1.f, tolerance) && MathF::IsNearZero(m21, tolerance) &&
			MathF::IsNearZero(m31, tolerance) &&
			MathF::IsNearZero(m12, tolerance) && MathF::Compare(m22, 1.f, tolerance) &&
			MathF::IsNearZero(m32, tolerance) &&
			MathF::IsNearZero(m13, tolerance) && MathF::IsNearZero(m23, tolerance) &&
			MathF::Compare(m33, 1.f, tolerance);
	}

	/**
	 * @brief Checks if this matrix is a zero matrix
	 * @param tolerance Floating-point comparison tolerance
	 * @return bool True if all elements are zero within tolerance
	 */
	bool Matrix3::IsZero(float tolerance) const
	{
		return MathF::IsNearZero(m11, tolerance) && MathF::IsNearZero(m21, tolerance) &&
			MathF::IsNearZero(m31, tolerance) &&
			MathF::IsNearZero(m12, tolerance) && MathF::IsNearZero(m22, tolerance) &&
			MathF::IsNearZero(m32, tolerance) &&
			MathF::IsNearZero(m13, tolerance) && MathF::IsNearZero(m23, tolerance) &&
			MathF::IsNearZero(m33, tolerance);
	}

	/**
	 * @brief Checks if this matrix is orthogonal (M^T * M = I)
	 * @return bool True if matrix is orthogonal
	 */
	bool Matrix3::IsOrthogonal() const
	{
		return (Transposed() * (*this)).IsIdentity();
	}

	// ===== Column/Row Access =====

	/**
	 * @brief Gets a column vector from the matrix
	 * @param index Column index (0, 1, or 2)
	 * @return Vector3 The specified column as a vector
	 * @throws runtime_error If index is out of bounds
	 */
	Vector3 Matrix3::GetColumn(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector3{ m11, m21, m31 };
		}
		case 1:
		{
			return Vector3{ m12, m22, m32 };
		}
		case 2:
		{
			return Vector3{ m13, m23, m33 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * @brief Sets a column in the matrix
	 * @param index Column index (0, 1, or 2)
	 * @param column Vector to set as the column
	 * @throws runtime_error If index is out of bounds
	 */
	void Matrix3::SetColumn(int index, const Vector3& column)
	{
		switch (index)
		{
		case 0:
		{
			m11 = column.x;
			m21 = column.y;
			m31 = column.z;
			break;
		}
		case 1:
		{
			m12 = column.x;
			m22 = column.y;
			m32 = column.z;
			break;
		}
		case 2:
		{
			m13 = column.x;
			m23 = column.y;
			m33 = column.z;
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
	 * @param index Row index (0, 1, or 2)
	 * @return Vector3 The specified row as a vector
	 * @throws runtime_error If index is out of bounds
	 */
	Vector3 Matrix3::GetRow(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector3{ m11, m12, m13 };
		}
		case 1:
		{
			return Vector3{ m21, m22, m23 };
		}
		case 2:
		{
			return Vector3{ m31, m32, m33 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * @brief Sets a row in the matrix
	 * @param index Row index (0, 1, or 2)
	 * @param row Vector to set as the row
	 * @throws runtime_error If index is out of bounds
	 */
	void Matrix3::SetRow(int index, const Vector3& row)
	{
		switch (index)
		{
		case 0:
		{
			m11 = row.x;
			m12 = row.y;
			m13 = row.z;
			break;
		}
		case 1:
		{
			m21 = row.x;
			m22 = row.y;
			m23 = row.z;
			break;
		}
		case 2:
		{
			m31 = row.x;
			m32 = row.y;
			m33 = row.z;
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
	string Matrix3::ToString() const
	{
		return std::format(
			"[\n\t{}, {}, {},\n\t{}, {}, {},\n\t {}, {}, {}\n]",
			m11, m12, m13, m21, m22, m23, m31, m32, m33
		);
	}

	/**
	 * @brief Stream output operator for Matrix3
	 * @param stream Output stream
	 * @param matrix Matrix to output
	 * @return ostream& Reference to the stream
	 */
	ostream& operator<<(ostream& stream, const Matrix3& matrix)
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
	bool Matrix3::operator==(const Matrix3& rhs) const
	{
		return MathF::Compare(m11, rhs.m11) && MathF::Compare(m21, rhs.m21) && MathF::Compare(m31, rhs.m31) &&
			MathF::Compare(m12, rhs.m12) && MathF::Compare(m22, rhs.m22) && MathF::Compare(m32, rhs.m32) &&
			MathF::Compare(m13, rhs.m13) && MathF::Compare(m23, rhs.m23) && MathF::Compare(m33, rhs.m33);
	}

	/**
	 * @brief Inequality comparison operator
	 * @param rhs Matrix to compare with
	 * @return bool True if matrices are not equal
	 */
	bool Matrix3::operator!=(const Matrix3& rhs) const
	{
		return !MathF::Compare(m11, rhs.m11) || !MathF::Compare(m21, rhs.m21) || !MathF::Compare(m31, rhs.m31) ||
			!MathF::Compare(m12, rhs.m12) || !MathF::Compare(m22, rhs.m22) || !MathF::Compare(m32, rhs.m32) ||
			!MathF::Compare(m13, rhs.m13) || !MathF::Compare(m23, rhs.m23) || !MathF::Compare(m33, rhs.m33);
	}

	// ===== Arithmetic Operators =====

	/**
	 * @brief Matrix multiplication operator
	 * @param rhs Matrix to multiply with
	 * @return Matrix3 Result of matrix multiplication (this * rhs)
	 */
	Matrix3 Matrix3::operator*(const Matrix3& rhs) const
	{
		return
		{
			// First row of result
			m11 * rhs.m11 + m12 * rhs.m21 + m13 * rhs.m31,
			m11 * rhs.m12 + m12 * rhs.m22 + m13 * rhs.m32,
			m11 * rhs.m13 + m12 * rhs.m23 + m13 * rhs.m33,

			// Second row of result
			m21 * rhs.m11 + m22 * rhs.m21 + m23 * rhs.m31,
			m21 * rhs.m12 + m22 * rhs.m22 + m23 * rhs.m32,
			m21 * rhs.m13 + m22 * rhs.m23 + m23 * rhs.m33,

			// Third row of result
			m31 * rhs.m11 + m32 * rhs.m21 + m33 * rhs.m31,
			m31 * rhs.m12 + m32 * rhs.m22 + m33 * rhs.m32,
			m31 * rhs.m13 + m32 * rhs.m23 + m33 * rhs.m33
		};
	}

	/**
	 * @brief Scalar multiplication operator
	 * @param scalar Value to multiply all elements by
	 * @return Matrix3 Result of scalar multiplication
	 */
	Matrix3 Matrix3::operator*(float scalar) const
	{
		return Matrix3
		{
			m11 * scalar, m12 * scalar, m13 * scalar,
			m21 * scalar, m22 * scalar, m23 * scalar,
			m31 * scalar, m32 * scalar, m33 * scalar
		};
	}

	/**
	 * @brief Scalar division operator
	 * @param scalar Value to divide all elements by
	 * @return Matrix3 Result of scalar division
	 * @throws runtime_error If scalar is zero or near-zero
	 */
	Matrix3 Matrix3::operator/(float scalar) const
	{
		if (MathF::IsNearZero(scalar))
		{
			throw runtime_error("Division by zero!");
		}

		return Matrix3
		{
			m11 / scalar, m12 / scalar, m13 / scalar,
			m21 / scalar, m22 / scalar, m23 / scalar,
			m31 / scalar, m32 / scalar, m33 / scalar
		};
	}

	/**
	 * @brief Matrix-vector multiplication operator
	 * @param rhs Vector to multiply with
	 * @return Vector3 Result of matrix-vector multiplication
	 */
	Vector3 Matrix3::operator*(const Vector3& rhs) const
	{
		return
		{
			m11 * rhs.x + m12 * rhs.y + m13 * rhs.z,
			m21 * rhs.x + m22 * rhs.y + m23 * rhs.z,
			m31 * rhs.x + m32 * rhs.y + m33 * rhs.z
		};
	}

	/**
	 * @brief Subscript operator for column access
	 * @param index Column index (0, 1, or 2)
	 * @return Vector3 The specified column as a vector
	 */
	Vector3 Matrix3::operator[](int index) const
	{
		return GetColumn(index);
	}

	/**
	 * @brief Assignment operator
	 * @param rhs Matrix to assign from
	 * @return Matrix3& Reference to this matrix
	 */
	Matrix3& Matrix3::operator=(const Matrix3& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		m11 = rhs.m11;
		m12 = rhs.m12;
		m13 = rhs.m13;
		m21 = rhs.m21;
		m22 = rhs.m22;
		m23 = rhs.m23;
		m31 = rhs.m31;
		m32 = rhs.m32;
		m33 = rhs.m33;

		return *this;
	}

	/**
	 * @brief Free function for scalar-matrix multiplication (scalar * matrix)
	 * @param scalar Value to multiply by
	 * @param rhs Matrix to multiply
	 * @return Matrix3 Result of scalar multiplication
	 */
	Matrix3 operator*(float scalar, const Matrix3& rhs)
	{
		return rhs * scalar;
	}
}