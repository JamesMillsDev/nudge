#include "Nudge/Matrix2.hpp"

#include <format>
#include <stdexcept>

#include "Nudge/MathF.hpp"
#include "Nudge/Vector2.hpp"

using std::runtime_error;

namespace Nudge
{
	/**
	 * Matrix2 Implementation
	 * Column-major 2x2 matrix operations
	 *
	 * @author JamesMillsAIE
	 * @date 2025-06-05
	 */

	 /**
	  * Creates a 2x2 identity matrix
	  * | 1  0 |
	  * | 0  1 |
	  * @return Identity matrix
	  */
	Matrix2 Matrix2::Identity()
	{
		return Matrix2
		{
			1.f, 0.f,
			0.f, 1.f
		};
	}

	/**
	 * Creates a 2x2 zero matrix
	 * | 0  0 |
	 * | 0  0 |
	 * @return Zero matrix
	 */
	Matrix2 Matrix2::Zero()
	{
		return Matrix2{ 0.f };
	}

	/**
	 * Creates a 2D scale matrix
	 * | sx  0 |
	 * | 0  sy |
	 * @param sx X-axis scale factor
	 * @param sy Y-axis scale factor
	 * @return Scale matrix
	 */
	Matrix2 Matrix2::Scale(float sx, float sy)
	{
		return Matrix2
		{
			sx, 0.f,
			0.f, sy
		};
	}

	/**
	 * Creates a 2D scale matrix from Vector2
	 * @param scale Vector containing scale factors (x, y)
	 * @return Scale matrix
	 */
	Matrix2 Matrix2::Scale(const Vector2& scale)
	{
		return Matrix2
		{
			scale.x, 0.f,
			0.f, scale.y
		};
	}

	/**
	 * Creates a 2D rotation matrix for counter-clockwise rotation
	 * | cos(theta) -sin(theta) |
	 * | sin(theta)  cos(theta) |
	 * @param degrees Rotation angle in degrees (positive = counter-clockwise)
	 * @return Rotation matrix
	 */
	Matrix2 Matrix2::Rotation(float degrees)
	{
		const float theta = MathF::Radians(degrees);

		return Matrix2
		{
			cos(theta), -sin(theta),
			sin(theta), cos(theta)
		};
	}

	/**
	 * Default constructor - creates identity matrix
	 * Delegates to scalar constructor with value 1.0f
	 */
	Matrix2::Matrix2()
		: Matrix2{ 1.f }
	{
	}

	/**
	 * Scalar constructor - creates scalar matrix (scalar on diagonal, 0 elsewhere)
	 * | scalar    0   |
	 * |   0    scalar |
	 * @param scalar Value for diagonal elements
	 */
	Matrix2::Matrix2(float scalar)
		: Matrix2{ scalar, 0.f, 0.f, scalar }
	{
	}

	/**
	 * Element-wise constructor with row-by-row input for intuitive use
	 * Despite column-major storage, parameters follow row-major order for ease
	 * Creates matrix:
	 * | m11  m12 |
	 * | m21  m22 |
	 * @param m11 Element at row 1, column 1
	 * @param m12 Element at row 1, column 2
	 * @param m21 Element at row 2, column 1
	 * @param m22 Element at row 2, column 2
	 */
	Matrix2::Matrix2(float m11, float m12, float m21, float m22)
		: m11{ m11 }, m21{ m21 }, m12{ m12 }, m22{ m22 }
	{
	}

	/**
	 * Column constructor from two Vector2's
	 * @param col1 First column vector [m11, m21]
	 * @param col2 Second column vector [m12, m22]
	 */
	Matrix2::Matrix2(const Vector2& col1, const Vector2& col2)
		: m11{ col1.x }, m21{ col1.y }, m12{ col2.x }, m22{ col2.y }
	{
	}

	/**
	 * Array constructor from column-major ordered array
	 * Expected array layout: [m11, m21, m12, m22]
	 * @param values Array of 4 floats in column-major order
	 */
	Matrix2::Matrix2(float values[4])
		: m11{ values[0] }, m21{ values[1] }, m12{ values[2] }, m22{ values[3] }
	{
	}

	/**
	 * Copy constructor (compiler-generated default is sufficient)
	 * @param rhs Matrix2 to copy from
	 */
	Matrix2::Matrix2(const Matrix2& rhs) = default;

	/**
	 * Calculates the determinant of this 2x2 matrix
	 * Formula: det = m11*m22 - m12*m21
	 * @return Determinant value
	 */
	float Matrix2::Determinant() const
	{
		return m11 * m22 - m12 * m21;
	}

	/**
	 * Returns a transposed copy of this matrix (rows become columns)
	 * Original matrix remains unchanged
	 * @return New transposed matrix
	 */
	Matrix2 Matrix2::Transposed() const
	{
		return Matrix2
		{
			m11, m21,
			m12, m22
		};
	}

	/**
	 * Transposes this matrix in-place (modifies original)
	 * For 2x2 matrix, only need to swap off-diagonal elements
	 */
	void Matrix2::Transpose()
	{
		std::swap(m12, m21);
	}

	/**
	 * Calculates and returns the inverse of this matrix
	 * For 2x2 matrix: (1/det) * | m22 -m12 |
	 *                            |-m21  m11 |
	 * @return Inverse matrix
	 * @throws runtime_error if matrix is not invertible (determinant near 0)
	 */
	Matrix2 Matrix2::Inverse() const
	{
		float det = Determinant();

		if (MathF::IsNearZero(det))
		{
			throw runtime_error("Matrix is not invertible");
		}

		return 1.f / det * Matrix2{ m22, -m12, -m21, m11 };
	}

	/**
	 * Checks if this matrix is approximately an identity matrix
	 * Uses floating-point tolerance for comparison
	 * @param tolerance The sensitivity of the comparisons
	 * @return True if matrix is approximately identity
	 */
	bool Matrix2::IsIdentity(float tolerance) const
	{
		return MathF::Compare(m11, 1.f, tolerance) && MathF::IsNearZero(m21, tolerance) &&
			MathF::IsNearZero(m12, tolerance) && MathF::Compare(m22, 1.f, tolerance);
	}

	/**
	 * Checks if this matrix is approximately a zero matrix
	 * Uses floating-point tolerance for comparison
	 * @param tolerance The sensitivity of the comparisons
	 * @return True if all elements are approximately zero
	 */
	bool Matrix2::IsZero(float tolerance) const
	{
		return MathF::IsNearZero(m11, tolerance) && MathF::IsNearZero(m21, tolerance) &&
			MathF::IsNearZero(m12, tolerance) && MathF::IsNearZero(m22, tolerance);
	}

	/**
	 * Gets a column vector from the matrix
	 * @param index Column index (0 = first column, 1 = second column)
	 * @return Column vector at specified index
	 * @throws runtime_error if index is out of bounds
	 */
	Vector2 Matrix2::GetColumn(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector2{ m11, m21 };
		}
		case 1:
		{
			return Vector2{ m12, m22 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * Sets a column in the matrix
	 * @param index Column index (0 = first column, 1 = second column)
	 * @param column New column vector
	 * @throws runtime_error if index is out of bounds
	 */
	void Matrix2::SetColumn(int index, const Vector2& column)
	{
		switch (index)
		{
		case 0:
		{
			m11 = column.x;
			m21 = column.y;
			break;
		}
		case 1:
		{
			m12 = column.x;
			m22 = column.y;
			break;
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * Gets a row vector from the matrix
	 * @param index Row index (0 = first row, 1 = second row)
	 * @return Row vector at specified index
	 * @throws runtime_error if index is out of bounds
	 */
	Vector2 Matrix2::GetRow(int index) const
	{
		switch (index)
		{
		case 0:
		{
			return Vector2{ m11, m12 };
		}
		case 1:
		{
			return Vector2{ m21, m22 };
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * Sets a row in the matrix
	 * @param index Row index (0 = first row, 1 = second row)
	 * @param row New row vector
	 * @throws runtime_error if index is out of bounds
	 */
	void Matrix2::SetRow(int index, const Vector2& row)
	{
		switch (index)
		{
		case 0:
		{
			m11 = row.x;
			m12 = row.y;
			break;
		}
		case 1:
		{
			m21 = row.x;
			m22 = row.y;
			break;
		}
		default:
		{
			throw runtime_error("Index out of bounds!");
		}
		}
	}

	/**
	 * Returns a formatted string representation of the matrix
	 * Format shows matrix in mathematical row/column layout
	 * @return String representation for debugging/display
	 */
	string Matrix2::ToString() const
	{
		return std::format("[\n\t{}, {},\n\t{}, {}\n]", m11, m12, m21, m22);
	}

	/**
	 * Stream output operator for Matrix2
	 * @param stream Output stream to write to
	 * @param matrix Matrix to output
	 * @return Reference to stream for chaining
	 */
	ostream& operator<<(ostream& stream, const Matrix2& matrix)
	{
		stream << matrix.ToString();

		return stream;
	}

	/**
	 * Equality comparison operator using floating-point tolerance
	 * @param rhs Matrix to compare with
	 * @return True if matrices are approximately equal
	 */
	bool Matrix2::operator==(const Matrix2& rhs) const
	{
		return MathF::Compare(m11, rhs.m11) && MathF::Compare(m21, rhs.m21) &&
			MathF::Compare(m12, rhs.m12) && MathF::Compare(m22, rhs.m22);
	}

	/**
	 * Inequality comparison operator
	 * @param rhs Matrix to compare with
	 * @return True if matrices are not approximately equal
	 */
	bool Matrix2::operator!=(const Matrix2& rhs) const
	{
		return !MathF::Compare(m11, rhs.m11) || !MathF::Compare(m21, rhs.m21) ||
			!MathF::Compare(m12, rhs.m12) || !MathF::Compare(m22, rhs.m22);
	}

	/**
	 * Matrix multiplication operator
	 * Performs standard matrix multiplication: this * rhs
	 * @param rhs Right-hand side matrix
	 * @return Product matrix
	 */
	Matrix2 Matrix2::operator*(const Matrix2& rhs) const
	{
		return
		{
			m11 * rhs.m11 + m12 * rhs.m21,
			m11 * rhs.m12 + m12 * rhs.m22,
			m21 * rhs.m11 + m22 * rhs.m21,
			m21 * rhs.m12 + m22 * rhs.m22
		};
	}

	/**
	 * Scalar multiplication operator
	 * Multiplies all matrix elements by scalar value
	 * @param scalar Scalar value to multiply by
	 * @return New matrix with all elements scaled
	 */
	Matrix2 Matrix2::operator*(float scalar) const
	{
		return
		{
			m11 * scalar, m12 * scalar,
			m21 * scalar, m22 * scalar
		};
	}

	/**
	 * Scalar division operator
	 * Divides all matrix elements by scalar value
	 * @param scalar Scalar value to divide by
	 * @return New matrix with all elements divided by scalar
	 * @throws runtime_error if scalar is approximately zero
	 */
	Matrix2 Matrix2::operator/(float scalar) const
	{
		if (MathF::IsNearZero(scalar))
		{
			throw runtime_error("Division by zero!");
		}

		return
		{
			m11 / scalar, m12 / scalar,
			m21 / scalar, m22 / scalar
		};
	}

	/**
	 * Matrix-vector multiplication operator
	 * Treats vector as column vector and performs matrix-vector multiplication
	 * @param rhs Vector to multiply (treated as column vector)
	 * @return Transformed vector
	 */
	Vector2 Matrix2::operator*(const Vector2& rhs) const
	{
		return
		{
			m11 * rhs.x + m12 * rhs.y,
			m21 * rhs.x + m22 * rhs.y
		};
	}

	/**
	 * Column access operator
	 * Returns the specified column as a Vector2
	 * @param index Column index (0 or 1)
	 * @return Column vector at specified index
	 * @throws runtime_error if index is out of bounds
	 */
	Vector2 Matrix2::operator[](int index) const
	{
		return GetColumn(index);
	}

	/**
	 * Assignment operator with self-assignment protection
	 * @param rhs Matrix to assign from
	 * @return Reference to this matrix after assignment
	 */
	Matrix2& Matrix2::operator=(const Matrix2& rhs)
	{
		if (*this == rhs)
		{
			return *this;
		}

		m11 = rhs.m11;
		m12 = rhs.m12;
		m21 = rhs.m21;
		m22 = rhs.m22;

		return *this;
	}

	/**
	 * Global scalar multiplication operator (scalar * matrix)
	 * Allows multiplication in both orders: matrix * scalar and scalar * matrix
	 * @param scalar Scalar value
	 * @param rhs Matrix to multiply
	 * @return Scaled matrix
	 */
	Matrix2 operator*(float scalar, const Matrix2& rhs)
	{
		return rhs * scalar;
	}
}
