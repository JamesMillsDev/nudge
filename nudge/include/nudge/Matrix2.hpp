#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

namespace Nudge
{
	class Vector2;

	/**
	 * 2x2 Matrix class using column-major storage
	 *
	 * Memory layout: [m11, m21, m12, m22] (column-major)
	 * Mathematical representation:
	 * | m11  m12 |
	 * | m21  m22 |
	 *
	 * Column-major storage means columns are stored contiguously in memory,
	 * which is compatible with OpenGL and most mathematical libraries.
	 */
	class Matrix2
	{
	public:
		// Column-major storage for 2x2 matrix
		float m11, m21, // Column 1: [m11, m21]
		      m12, m22; // Column 2: [m12, m22]

	public:
		/**
		 * Returns a 2x2 identity matrix
		 * | 1  0 |
		 * | 0  1 |
		 * @return Identity matrix
		 */
		static Matrix2 Identity();

		/**
		 * Returns a 2x2 zero matrix
		 * | 0  0 |
		 * | 0  0 |
		 * @return Zero matrix
		 */
		static Matrix2 Zero();

		/**
		 * Creates a 2D scale matrix with separate x and y scale factors
		 * | sx  0 |
		 * | 0  sy |
		 * @param sx Scale factor for x-axis
		 * @param sy Scale factor for y-axis
		 * @return Scale matrix
		 */
		static Matrix2 Scale(float sx, float sy);

		/**
		 * Creates a 2D scale matrix from a Vector2
		 * @param scale Vector containing x and y scale factors
		 * @return Scale matrix
		 */
		static Matrix2 Scale(const Vector2& scale);

		/**
		 * Creates a 2D rotation matrix for counter-clockwise rotation
		 * | cos(θ) -sin(θ) |
		 * | sin(θ)  cos(θ) |
		 * @param degrees Rotation angle in degrees (positive = counter-clockwise)
		 * @return Rotation matrix
		 */
		static Matrix2 Rotation(float degrees);

	public:
		/**
		 * Default constructor - creates identity matrix
		 */
		Matrix2();

		/**
		 * Constructor that creates a scalar matrix (scalar on diagonal, 0 elsewhere)
		 * | scalar    0   |
		 * |   0    scalar |
		 * @param scalar Value to place on the diagonal
		 */
		explicit Matrix2(float scalar);

		/**
		 * Constructor with explicit element values (row-by-row input for intuitive use)
		 * Creates matrix:
		 * | m11  m12 |
		 * | m21  m22 |
		 * Note: Despite column-major storage, parameters are ordered row-by-row for ease of use
		 * @param m11 Element at row 1, column 1
		 * @param m12 Element at row 1, column 2
		 * @param m21 Element at row 2, column 1
		 * @param m22 Element at row 2, column 2
		 */
		Matrix2(float m11, float m12, float m21, float m22);

		/**
		 * Constructor from two column vectors
		 * @param col1 First column vector [m11, m21]
		 * @param col2 Second column vector [m12, m22]
		 */
		Matrix2(const Vector2& col1, const Vector2& col2);

		/**
		 * Constructor from array of floats in column-major order
		 * Array should contain [m11, m21, m12, m22]
		 * @param values Array of 4 float values in column-major order
		 */
		explicit Matrix2(float values[4]);

		/**
		 * Copy constructor
		 * @param rhs Matrix2 to copy from
		 */
		Matrix2(const Matrix2& rhs);

	public:
		/**
		 * Calculates the determinant of this 2x2 matrix
		 * @return The determinant value
		 */
		float Determinant() const;

		/**
		 * Returns a transposed copy of this matrix (rows become columns)
		 * Original matrix remains unchanged
		 * @return New transposed matrix
		 */
		Matrix2 Transposed() const;

		/**
		 * Transposes this matrix in-place (modifies the original)
		 * Swaps m12 and m21
		 */
		void Transpose();

		/**
		 * Calculates and returns the inverse of this matrix
		 * @return Inverse matrix
		 * @throws runtime_error if matrix is not invertible (determinant approx. 0)
		 */
		Matrix2 Inverse() const;

		/**
		 * Checks if this matrix is an identity matrix (within floating-point tolerance)
		 * @param tolerance The sensitivity of the comparisons
		 * @return True if this matrix is approximately the identity matrix
		 */
		bool IsIdentity(float tolerance = FLT_EPSILON) const;

		/**
		 * Checks if this matrix is a zero matrix (within floating-point tolerance)
		 * @param tolerance The sensitivity of the comparisons
		 * @return True if all elements are approximately zero
		 */
		bool IsZero(float tolerance = FLT_EPSILON) const;

		/**
		 * Gets a column vector from the matrix
		 * @param index Column index (0 for first column, 1 for second column)
		 * @return Column vector at the specified index
		 * @throws runtime_error if index is out of bounds (not 0 or 1)
		 */
		Vector2 GetColumn(int index) const;

		/**
		 * Sets a column in the matrix
		 * @param index Column index (0 for first column, 1 for second column)
		 * @param column Vector to set as the column
		 * @throws runtime_error if index is out of bounds (not 0 or 1)
		 */
		void SetColumn(int index, const Vector2& column);

		/**
		 * Gets a row vector from the matrix
		 * @param index Row index (0 for first row, 1 for second row)
		 * @return Row vector at the specified index
		 * @throws runtime_error if index is out of bounds (not 0 or 1)
		 */
		Vector2 GetRow(int index) const;

		/**
		 * Sets a row in the matrix
		 * @param index Row index (0 for first row, 1 for second row)
		 * @param row Vector to set as the row (parameter should be renamed to 'row')
		 * @throws runtime_error if index is out of bounds (not 0 or 1)
		 */
		void SetRow(int index, const Vector2& row);

		/**
		 * Returns a string representation of this matrix for debugging/display
		 * @return String representation showing matrix elements in readable format
		 */
		string ToString() const;

	public:
		/**
		 * Stream output operator for Matrix2
		 * @param stream Output stream to write to
		 * @param matrix Matrix to output
		 * @return Reference to the stream for chaining
		 */
		friend ostream& operator<<(ostream& stream, const Matrix2& matrix);

		/**
		 * Equality comparison operator (uses floating-point tolerance)
		 * @param rhs Matrix to compare with
		 * @return True if matrices are approximately equal within tolerance
		 */
		bool operator==(const Matrix2& rhs) const;

		/**
		 * Inequality comparison operator
		 * @param rhs Matrix to compare with
		 * @return True if matrices are not approximately equal
		 */
		bool operator!=(const Matrix2& rhs) const;

		/**
		 * Matrix multiplication operator
		 * Performs standard matrix multiplication: this * rhs
		 * @param rhs Matrix to multiply with (right-hand side)
		 * @return Result of matrix multiplication
		 */
		Matrix2 operator*(const Matrix2& rhs) const;

		/**
		 * Scalar multiplication operator
		 * Multiplies all matrix elements by the scalar value
		 * @param scalar Scalar value to multiply by
		 * @return New matrix with all elements multiplied by scalar
		 */
		Matrix2 operator*(float scalar) const;

		/**
		 * Scalar division operator
		 * Divides all matrix elements by the scalar value
		 * @param scalar Scalar value to divide by (must not be zero)
		 * @return New matrix with all elements divided by scalar
		 * @throws May throw if scalar is approximately zero
		 */
		Matrix2 operator/(float scalar) const;

		/**
		 * Matrix-vector multiplication operator
		 * Treats the vector as a column vector and performs matrix-vector multiplication
		 * Result = this * vector
		 * @param rhs Vector to multiply with (treated as column vector)
		 * @return Resulting vector after transformation
		 */
		Vector2 operator*(const Vector2& rhs) const;

		/**
		 * Column access operator
		 * Returns the specified column as a Vector2
		 * @param index Column index (0 for first column, 1 for second column)
		 * @return Column vector at the specified index
		 * @throws runtime_error if index is out of bounds (not 0 or 1)
		 */
		Vector2 operator[](int index) const;

		/**
		 * Assignment operator
		 * @param rhs Matrix to assign from
		 * @return Reference to this matrix after assignment
		 */
		Matrix2& operator=(const Matrix2& rhs);
	};

	/**
	 * Scalar multiplication operator
	 * Multiplies all matrix elements by the scalar value
	 * @param scalar Scalar value to multiply by
	 * @param rhs The matrix to apply the scalar to
	 * @return New matrix with all elements multiplied by scalar
	 */
	Matrix2 operator*(float scalar, const Matrix2& rhs);
}
