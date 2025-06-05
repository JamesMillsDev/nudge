#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

namespace Nudge
{
	class Matrix2;
	class Vector2;
	class Vector3;

    /**
	 * 3x3 Matrix class using column-major storage
	 *
	 * Memory layout: [m11, m21, m31, m12, m22, m32, m13, m23, m33] (column-major)
	 * Mathematical representation:
	 * | m11  m12  m13 |
	 * | m21  m22  m23 |
	 * | m31  m32  m33 |
	 *
	 * Column-major storage means columns are stored contiguously in memory,
	 * which is compatible with OpenGL and most mathematical libraries.
	 * This matches the storage convention used in Matrix2.
	 *
	 * Supports 3D transformations including rotation, scaling, and 2D translation
	 * in homogeneous coordinates.
	 */
    class Matrix3
    {
    public:
        // Column-major storage for 3x3 matrix (consistent with Matrix2)
        float m11, m21, m31,  // Column 1: [m11, m21, m31]
            m12, m22, m32,  // Column 2: [m12, m22, m32]
            m13, m23, m33;  // Column 3: [m13, m23, m33]

    public:
        /**
         * Returns a 3x3 identity matrix
         * | 1  0  0 |
         * | 0  1  0 |
         * | 0  0  1 |
         * @return Identity matrix
         */
        static Matrix3 Identity();

        /**
         * Returns a 3x3 zero matrix
         * | 0  0  0 |
         * | 0  0  0 |
         * | 0  0  0 |
         * @return Zero matrix
         */
        static Matrix3 Zero();

        /**
         * Creates a 3D scale matrix
         * | sx  0   0 |
         * | 0   sy  0 |
         * | 0   0  sz |
         * @param sx X-axis scale factor
         * @param sy Y-axis scale factor
         * @param sz Z-axis scale factor
         * @return Scale matrix
         */
        static Matrix3 Scale(float sx, float sy, float sz);

        /**
         * Creates a 3D scale matrix from Vector3
         * @param scale Vector containing scale factors (x, y, z)
         * @return Scale matrix
         */
        static Matrix3 Scale(const Vector3& scale);

        /**
         * Creates a rotation matrix around the X-axis
         * | 1    0       0    |
         * | 0  cos(θ) -sin(θ) |
         * | 0  sin(θ)  cos(θ) |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative X-axis)
         * @return X-axis rotation matrix
         */
        static Matrix3 RotationX(float degrees);

        /**
         * Creates a rotation matrix around the Y-axis
         * |  cos(θ)  0  sin(θ) |
         * |    0     1    0    |
         * | -sin(θ)  0  cos(θ) |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative Y-axis)
         * @return Y-axis rotation matrix
         */
        static Matrix3 RotationY(float degrees);

        /**
         * Creates a rotation matrix around the Z-axis
         * | cos(θ) -sin(θ)  0 |
         * | sin(θ)  cos(θ)  0 |
         * |   0       0     1 |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative Z-axis)
         * @return Z-axis rotation matrix
         */
        static Matrix3 RotationZ(float degrees);

        /**
         * Creates a rotation matrix from Euler angles (XYZ order)
         * Applies rotations in order: Z * Y * X
         * @param euler Vector containing rotation angles in degrees (x, y, z)
         * @return Combined rotation matrix
         */
        static Matrix3 Rotation(const Vector3& euler);

        /**
         * Creates a rotation matrix around an arbitrary axis
         * Uses Rodrigues' rotation formula
         * @param axis Rotation axis (should be normalized)
         * @param degrees Rotation angle in degrees around the axis
         * @return Axis-angle rotation matrix
         */
        static Matrix3 Rotation(const Vector3& axis, float degrees);

        /**
         * Creates a 2D translation matrix in homogeneous coordinates
         * | 1  0  tx |
         * | 0  1  ty |
         * | 0  0   1 |
         * Useful for 2D transformations in 3x3 homogeneous space
         * @param tx Translation along X-axis
         * @param ty Translation along Y-axis
         * @return 2D translation matrix
         */
        static Matrix3 Translation(float tx, float ty);

        /**
         * Creates a 2D translation matrix from Vector2
         * @param translation Vector containing translation values (x, y)
         * @return 2D translation matrix
         */
        static Matrix3 Translation(const Vector2& translation);

    public:
        /**
         * Default constructor - creates identity matrix
         */
        Matrix3();

        /**
         * Scalar constructor - creates scalar matrix (scalar on diagonal, 0 elsewhere)
         * | scalar    0      0   |
         * |   0    scalar    0   |
         * |   0      0    scalar |
         * @param scalar Value for diagonal elements
         */
        explicit Matrix3(float scalar);

        /**
         * Element-wise constructor with row-by-row input for intuitive use
         * Despite column-major storage, parameters follow row-major order for ease
         * Creates matrix:
         * | m11  m12  m13 |
         * | m21  m22  m23 |
         * | m31  m32  m33 |
         * @param m11 Element at row 1, column 1
         * @param m12 Element at row 1, column 2
         * @param m13 Element at row 1, column 3
         * @param m21 Element at row 2, column 1
         * @param m22 Element at row 2, column 2
         * @param m23 Element at row 2, column 3
         * @param m31 Element at row 3, column 1
         * @param m32 Element at row 3, column 2
         * @param m33 Element at row 3, column 3
         */
        Matrix3(float m11, float m12, float m13, float m21, float m22, float m23, float m31, float m32, float m33);

        /**
         * Column constructor from three Vector3s
         * @param col1 First column vector [m11, m21, m31]
         * @param col2 Second column vector [m12, m22, m32]
         * @param col3 Third column vector [m13, m23, m33]
         */
        Matrix3(const Vector3& col1, const Vector3& col2, const Vector3& col3);

        /**
         * Array constructor from column-major ordered array
         * Expected array layout: [m11, m21, m31, m12, m22, m32, m13, m23, m33]
         * @param values Array of 9 floats in column-major order
         */
        explicit Matrix3(float values[9]);

        /**
         * Constructor to extend a 2x2 matrix to 3x3 with identity
         * | m2.m11  m2.m12   0 |
         * | m2.m21  m2.m22   0 |
         * |   0       0      1 |
         * Note: Matrix2 uses column-major storage, so this mapping is consistent
         * @param matrix 2x2 matrix to extend
         */
        explicit Matrix3(const Matrix2& matrix);

        /**
         * Copy constructor
         * @param rhs Matrix3 to copy from
         */
        Matrix3(const Matrix3& rhs);

    public:
        /**
         * Calculates the determinant of this 3x3 matrix
         * Uses the rule of Sarrus or cofactor expansion
         * det = m11(m22*m33 - m23*m32) - m12(m21*m33 - m23*m31) + m13(m21*m32 - m22*m31)
         * @return Determinant value
         */
        float Determinant() const;

        /**
         * Returns a transposed copy of this matrix (rows become columns)
         * Original matrix remains unchanged
         * @return New transposed matrix
         */
        Matrix3 Transposed() const;

        /**
         * Transposes this matrix in-place (modifies original)
         * Swaps elements across the main diagonal
         */
        void Transpose();

        /**
         * Calculates and returns the inverse of this matrix
         * Uses adjugate matrix method: A^(-1) = (1/det(A)) * adj(A)
         * @return Inverse matrix
         * @throws runtime_error if matrix is not invertible (determinant near 0)
         */
        Matrix3 Inverse() const;

        /**
         * Checks if this matrix is approximately an identity matrix
         * Uses floating-point tolerance for comparison
         * @param tolerance Tolerance for floating-point comparison (default: FLT_EPSILON)
         * @return True if matrix is approximately identity
         */
        bool IsIdentity(float tolerance = FLT_EPSILON) const;

        /**
         * Checks if this matrix is approximately a zero matrix
         * Uses floating-point tolerance for comparison
         * @param tolerance Tolerance for floating-point comparison (default: FLT_EPSILON)
         * @return True if all elements are approximately zero
         */
        bool IsZero(float tolerance = FLT_EPSILON) const;

        /**
         * Checks if this matrix is orthogonal (columns/rows are orthonormal)
         * A matrix is orthogonal if M * M^T = I (within tolerance)
         * Useful for verifying rotation matrices
         * @return True if matrix is orthogonal
         */
        bool IsOrthogonal() const;

        /**
         * Gets a column vector from the matrix
         * @param index Column index (0, 1, or 2)
         * @return Column vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector3 GetColumn(int index) const;

        /**
         * Sets a column in the matrix
         * @param index Column index (0, 1, or 2)
         * @param column New column vector
         * @throws runtime_error if index is out of bounds
         */
        void SetColumn(int index, const Vector3& column);

        /**
         * Gets a row vector from the matrix
         * @param index Row index (0, 1, or 2)
         * @return Row vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector3 GetRow(int index) const;

        /**
         * Sets a row in the matrix
         * @param index Row index (0, 1, or 2)
         * @param row New row vector
         * @throws runtime_error if index is out of bounds
         */
        void SetRow(int index, const Vector3& row);

        /**
         * Returns a formatted string representation of the matrix
         * Format shows matrix in mathematical row/column layout
         * @return String representation for debugging/display
         */
        string ToString() const;

    public:
        /**
         * Stream output operator for Matrix3
         * @param stream Output stream to write to
         * @param matrix Matrix to output
         * @return Reference to stream for chaining
         */
        friend ostream& operator<<(ostream& stream, const Matrix3& matrix);

        /**
         * Equality comparison operator using floating-point tolerance
         * @param rhs Matrix to compare with
         * @return True if matrices are approximately equal
         */
        bool operator==(const Matrix3& rhs) const;

        /**
         * Inequality comparison operator
         * @param rhs Matrix to compare with
         * @return True if matrices are not approximately equal
         */
        bool operator!=(const Matrix3& rhs) const;

        /**
         * Matrix multiplication operator
         * Performs standard matrix multiplication: this * rhs
         * @param rhs Right-hand side matrix
         * @return Product matrix
         */
        Matrix3 operator*(const Matrix3& rhs) const;

        /**
         * Scalar multiplication operator
         * Multiplies all matrix elements by scalar value
         * @param scalar Scalar value to multiply by
         * @return New matrix with all elements scaled
         */
        Matrix3 operator*(float scalar) const;

        /**
         * Scalar division operator
         * Divides all matrix elements by scalar value
         * @param scalar Scalar value to divide by
         * @return New matrix with all elements divided by scalar
         * @throws runtime_error if scalar is approximately zero
         */
        Matrix3 operator/(float scalar) const;

        /**
         * Matrix-vector multiplication operator
         * Treats vector as column vector and performs matrix-vector multiplication
         * @param rhs Vector to multiply (treated as column vector)
         * @return Transformed vector
         */
        Vector3 operator*(const Vector3& rhs) const;

        /**
         * Column access operator
         * Returns the specified column as a Vector3
         * @param index Column index (0, 1, or 2)
         * @return Column vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector3 operator[](int index) const;

        /**
         * Assignment operator with self-assignment protection
         * @param rhs Matrix to assign from
         * @return Reference to this matrix after assignment
         */
        Matrix3& operator=(const Matrix3& rhs);
    };

    /**
     * Global scalar multiplication operator (scalar * matrix)
     * Allows multiplication in both orders: matrix * scalar and scalar * matrix
     * @param scalar Scalar value
     * @param rhs Matrix to multiply
     * @return Scaled matrix
     */
    Matrix3 operator*(float scalar, const Matrix3& rhs);
}
