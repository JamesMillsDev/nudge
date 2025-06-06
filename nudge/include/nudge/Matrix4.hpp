/**
 * @file Matrix4.hpp
 * @brief 4x4 Matrix class for 3D transformations using column-major storage
 * @author JamesMillsAIE
 * @date 2025-06-06
 */

#pragma once

#include <ostream>
#include <string>

using std::ostream;
using std::string;

namespace Nudge
{
    class Matrix2;
    class Matrix3;
    class Vector3;
    class Vector4;

    /**
     * 4x4 Matrix class using column-major storage
     *
     * Memory layout: [m11, m21, m31, m41, m12, m22, m32, m42, m13, m23, m33, m43, m14, m24, m34, m44] (column-major)
     * Mathematical representation:
     * | m11  m12  m13  m14 |
     * | m21  m22  m23  m24 |
     * | m31  m32  m33  m34 |
     * | m41  m42  m43  m44 |
     *
     * Column-major storage means columns are stored contiguously in memory,
     * which is compatible with OpenGL and most mathematical libraries.
     * This matches the storage convention used in Matrix2 and Matrix3.
     *
     * Supports full 3D transformations including rotation, scaling, translation,
     * and perspective projection in homogeneous coordinates.
     */
    class Matrix4
    {
    public:
        // Column-major storage for 4x4 matrix (consistent with Matrix2 and Matrix3)
        float m11, m21, m31, m41,  // Column 1: [m11, m21, m31, m41]
            m12, m22, m32, m42,  // Column 2: [m12, m22, m32, m42]
            m13, m23, m33, m43,  // Column 3: [m13, m23, m33, m43]
            m14, m24, m34, m44;  // Column 4: [m14, m24, m34, m44]

    public:
        /**
         * Returns a 4x4 identity matrix
         * | 1  0  0  0 |
         * | 0  1  0  0 |
         * | 0  0  1  0 |
         * | 0  0  0  1 |
         * @return Identity matrix
         */
        static Matrix4 Identity();

        /**
         * Returns a 4x4 zero matrix
         * | 0  0  0  0 |
         * | 0  0  0  0 |
         * | 0  0  0  0 |
         * | 0  0  0  0 |
         * @return Zero matrix
         */
        static Matrix4 Zero();

        /**
         * Creates a 3D scale matrix
         * | sx  0   0   0 |
         * | 0   sy  0   0 |
         * | 0   0   sz  0 |
         * | 0   0   0   1 |
         * @param sx X-axis scale factor
         * @param sy Y-axis scale factor
         * @param sz Z-axis scale factor
         * @return Scale matrix
         */
        static Matrix4 Scale(float sx, float sy, float sz);

        /**
         * Creates a 3D scale matrix from Vector3
         * @param scale Vector containing scale factors (x, y, z)
         * @return Scale matrix
         */
        static Matrix4 Scale(const Vector3& scale);

        /**
         * Creates a rotation matrix around the X-axis
         * | 1    0       0     0 |
         * | 0  cos(a) -sin(a)  0 |
         * | 0  sin(a)  cos(a)  0 |
         * | 0    0       0     1 |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative X-axis)
         * @return X-axis rotation matrix
         */
        static Matrix4 RotationX(float degrees);

        /**
         * Creates a rotation matrix around the Y-axis
         * |  cos(a)  0  sin(a)  0 |
         * |    0     1    0     0 |
         * | -sin(a)  0  cos(a)  0 |
         * |    0     0    0     1 |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative Y-axis)
         * @return Y-axis rotation matrix
         */
        static Matrix4 RotationY(float degrees);

        /**
         * Creates a rotation matrix around the Z-axis
         * | cos(a) -sin(a)  0  0 |
         * | sin(a)  cos(a)  0  0 |
         * |   0       0     1  0 |
         * |   0       0     0  1 |
         * @param degrees Rotation angle in degrees (positive = counter-clockwise when looking down negative Z-axis)
         * @return Z-axis rotation matrix
         */
        static Matrix4 RotationZ(float degrees);

        /**
         * Creates a rotation matrix from Euler angles (XYZ order)
         * Applies rotations in order: Z * Y * X
         * @param euler Vector containing rotation angles in degrees (x, y, z)
         * @return Combined rotation matrix
         */
        static Matrix4 Rotation(const Vector3& euler);

        /**
         * Creates a rotation matrix around an arbitrary axis
         * Uses Rodrigues' rotation formula
         * @param axis Rotation axis (should be normalized)
         * @param degrees Rotation angle in degrees around the axis
         * @return Axis-angle rotation matrix
         */
        static Matrix4 Rotation(const Vector3& axis, float degrees);

        /**
         * Creates a 3D translation matrix
         * | 1  0  0  tx |
         * | 0  1  0  ty |
         * | 0  0  1  tz |
         * | 0  0  0   1 |
         * @param tx Translation along X-axis
         * @param ty Translation along Y-axis
         * @param tz Translation along Z-axis
         * @return 3D translation matrix
         */
        static Matrix4 Translation(float tx, float ty, float tz);

        /**
         * Creates a 3D translation matrix from Vector3
         * @param translation Vector containing translation values (x, y, z)
         * @return 3D translation matrix
         */
        static Matrix4 Translation(const Vector3& translation);

        /**
         * Creates a "look at" view matrix
         * Standard camera/view matrix for 3D rendering
         * @param eye Camera position
         * @param target Point to look at
         * @param up Up vector (usually (0, 1, 0))
         * @return View matrix
         */
        static Matrix4 LookAt(const Vector3& eye, const Vector3& target, const Vector3& up);

        /**
         * Creates a perspective projection matrix
         * Standard perspective projection for 3D rendering
         * @param fovY Field of view in Y direction (degrees)
         * @param aspectRatio Aspect ratio (width/height)
         * @param nearPlane Near clipping plane distance
         * @param farPlane Far clipping plane distance
         * @return Perspective projection matrix
         */
        static Matrix4 Perspective(float fovY, float aspectRatio, float nearPlane, float farPlane);

        /**
         * Creates an orthographic projection matrix
         * Parallel projection for 2D or isometric 3D rendering
         * @param left Left clipping plane
         * @param right Right clipping plane
         * @param bottom Bottom clipping plane
         * @param top Top clipping plane
         * @param nearPlane Near clipping plane
         * @param farPlane Far clipping plane
         * @return Orthographic projection matrix
         */
        static Matrix4 Orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);

        /**
         * Creates a transformation matrix combining translation, rotation, and scale
         * Applies transformations in order: Scale * Rotation * Translation
         * @param translation Translation vector
         * @param rotation Rotation angles in degrees (Euler angles)
         * @param scale Scale factors
         * @return Combined transformation matrix
         */
        static Matrix4 TRS(const Vector3& translation, const Vector3& rotation, const Vector3& scale);

    public:
        /**
         * Default constructor - creates identity matrix
         */
        Matrix4();

        /**
         * Scalar constructor - creates scalar matrix (scalar on diagonal, 0 elsewhere)
         * | scalar    0      0      0   |
         * |   0    scalar    0      0   |
         * |   0      0    scalar    0   |
         * |   0      0      0    scalar |
         * @param scalar Value for diagonal elements
         */
        explicit Matrix4(float scalar);

        /**
         * Element-wise constructor with row-by-row input for intuitive use
         * Despite column-major storage, parameters follow row-major order for ease
         * Creates matrix:
         * | m11  m12  m13  m14 |
         * | m21  m22  m23  m24 |
         * | m31  m32  m33  m34 |
         * | m41  m42  m43  m44 |
         * @param m11 Element at row 1, column 1
         * @param m12 Element at row 1, column 2
         * @param m13 Element at row 1, column 3
         * @param m14 Element at row 1, column 4
         * @param m21 Element at row 2, column 1
         * @param m22 Element at row 2, column 2
         * @param m23 Element at row 2, column 3
         * @param m24 Element at row 2, column 4
         * @param m31 Element at row 3, column 1
         * @param m32 Element at row 3, column 2
         * @param m33 Element at row 3, column 3
         * @param m34 Element at row 3, column 4
         * @param m41 Element at row 4, column 1
         * @param m42 Element at row 4, column 2
         * @param m43 Element at row 4, column 3
         * @param m44 Element at row 4, column 4
         */
        Matrix4(float m11, float m12, float m13, float m14,
            float m21, float m22, float m23, float m24,
            float m31, float m32, float m33, float m34,
            float m41, float m42, float m43, float m44);

        /**
         * Column constructor from four Vector4s
         * @param col1 First column vector [m11, m21, m31, m41]
         * @param col2 Second column vector [m12, m22, m32, m42]
         * @param col3 Third column vector [m13, m23, m33, m43]
         * @param col4 Fourth column vector [m14, m24, m34, m44]
         */
        Matrix4(const Vector4& col1, const Vector4& col2, const Vector4& col3, const Vector4& col4);

        /**
         * Array constructor from column-major ordered array
         * Expected array layout: [m11, m21, m31, m41, m12, m22, m32, m42, m13, m23, m33, m43, m14, m24, m34, m44]
         * @param values Array of 16 floats in column-major order
         */
        explicit Matrix4(float values[16]);

        /**
         * Constructor to extend a 3x3 matrix to 4x4 with identity
         * | m3.m11  m3.m12  m3.m13   0 |
         * | m3.m21  m3.m22  m3.m23   0 |
         * | m3.m31  m3.m32  m3.m33   0 |
         * |   0       0       0      1 |
         * Note: Matrix3 uses column-major storage, so this mapping is consistent
         * @param matrix 3x3 matrix to extend
         */
        explicit Matrix4(const Matrix3& matrix);

        /**
         * Copy constructor
         * @param rhs Matrix4 to copy from
         */
        Matrix4(const Matrix4& rhs);

    public:
        /**
         * Calculates the determinant of this 4x4 matrix
         * Uses cofactor expansion along the first row
         * @return Determinant value
         */
        float Determinant() const;

        /**
         * Returns a transposed copy of this matrix (rows become columns)
         * Original matrix remains unchanged
         * @return New transposed matrix
         */
        Matrix4 Transposed() const;

        /**
         * Transposes this matrix in-place (modifies original)
         * Swaps elements across the main diagonal
         */
        void Transpose();

        /**
         * @brief Calculates the cofactor matrix of this 4x4 matrix.
         *
         * The cofactor matrix is computed by calculating the cofactor of each element,
         * where the cofactor C_ij = (-1)^(i+j) * M_ij, and M_ij is the minor
         * (determinant of the 3x3 submatrix obtained by removing row i and column j).
         *
         * @return Matrix4 The cofactor matrix
         */
        Matrix4 Cofactor() const;

        /**
         * @brief Calculates the adjugate (adjoint) matrix of this 4x4 matrix.
         *
         * The adjugate matrix is the transpose of the cofactor matrix. It is used
         * in computing the matrix inverse: A^(-1) = (1/det(A)) * adj(A).
         *
         * @return Matrix4 The adjugate matrix (transpose of the cofactor matrix)
         */
        Matrix4 Adjugate() const;

        /**
         * Calculates and returns the inverse of this matrix
         * Uses adjugate matrix method: A^(-1) = (1/det(A)) * adj(A)
         * @return Inverse matrix
         * @throws runtime_error if matrix is not invertible (determinant near 0)
         */
        Matrix4 Inverse() const;

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
         * Extracts the translation component from a transformation matrix
         * Returns the fourth column (without w component) as translation vector
         * @return Translation vector (x, y, z)
         */
        Vector3 GetTranslation() const;

        /**
         * Extracts the scale component from a transformation matrix
         * Calculates the length of each column vector (excluding translation)
         * @return Scale vector (x, y, z)
         */
        Vector3 GetScale() const;

        /**
         * Extracts the rotation component as a 3x3 matrix
         * Removes translation and normalizes scale from the transformation matrix
         * @return 3x3 rotation matrix
         */
        Matrix3 GetRotation() const;

        /**
         * Gets a column vector from the matrix
         * @param index Column index (0, 1, 2, or 3)
         * @return Column vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector4 GetColumn(int index) const;

        /**
         * Sets a column in the matrix
         * @param index Column index (0, 1, 2, or 3)
         * @param column New column vector
         * @throws runtime_error if index is out of bounds
         */
        void SetColumn(int index, const Vector4& column);

        /**
         * Gets a row vector from the matrix
         * @param index Row index (0, 1, 2, or 3)
         * @return Row vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector4 GetRow(int index) const;

        /**
         * Sets a row in the matrix
         * @param index Row index (0, 1, 2, or 3)
         * @param row New row vector
         * @throws runtime_error if index is out of bounds
         */
        void SetRow(int index, const Vector4& row);

        /**
         * Returns a formatted string representation of the matrix
         * Format shows matrix in mathematical row/column layout
         * @return String representation for debugging/display
         */
        string ToString() const;

    public:
        /**
         * Stream output operator for Matrix4
         * @param stream Output stream to write to
         * @param matrix Matrix to output
         * @return Reference to stream for chaining
         */
        friend ostream& operator<<(ostream& stream, const Matrix4& matrix);

        /**
         * Equality comparison operator using floating-point tolerance
         * @param rhs Matrix to compare with
         * @return True if matrices are approximately equal
         */
        bool operator==(const Matrix4& rhs) const;

        /**
         * Inequality comparison operator
         * @param rhs Matrix to compare with
         * @return True if matrices are not approximately equal
         */
        bool operator!=(const Matrix4& rhs) const;

        /**
         * Matrix multiplication operator
         * Performs standard matrix multiplication: this * rhs
         * @param rhs Right-hand side matrix
         * @return Product matrix
         */
        Matrix4 operator*(const Matrix4& rhs) const;

        /**
         * Scalar multiplication operator
         * Multiplies all matrix elements by scalar value
         * @param scalar Scalar value to multiply by
         * @return New matrix with all elements scaled
         */
        Matrix4 operator*(float scalar) const;

        /**
         * Scalar division operator
         * Divides all matrix elements by scalar value
         * @param scalar Scalar value to divide by
         * @return New matrix with all elements divided by scalar
         * @throws runtime_error if scalar is approximately zero
         */
        Matrix4 operator/(float scalar) const;

        /**
         * Matrix-vector multiplication operator (Vector4)
         * Treats vector as column vector and performs matrix-vector multiplication
         * @param rhs Vector4 to multiply (treated as column vector)
         * @return Transformed Vector4
         */
        Vector4 operator*(const Vector4& rhs) const;

        /**
         * Matrix-vector multiplication operator (Vector3)
         * Treats Vector3 as homogeneous coordinate with w=1
         * Performs matrix-vector multiplication and returns transformed Vector3
         * @param rhs Vector3 to multiply (treated as [x, y, z, 1])
         * @return Transformed Vector3 (with w-division if perspective)
         */
        Vector3 operator*(const Vector3& rhs) const;

        /**
         * Column access operator
         * Returns the specified column as a Vector4
         * @param index Column index (0, 1, 2, or 3)
         * @return Column vector at specified index
         * @throws runtime_error if index is out of bounds
         */
        Vector4 operator[](int index) const;

        /**
         * Assignment operator with self-assignment protection
         * @param rhs Matrix to assign from
         * @return Reference to this matrix after assignment
         */
        Matrix4& operator=(const Matrix4& rhs);
    };

    /**
     * Global scalar multiplication operator (scalar * matrix)
     * Allows multiplication in both orders: matrix * scalar and scalar * matrix
     * @param scalar Scalar value
     * @param rhs Matrix to multiply
     * @return Scaled matrix
     */
    Matrix4 operator*(float scalar, const Matrix4& rhs);
}