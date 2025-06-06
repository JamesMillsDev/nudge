#include <numbers>

#include "CppUnitTest.h"

#include "Nudge/MathF.hpp"
#include "Nudge/Quaternion.hpp"
#include "Nudge/Vector3.hpp"
#include "Nudge/Matrix3.hpp"
#include "Nudge/Matrix4.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using std::runtime_error;
using std::numbers::pi_v;

namespace Nudge
{
    TEST_CLASS(QuaternionTests)
    {
    public:
        // Helper method for floating point comparison
        static void AssertFloatEqual(const float expected, const float actual, const float tolerance = 0.0001f)
        {
            Assert::IsTrue(MathF::Compare(expected, actual, tolerance), L"Float values are not equal within tolerance");
        }

        static void AssertVector3Equal(const Vector3& expected, const Vector3& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
        }

        static void AssertQuaternionEqual(const Quaternion& expected, const Quaternion& actual, float tolerance = 0.0001f)
        {
            AssertFloatEqual(expected.x, actual.x, tolerance);
            AssertFloatEqual(expected.y, actual.y, tolerance);
            AssertFloatEqual(expected.z, actual.z, tolerance);
            AssertFloatEqual(expected.w, actual.w, tolerance);
        }

        // Static Factory Method Tests
        TEST_METHOD(Identity_CreatesIdentityQuaternion)
        {
            Quaternion identity = Quaternion::Identity();
            AssertQuaternionEqual(Quaternion(0.0f, 0.0f, 0.0f, 1.0f), identity);
        }

        TEST_METHOD(FromAxisAngle_ZeroAngle_ReturnsIdentity)
        {
            Vector3 axis(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::FromAxisAngle(axis, 0.0f);
            AssertQuaternionEqual(Quaternion::Identity(), quat);
        }

        TEST_METHOD(FromAxisAngle_90DegreesAroundY_CreatesCorrectQuaternion)
        {
            Vector3 axis(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::FromAxisAngle(axis, 90.0f);
            // Note: Your constructor has a bug - it uses Cos for both halfCos and halfSin
            // The test expects the correct mathematical result, not the buggy implementation
            float expected = MathF::Sqrt(2.0f) / 2.0f;
            AssertQuaternionEqual(Quaternion(0.0f, expected, 0.0f, expected), quat, 0.001f);
        }

        TEST_METHOD(FromAxisAngle_180DegreesAroundX_CreatesCorrectQuaternion)
        {
            Vector3 axis(1.0f, 0.0f, 0.0f);
            Quaternion quat = Quaternion::FromAxisAngle(axis, 180.0f);
            // 180 degrees = pi radians, sin(pi/2) = 1, cos(pi/2) = 0
            AssertQuaternionEqual(Quaternion(1.0f, 0.0f, 0.0f, 0.0f), quat, 0.001f);
        }

        TEST_METHOD(FromEuler_ZeroRotation_ReturnsIdentity)
        {
            Vector3 euler(0.0f, 0.0f, 0.0f);
            Quaternion quat = Quaternion::FromEuler(euler);
            AssertQuaternionEqual(Quaternion::Identity(), quat);
        }

        TEST_METHOD(FromEuler_90DegreesY_CreatesCorrectQuaternion)
        {
            Vector3 euler(0.0f, 90.0f, 0.0f);
            Quaternion quat = Quaternion::FromEuler(euler);
            float expected = MathF::Sqrt(2.0f) / 2.0f;
            AssertQuaternionEqual(Quaternion(0.0f, expected, 0.0f, expected), quat, 0.001f);
        }

        // Note: These tests will fail until you implement the missing methods
        TEST_METHOD(FromToRotation_SameVector_ReturnsIdentity)
        {
            Vector3 from(1.0f, 0.0f, 0.0f);
            Vector3 to(1.0f, 0.0f, 0.0f);
            Quaternion quat = Quaternion::FromToRotation(from, to);
            AssertQuaternionEqual(Quaternion::Identity(), quat);
        }

        TEST_METHOD(FromToRotation_OppositeVectors_Returns180DegreeRotation)
        {
            Vector3 from(1.0f, 0.0f, 0.0f);
            Vector3 to(-1.0f, 0.0f, 0.0f);
            Quaternion quat = Quaternion::FromToRotation(from, to);
            // Should be 180 degree rotation around Y or Z axis
            AssertFloatEqual(0.0f, quat.w, 0.001f); // 180 degrees has w=0
        }

        TEST_METHOD(FromToRotation_90DegreeRotation_CreatesCorrectQuaternion)
        {
            Vector3 from(1.0f, 0.0f, 0.0f);
            Vector3 to(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::FromToRotation(from, to);
            float expected = MathF::Sqrt(2.0f) / 2.0f;
            // Should be 90 degree rotation around Z axis
            AssertQuaternionEqual(Quaternion(0.0f, 0.0f, expected, expected), quat, 0.001f);
        }

        TEST_METHOD(LookRotation_ForwardZ_ReturnsIdentity)
        {
            Vector3 forward(0.0f, 0.0f, 1.0f);
            Vector3 up(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::LookRotation(forward, up);
            AssertQuaternionEqual(Quaternion::Identity(), quat);
        }

        TEST_METHOD(LookRotation_ForwardX_Creates90DegreeYRotation)
        {
            Vector3 forward(1.0f, 0.0f, 0.0f);
            Vector3 up(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::LookRotation(forward, up);
            float expected = MathF::Sqrt(2.0f) / 2.0f;
            AssertQuaternionEqual(Quaternion(0.0f, expected, 0.0f, expected), quat, 0.001f);
        }

        // Interpolation Tests (these will fail until implemented)
        TEST_METHOD(Lerp_T0_ReturnsFirstQuaternion)
        {
            Quaternion a(0.0f, 0.0f, 0.0f, 1.0f);
            Quaternion b(1.0f, 0.0f, 0.0f, 0.0f);
            Quaternion result = Quaternion::Lerp(a, b, 0.0f);
            AssertQuaternionEqual(a, result);
        }

        TEST_METHOD(Lerp_T1_ReturnsSecondQuaternion)
        {
            Quaternion a(0.0f, 0.0f, 0.0f, 1.0f);
            Quaternion b(1.0f, 0.0f, 0.0f, 0.0f);
            Quaternion result = Quaternion::Lerp(a, b, 1.0f);
            AssertQuaternionEqual(b, result);
        }

        TEST_METHOD(Lerp_THalf_ReturnsMiddleValue)
        {
            Quaternion a(0.0f, 0.0f, 0.0f, 1.0f);
            Quaternion b(1.0f, 0.0f, 0.0f, 0.0f);
            Quaternion result = Quaternion::Lerp(a, b, 0.5f);
            // Note: Lerp result should be normalized, so this might not be exactly (0.5, 0, 0, 0.5)
            AssertQuaternionEqual(Quaternion(0.5f, 0.0f, 0.0f, 0.5f), result);
        }

        TEST_METHOD(LerpUnclamped_TNegative_ExtrapolatesBackward)
        {
            Quaternion a(0.0f, 0.0f, 0.0f, 1.0f);
            Quaternion b(1.0f, 0.0f, 0.0f, 0.0f);
            Quaternion result = Quaternion::LerpUnclamped(a, b, -0.5f);
            AssertQuaternionEqual(Quaternion(-0.5f, 0.0f, 0.0f, 1.5f), result);
        }

        TEST_METHOD(LerpUnclamped_TGreaterThan1_ExtrapolatesForward)
        {
            Quaternion a(0.0f, 0.0f, 0.0f, 1.0f);
            Quaternion b(1.0f, 0.0f, 0.0f, 0.0f);
            Quaternion result = Quaternion::LerpUnclamped(a, b, 1.5f);
            AssertQuaternionEqual(Quaternion(1.5f, 0.0f, 0.0f, -0.5f), result);
        }

        TEST_METHOD(Slerp_T0_ReturnsFirstQuaternion)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Quaternion result = Quaternion::Slerp(a, b, 0.0f);
            AssertQuaternionEqual(a, result);
        }

        TEST_METHOD(Slerp_T1_ReturnsSecondQuaternion)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Quaternion result = Quaternion::Slerp(a, b, 1.0f);
            AssertQuaternionEqual(b, result);
        }

        TEST_METHOD(Slerp_THalf_ReturnsHalfwayRotation)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Quaternion result = Quaternion::Slerp(a, b, 0.5f);
            Quaternion expected = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 45.0f);
            AssertQuaternionEqual(expected, result, 0.001f);
        }

        TEST_METHOD(SlerpUnclamped_TNegative_ExtrapolatesBackward)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 45.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Quaternion result = Quaternion::SlerpUnclamped(a, b, -1.0f);
            Quaternion expected = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
            AssertQuaternionEqual(expected, result, 0.001f);
        }

        // Constructor Tests
        TEST_METHOD(Constructor_Default_CreatesIdentityQuaternion)
        {
            Quaternion quat;
            AssertQuaternionEqual(Quaternion::Identity(), quat);
        }

        TEST_METHOD(Constructor_FourFloats_CreatesQuaternionWithSpecifiedValues)
        {
            Quaternion quat(0.1f, 0.2f, 0.3f, 0.4f);
            AssertQuaternionEqual(Quaternion(0.1f, 0.2f, 0.3f, 0.4f), quat);
        }

        TEST_METHOD(Constructor_AxisAngle_CreatesCorrectQuaternion)
        {
            Vector3 axis(0.0f, 1.0f, 0.0f);
            Quaternion quat(axis, 90.0f);
            Quaternion expected = Quaternion::FromAxisAngle(axis, 90.0f);
            AssertQuaternionEqual(expected, quat);
        }

        TEST_METHOD(Constructor_Copy_CreatesIdenticalQuaternion)
        {
            Quaternion original(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion copy(original);
            AssertQuaternionEqual(original, copy);
        }

        // Conversion Method Tests (these will fail until implemented)
        TEST_METHOD(Euler_IdentityQuaternion_ReturnsZeroEuler)
        {
            Quaternion identity = Quaternion::Identity();
            Vector3 euler = identity.Euler();
            AssertVector3Equal(Vector3(0.0f, 0.0f, 0.0f), euler, 0.001f);
        }

        TEST_METHOD(Euler_90DegreeYRotation_ReturnsCorrectEuler)
        {
            Quaternion quat = Quaternion::FromEuler(Vector3(0.0f, 90.0f, 0.0f));
            Vector3 euler = quat.Euler();
            AssertVector3Equal(Vector3(0.0f, 90.0f, 0.0f), euler, 0.001f);
        }

        TEST_METHOD(ToMatrix3_IdentityQuaternion_ReturnsIdentityMatrix)
        {
            Quaternion identity = Quaternion::Identity();
            Matrix3 matrix = identity.ToMatrix3();
            Assert::IsTrue(matrix.IsIdentity(0.001f));
        }

        TEST_METHOD(ToMatrix3_90DegreeYRotation_ReturnsCorrectMatrix)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Matrix3 matrix = quat.ToMatrix3();
            Matrix3 expected = Matrix3::RotationY(90.0f);
            // Compare by transforming a test vector
            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 result1 = matrix * testVector;
            Vector3 result2 = expected * testVector;
            AssertVector3Equal(result2, result1, 0.001f);
        }

        TEST_METHOD(ToMatrix4_IdentityQuaternion_ReturnsIdentityMatrix)
        {
            Quaternion identity = Quaternion::Identity();
            Matrix4 matrix = identity.ToMatrix4();
            Assert::IsTrue(matrix.IsIdentity(0.001f));
        }

        TEST_METHOD(ToMatrix4_90DegreeXRotation_ReturnsCorrectMatrix)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 90.0f);
            Matrix4 matrix = quat.ToMatrix4();
            Matrix4 expected = Matrix4::RotationX(90.0f);
            // Compare by transforming a test vector
            Vector3 testVector(0.0f, 1.0f, 0.0f);
            Vector3 result1 = matrix * testVector;
            Vector3 result2 = expected * testVector;
            AssertVector3Equal(result2, result1, 0.001f);
        }

        // Operator Tests (these will fail until implemented)
        TEST_METHOD(QuaternionMultiplication_IdentityQuaternion_ReturnsOriginal)
        {
            Quaternion quat(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion identity = Quaternion::Identity();
            Quaternion result = quat * identity;
            AssertQuaternionEqual(quat, result);
        }

        TEST_METHOD(QuaternionMultiplication_TwoRotations_CombinesRotations)
        {
            Quaternion rotX = Quaternion::FromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 90.0f);
            Quaternion rotY = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Quaternion combined = rotY * rotX; // Apply X rotation first, then Y

            // Test by rotating a vector
            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 result = combined * testVector;
            // After X rotation: (1,0,0) -> (1,0,0) (no change)
            // After Y rotation: (1,0,0) -> (0,0,-1)
            AssertVector3Equal(Vector3(0.0f, 0.0f, -1.0f), result, 0.001f);
        }

        TEST_METHOD(VectorMultiplication_IdentityQuaternion_ReturnsOriginalVector)
        {
            Quaternion identity = Quaternion::Identity();
            Vector3 vector(1.0f, 2.0f, 3.0f);
            Vector3 result = identity * vector;
            AssertVector3Equal(vector, result);
        }

        TEST_METHOD(VectorMultiplication_90DegreeXRotation_RotatesVector)
        {
            Quaternion rotX = Quaternion::FromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 90.0f);
            Vector3 vector(0.0f, 1.0f, 0.0f);
            Vector3 result = rotX * vector;
            AssertVector3Equal(Vector3(0.0f, 0.0f, 1.0f), result, 0.001f);
        }

        TEST_METHOD(VectorMultiplication_90DegreeYRotation_RotatesVector)
        {
            Quaternion rotY = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Vector3 vector(1.0f, 0.0f, 0.0f);
            Vector3 result = rotY * vector;
            AssertVector3Equal(Vector3(0.0f, 0.0f, -1.0f), result, 0.001f);
        }

        TEST_METHOD(VectorMultiplication_90DegreeZRotation_RotatesVector)
        {
            Quaternion rotZ = Quaternion::FromAxisAngle(Vector3(0.0f, 0.0f, 1.0f), 90.0f);
            Vector3 vector(1.0f, 0.0f, 0.0f);
            Vector3 result = rotZ * vector;
            AssertVector3Equal(Vector3(0.0f, 1.0f, 0.0f), result, 0.001f);
        }

        // Edge Case Tests
        TEST_METHOD(EdgeCase_VerySmallRotation_HandledCorrectly)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.001f);
            Vector3 euler = quat.Euler();
            AssertVector3Equal(Vector3(0.0f, 0.001f, 0.0f), euler, 0.01f);
        }

        TEST_METHOD(EdgeCase_NegativeAngle_CreatesCorrectQuaternion)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), -90.0f);
            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 result = quat * testVector;
            AssertVector3Equal(Vector3(0.0f, 0.0f, 1.0f), result, 0.001f);
        }

        TEST_METHOD(EdgeCase_LargeAngle_NormalizedCorrectly)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 450.0f); // 450 = 90 + 360
            Quaternion expected = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 result1 = quat * testVector;
            Vector3 result2 = expected * testVector;
            AssertVector3Equal(result2, result1, 0.001f);
        }

        // Mathematical Property Tests
        TEST_METHOD(Property_FromEulerToEuler_RoundTrip)
        {
            Vector3 originalEuler(30.0f, 45.0f, 60.0f);
            Quaternion quat = Quaternion::FromEuler(originalEuler);
            Vector3 convertedEuler = quat.Euler();
            AssertVector3Equal(originalEuler, convertedEuler, 0.001f);
        }

        TEST_METHOD(Property_FromAxisAngleToMatrix_Consistency)
        {
            Vector3 axis(0.0f, 1.0f, 0.0f);
            float angle = 45.0f;
            Quaternion quat = Quaternion::FromAxisAngle(axis, angle);
            Matrix3 quatMatrix = quat.ToMatrix3();
            Matrix3 directMatrix = Matrix3::Rotation(axis, angle);

            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 result1 = quatMatrix * testVector;
            Vector3 result2 = directMatrix * testVector;
            AssertVector3Equal(result2, result1, 0.001f);
        }

        TEST_METHOD(Property_QuaternionMultiplication_Associativity)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(1.0f, 0.0f, 0.0f), 30.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 45.0f);
            Quaternion c = Quaternion::FromAxisAngle(Vector3(0.0f, 0.0f, 1.0f), 60.0f);

            Quaternion result1 = (a * b) * c;
            Quaternion result2 = a * (b * c);

            Vector3 testVector(1.0f, 0.0f, 0.0f);
            Vector3 transformed1 = result1 * testVector;
            Vector3 transformed2 = result2 * testVector;
            AssertVector3Equal(transformed2, transformed1, 0.001f);
        }

        TEST_METHOD(Property_IdentityIsMultiplicativeIdentity)
        {
            Quaternion quat = Quaternion::FromAxisAngle(Vector3(1.0f, 1.0f, 1.0f), 45.0f);
            Quaternion identity = Quaternion::Identity();

            Quaternion leftProduct = identity * quat;
            Quaternion rightProduct = quat * identity;

            AssertQuaternionEqual(quat, leftProduct);
            AssertQuaternionEqual(quat, rightProduct);
        }

        TEST_METHOD(Property_SlerpIsCommutativeForT)
        {
            Quaternion a = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
            Quaternion b = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);

            Quaternion result1 = Quaternion::Slerp(a, b, 0.3f);
            Quaternion result2 = Quaternion::Slerp(b, a, 0.7f);

            AssertQuaternionEqual(result1, result2, 0.001f);
        }

        TEST_METHOD(Property_FromToRotation_ActuallyRotates)
        {
            Vector3 from(1.0f, 0.0f, 0.0f);
            Vector3 to(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::FromToRotation(from, to);

            Vector3 result = quat * from;
            AssertVector3Equal(to.Normalized(), result.Normalized(), 0.001f);
        }

        TEST_METHOD(Property_LookRotation_LooksInCorrectDirection)
        {
            Vector3 forward(1.0f, 0.0f, 0.0f);
            Vector3 up(0.0f, 1.0f, 0.0f);
            Quaternion quat = Quaternion::LookRotation(forward, up);

            Vector3 transformedForward = quat * Vector3(0.0f, 0.0f, 1.0f); // Default forward
            AssertVector3Equal(forward.Normalized(), transformedForward.Normalized(), 0.001f);
        }

        // Integration Tests
        TEST_METHOD(Integration_QuaternionToMatrixToVector_ConsistentWithDirectRotation)
        {
            Vector3 axis(1.0f, 1.0f, 1.0f);
            float angle = 120.0f;
            Vector3 testVector(1.0f, 0.0f, 0.0f);

            // Direct quaternion rotation
            Quaternion quat = Quaternion::FromAxisAngle(axis, angle);
            Vector3 result1 = quat * testVector;

            // Via matrix conversion
            Matrix3 matrix = quat.ToMatrix3();
            Vector3 result2 = matrix * testVector;

            AssertVector3Equal(result1, result2, 0.001f);
        }

        TEST_METHOD(Integration_EulerToQuaternionToMatrix_PreservesRotation)
        {
            Vector3 euler(30.0f, 45.0f, 60.0f);
            Vector3 testVector(1.0f, 0.0f, 0.0f);

            // Via quaternion
            Quaternion quat = Quaternion::FromEuler(euler);
            Vector3 result1 = quat * testVector;

            // Via direct matrix
            Matrix3 matrix = Matrix3::Rotation(euler);
            Vector3 result2 = matrix * testVector;

            AssertVector3Equal(result2, result1, 0.001f);
        }

        // Equality and Inequality Operator Tests
        TEST_METHOD(Equality_SameQuaternions_ReturnsTrue)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.1f, 0.2f, 0.3f, 0.4f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Equality_DifferentQuaternions_ReturnsFalse)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.1f, 0.2f, 0.3f, 0.5f);
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Equality_IdentityQuaternions_ReturnsTrue)
        {
            Quaternion identity1 = Quaternion::Identity();
            Quaternion identity2 = Quaternion::Identity();
            Assert::IsTrue(identity1 == identity2);
        }

        TEST_METHOD(Equality_NearEqualQuaternions_ReturnsTrue)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.100001f, 0.200001f, 0.300001f, 0.400001f);
            Assert::IsTrue(a == b); // Should be within default tolerance
        }

        TEST_METHOD(Equality_SlightlyDifferentQuaternions_ReturnsFalse)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.11f, 0.2f, 0.3f, 0.4f); // Outside tolerance
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(Equality_SelfComparison_ReturnsTrue)
        {
            Quaternion quat(0.1f, 0.2f, 0.3f, 0.4f);
            Assert::IsTrue(quat == quat);
        }

        TEST_METHOD(Inequality_DifferentQuaternions_ReturnsTrue)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.1f, 0.2f, 0.3f, 0.5f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(Inequality_SameQuaternions_ReturnsFalse)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.1f, 0.2f, 0.3f, 0.4f);
            Assert::IsFalse(a != b);
        }

        TEST_METHOD(Inequality_IdentityQuaternions_ReturnsFalse)
        {
            Quaternion identity1 = Quaternion::Identity();
            Quaternion identity2 = Quaternion::Identity();
            Assert::IsFalse(identity1 != identity2);
        }

        TEST_METHOD(Inequality_NearEqualQuaternions_ReturnsFalse)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.100001f, 0.200001f, 0.300001f, 0.400001f);
            Assert::IsFalse(a != b); // Should be within default tolerance
        }

        TEST_METHOD(Inequality_SlightlyDifferentQuaternions_ReturnsTrue)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(0.11f, 0.2f, 0.3f, 0.4f); // Outside tolerance
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(Inequality_SelfComparison_ReturnsFalse)
        {
            Quaternion quat(0.1f, 0.2f, 0.3f, 0.4f);
            Assert::IsFalse(quat != quat);
        }

        TEST_METHOD(Equality_ZeroQuaternions_ReturnsTrue)
        {
            Quaternion zero1(0.0f, 0.0f, 0.0f, 0.0f);
            Quaternion zero2(0.0f, 0.0f, 0.0f, 0.0f);
            Assert::IsTrue(zero1 == zero2);
        }

        TEST_METHOD(Equality_NegativeValues_WorksCorrectly)
        {
            Quaternion a(-0.1f, -0.2f, -0.3f, -0.4f);
            Quaternion b(-0.1f, -0.2f, -0.3f, -0.4f);
            Assert::IsTrue(a == b);
        }

        TEST_METHOD(Inequality_MixedSigns_ReturnsTrue)
        {
            Quaternion a(0.1f, 0.2f, 0.3f, 0.4f);
            Quaternion b(-0.1f, -0.2f, -0.3f, -0.4f);
            Assert::IsTrue(a != b);
        }

        TEST_METHOD(EdgeCase_Equality_VerySmallDifferences_HandledCorrectly)
        {
            Quaternion a(1e-6f, 1e-6f, 1e-6f, 1e-6f);
            Quaternion b(2e-6f, 2e-6f, 2e-6f, 2e-6f);
            // This should return false as the difference is outside typical tolerance
            Assert::IsFalse(a == b);
        }

        TEST_METHOD(EdgeCase_Equality_VeryLargeValues_WorksCorrectly)
        {
            Quaternion a(1e6f, 1e6f, 1e6f, 1e6f);
            Quaternion b(1e6f, 1e6f, 1e6f, 1e6f);
            Assert::IsTrue(a == b);
        }
    };
}