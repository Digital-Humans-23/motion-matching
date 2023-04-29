#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

// initialize every eigen matrix by zero!
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

// some useful macros
#ifndef INFINITY
#define INFINITY DBL_MAX
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define PI 3.1415926535897932

/**
 * This macro checks to see if the value of x is zero within epsilon:
 * -epsilon<x<epsilon
 */
#define IS_ZERO(x) (fabs(x) < 1e-10)

#define IS_EQUAL(x, y) IS_ZERO(((x) - (y)))

#define RAD(x) (((x)*PI) / 180.0)
#define RADF(x) (((x) * static_cast<float>(PI)) / 180.0f)

#define DEG(x) (((x)*180) / PI)
#define SQR(x) ((x) * (x))

#define SGN(x) (((x) < 0) ? (-1) : (1))

// some typedef
#define DynamicArray std::vector
typedef unsigned int uint;

namespace crl {

#ifdef _DEBUG  // DEBUG
typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> Matrix2x2;
typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> Matrix4x4;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2d;
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> Vector4d;
#else  // RELEASE
typedef Eigen::Matrix2d Matrix2x2;
typedef Eigen::Matrix4d Matrix4x4;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector4d Vector4d;
#endif  // _DEBUG

/* TODO: should we also put these into _DEBUG macro? */
typedef Eigen::AngleAxisd AngleAxisd;
typedef Eigen::VectorXd dVector;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3d Triplet;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Matrix3d Matrix3x3;
typedef Eigen::SparseMatrix<double> SparseMatrix;
typedef Eigen::Triplet<double> MTriplet;
typedef Eigen::Quaternion<double> Quaternion;

inline void resize(SparseMatrix &sm, int rows, int cols) {
    if (sm.rows() != rows || sm.cols() != cols)
        sm.resize(rows, cols);
    sm.setZero();
}

inline void resize(dVector &v, int n) {
    if (v.size() != n)
        v.resize(n);
    v.setZero();
}

inline void resize(Matrix &m, int rows, int cols) {
    if (m.rows() != rows || m.cols() != cols)
        m.resize(rows, cols);
    m.setZero();
}

class P3D {
public:
    double x = 0;
    double y = 0;
    double z = 0;

    explicit P3D(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    P3D() {
        this->x = this->y = this->z = 0;
    }

    P3D(const double *data) {
        this->x = data[0];
        this->y = data[1];
        this->z = data[2];
    }

    P3D operator+(const P3D &p) {
        return P3D(x + p.x, y + p.y, z + p.z);
    }

    P3D operator-(const P3D &p) {
        return P3D(x - p.x, y - p.y, z - p.z);
    }

    P3D &operator+=(const P3D &p) {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }

    P3D &operator-=(const P3D &p) {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return *this;
    }

    P3D operator*(double v) {
        return P3D(x * v, y * v, z * v);
    }

    P3D &operator*=(double v) {
        x *= v;
        y *= v;
        z *= v;
        return *this;
    }

    P3D operator/(double v) {
        return P3D(x / v, y / v, z / v);
    }

    P3D &operator/=(double v) {
        x /= v;
        y /= v;
        z /= v;
        return *this;
    }

    double &operator[](int idx) {
        if (idx == 0)
            return x;
        if (idx == 1)
            return y;
        return z;
    }

    const double &operator[](int idx) const {
        if (idx == 0)
            return x;
        if (idx == 1)
            return y;
        return z;
    }
};

/**
 * explicit conversion to still exploit the fast eigen operations as much as
 * possible
 */
inline P3D getP3D(const Vector3d &v) {
    return P3D(v.data());
}

class V3D : public Vector3d {
public:
    V3D(double x = 0, double y = 0, double z = 0) {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = z;
    }

    /**
     * this constructor will get called by default every time Vector3d
     * operations are involved
     */
    V3D(const Vector3d &v) {
        Vector3d::operator=(v);
    }

    /**
     * vector that points from p1 to p2
     */
    V3D(const P3D &p1, const P3D &p2) {
        (*this)[0] = p2.x - p1.x;
        (*this)[1] = p2.y - p1.y;
        (*this)[2] = p2.z - p1.z;
    }

    /**
     * vector that points from origin to p
     */
    explicit V3D(const P3D &p) {
        (*this)[0] = p.x;
        (*this)[1] = p.y;
        (*this)[2] = p.z;
    }

    V3D &operator=(const Vector3d &v) {
        Vector3d::operator=(v);
        return *this;
    }

    V3D operator+(const V3D &v) const {
        return V3D(Vector3d::operator+(v));
    }

    V3D operator+(const Vector3d &v) const {
        return V3D(Vector3d::operator+(v));
    }

    V3D operator-() const {
        return (V3D)Vector3d::operator-();
    }

    V3D operator-(const V3D &v) const {
        return (V3D)Vector3d::operator-(v);
    }

    V3D operator-(const Vector3d &v) const {
        return (V3D)Vector3d::operator-(v);
    }

    V3D operator*(double val) const {
        return V3D(Vector3d::operator*(val));
    }

    V3D operator/(double val) const {
        return V3D(Vector3d::operator/(val));
    }

    V3D cross(const V3D &other) const {
        return V3D(Vector3d::cross(other));
    }

    double getComponentAlong(const V3D &other) {
        return Vector3d::dot(other);
    };

    V3D unit() const {
        if (this->norm() < 10e-20)
            return V3D(1, 0, 0);
        return *this / this->norm();
    }

    double getComponentAlong(const V3D &other) const {
        return Vector3d::dot(other);
    };

    void setComponentAlong(const V3D &other, double val) {
        double oldVal = getComponentAlong(other);
        *this += other * (val - oldVal);
    };
};

inline V3D operator*(double val, const V3D &v) {
    return v * val;
}

inline P3D operator+(const P3D &p, const Vector3d &v) {
    return P3D(p.x + v[0], p.y + v[1], p.z + v[2]);
}

inline P3D operator-(const P3D &p, const Vector3d &v) {
    return P3D(p.x - v[0], p.y - v[1], p.z - v[2]);
}

class RigidTransformation {
public:
    Quaternion R = Quaternion::Identity();
    P3D T = P3D(0, 0, 0);

public:
    RigidTransformation(const Quaternion &_R = Quaternion::Identity(), const P3D &_T = P3D()) : R(_R), T(_T) {}
    ~RigidTransformation() {}

    P3D transform(const P3D &p) {
        return T + R * V3D(p);
    }

    V3D transform(const V3D &v) {
        return R * v;
    }

    RigidTransformation inverse() {
        RigidTransformation trans;
        trans.R = R.inverse();
        trans.T = P3D() - (trans.R * V3D(P3D(), T));
        return trans;
    }

    RigidTransformation operator*(const RigidTransformation &other) {
        RigidTransformation trans;
        // use rotation matrix based multiplication to avoid singularity.
        trans.R = R * other.R;
        trans.T = T + R * V3D(other.T);
        return trans;
    }

    RigidTransformation &operator*=(const RigidTransformation &other) {
        RigidTransformation trans;
        // use rotation matrix based multiplication to avoid singularity.
        trans.R = R * other.R;
        trans.T = T + R * V3D(other.T);
        *this = trans;
        return *this;
    }
};

}  // namespace crl