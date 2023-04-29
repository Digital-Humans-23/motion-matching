#pragma once

// possible loss of data in conversion between double and float
#pragma warning(disable : 4224)
// deprecated/unsafe functions such as fopen
#pragma warning(disable : 4996)

// This header file contains useful constants and macros.
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>

#include "crl-basic/utils/mathDefs.h"

namespace crl {

inline void boundToRange(double *v, double min, double max) {
    if (*v < min)
        *v = min;
    if (*v > max)
        *v = max;
}

inline void boundToRange(double &v, double min, double max) {
    if (v < min)
        v = min;
    if (v > max)
        v = max;
}

inline double safeACOS(double val) {
    if (val < -1)
        return PI;
    if (val > 1)
        return 0;
    return acos(val);
}

inline double safeASIN(double val) {
    boundToRange(&val, -1, 1);
    return asin(val);
}

/**
 * Returns a new vector obtained by rotating v. Alpha is specified in radians,
 * and axis is assumed to be a unit vector rotate vector using Rodrigues'
 * rotation formula
 */
inline V3D rotateVec(const V3D &v, double alpha, const V3D &axis) {
    assert(IS_EQUAL(axis.squaredNorm(), 1));

    double cosa = cos(alpha);
    double sina = sin(alpha);

    return v * cosa + axis.cross(v) * sina + axis * (axis.dot(v)) * (1 - cosa);
}

/**
 * this returns the smallest angle between u and v
 */
inline double angleBetween(const V3D &u, const V3D &v) {
    // U.V = |U|*|V|*cos(angle)
    // therefore angle = inverse cos (U.V/(|U|*|V|))
    double result = u.dot(v) / (u.norm() * v.norm());
    return safeACOS(result);
}

/**
 * tells us the rotation angle between u and v, given a rotation direction
 * specified by n
 */
inline double angleBetween(const V3D &u, const V3D &v, const V3D &n) {
    double a = angleBetween(u, v);
    if (u.cross(v).dot(n) < 0)
        a = 2 * PI - a;
    return a;
}

inline double getRandomNumberInRange(double min, double max) {
    double range = max - min;
    int largeIntValue = RAND_MAX;
    int randVal = rand();
    double val = (randVal % largeIntValue) / (double)(largeIntValue - 1);
    return min + val * range;
}

/**
 * Draw a number from a gaussian distribution. To get a number with a certain
 * mean and variance, take r = mean + sigma*getRandomGaussian()
 */
inline double getRandomGaussian() {
    double x1, x2, rquad;
    do {
        x1 = 2.0 * getRandomNumberInRange(0, 1) - 1.0;
        x2 = 2.0 * getRandomNumberInRange(0, 1) - 1.0;
        rquad = x1 * x1 + x2 * x2;
    } while (rquad >= 1 || rquad <= 0);

    double fac = sqrt(-2 * log(rquad) / rquad);

    return fac * x2;
}

/**
 * if v < min, this method returns 0. If v > max, it returns 1. For everything
 * else it returns an interpolated value;
 */
inline double mapTo01Range(double v, double min, double max) {
    double t = v;
    if (fabs(min - max) < 1e-10)
        return 1;
    boundToRange(&t, min, max);
    t = (t - min) / (max - min);
    return t;
}

inline double linearlyInterpolate(double v1, double v2, double t1, double t2, double t) {
    if (fabs(v1 - v2) < 1e-10)
        return v2;
    return (t - t1) / (t2 - t1) * v2 + (t2 - t) / (t2 - t1) * v1;
}

inline int roundToInt(double r) {
    return (int)((r > 0.0) ? (r + 0.5) : (r - 0.5));
}

inline double randNumberIn01Range() {
    return ((double)rand() / ((double)RAND_MAX + 1));
}

inline bool isNaN(double x) {
    return (x != x);
}

inline double getRotationAngle(const Quaternion &q, const V3D &v) {
    int sinSign = SGN(q.vec().dot(v));
    double result = 2 * safeACOS(q.w());
    if (sinSign < 0)
        result = -result;
    if (result > PI)
        result -= 2 * PI;
    if (result < -PI)
        result += 2 * PI;
    return result;
}

inline double getRotationAngle(const Quaternion &q) {
    return getRotationAngle(q, q.vec().normalized());
}

inline Quaternion getRotationQuaternion(double angle, const V3D &axis) {
    return Quaternion(AngleAxisd(angle, axis));
}

template <class MATType>
void print(const char *fName, const MATType &mat) {
    FILE *fp;
    fp = fopen(fName, "w");

    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            fprintf(fp, "%15.15f\t", mat.coeff(i, j));
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

/**
 * checks if the vector x satisfies the linear equation Ax = b. We assume the
 * sparse matrix is symmetric and
 */
inline void checkSymmetricLinearSystemResidual(const SparseMatrix &A, const dVector &x, const dVector &b) {
    // check how well we solved the linear system
    dVector rhs = A.triangularView<Eigen::Lower>() * x + A.transpose().triangularView<Eigen::StrictlyUpper>() * x;

    double residual = (rhs - b).norm();
    if (residual > 1e-8) {
        std::cout << "Linear system residual: " << residual << " " << std::endl;
        print(CRL_DATA_FOLDER "/out/A.m", A);
        print(CRL_DATA_FOLDER "/out/b.m", b);
        print(CRL_DATA_FOLDER "/out/x.m", x);
        print(CRL_DATA_FOLDER "/out/residual.m", rhs);
    }
}

/**
 * computes two vectors (a and b) that are orhtogonal to each other and to v.
 */
inline void getVectorsOrthogonalTo(const V3D &v, V3D &a, V3D &b) {
    // try to choose a vector in the y-z plane, if the z-coordinate is
    // significant enough
    if (v[0] * v[0] / (v.squaredNorm()) < 0.5)  // if the x component of the current vector is not too large,
                                                // relatively speaking, then choose x as the main component for a
                                                // (they won't be aligned perfectly)
        a = V3D(1, 0, 0);
    else  // if the x component of the current vector is large, then the y (or
          // z) components cannot be the only non-zero values, so a is not
          // conlinear with the current vector
        a = V3D(0, 1, 0);
    b = v.cross(a);  // b is orthogonal to both the current vector and a
    a = b.cross(v);  // and a is orthogonal to both a and this...
    a.normalize();
    b.normalize();
}

/**
 * Returns a (uniformly) random unit vector
 */
inline V3D getRandomUnitVector() {
    return V3D(randNumberIn01Range(), randNumberIn01Range(), randNumberIn01Range()).normalized();
}

/**
 * make skew symmetric for v
 */
inline Matrix3x3 getSkewSymmetricMatrix(const V3D &v) {
    Matrix3x3 result;
    result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return result;
}

inline Matrix3x3 getCrossProductMatrix(const V3D &v) {
    return getSkewSymmetricMatrix(v);
}

/**
 * compute a set of euler angle axes given the q - there are many choices here,
 * so one that minimizes the rotation angles is chosen
 */
inline void computeEulerAxesFromQuaternion(const Quaternion &q, V3D &a, V3D &b, V3D &c) {
    // assume q is a rotation transforming vectors from coordinate frame A to B
    // (i.e. B_q_A). We want a sequence of Euler rotations that take you from B
    // to A (i.e. B_q_A = rot(c) * rot(b) * rot(a). There are many options, so
    // just choose one of them...
    a = q.vec().normalized();
    // then b and c should be orthogonal to a - find a rotation between a and
    // the x-axis, and then apply the same rotation to the y and z axes to get b
    // and c...
    V3D rotAxis = a.cross(V3D(1, 0, 0)).normalized();
    double rotAngle = angleBetween(a, V3D(1, 0, 0));
    if (rotAxis.norm() < 0.000001) {
        rotAxis = a = V3D(1, 0, 0);
        rotAngle = 0;
    }

    //	V3D testV = V3D(1,0,0).rotate(-rotAngle, rotAxis);
    assert((rotateVec(V3D(1, 0, 0), -rotAngle, rotAxis) - a).norm() < 0.000001);

    a = rotateVec(V3D(1, 0, 0), -rotAngle, rotAxis);
    b = rotateVec(V3D(0, 1, 0), -rotAngle, rotAxis);
    c = rotateVec(V3D(0, 0, 1), -rotAngle, rotAxis);
}

/**
 * decompose the quaternion q as: q = R(c, gamma) * R(b, beta) * R(a, alpha).
 * Unknowns are: alpha, beta, gamma
 */
inline void computeEulerAnglesFromQuaternion(const Quaternion &q, const V3D &a, const V3D &b, const V3D &c, double &alpha, double &beta, double &gamma) {
    // the idea here is that the a axis only gets rotated about b and c, which
    // are assumed to be orthogonal to each other. Based on this info, we can
    // first compute the angles beta and gamma
    assert(IS_ZERO(a.dot(b)) && IS_ZERO(b.dot(c)));
    assert(IS_ZERO(a.norm() - 1) && IS_ZERO(b.norm() - 1) && IS_ZERO(c.norm() - 1));

    V3D aRot = q * a;

    if (IS_ZERO(a.dot(c))) {
        bool circular = a.cross(b).dot(c) > 0;
        // the three axes form an orthonormal basis (i.e. Tait-Bryan)...
        // singularity around beta = -PI/2 or PI/2
        if (circular) {
            beta = -safeASIN(aRot.dot(c));
            gamma = atan2(aRot.dot(b), aRot.dot(a));
        } else {
            beta = safeASIN(aRot.dot(c));
            gamma = atan2(-aRot.dot(b), aRot.dot(a));
        }
    } else if (IS_ZERO(a.dot(c) - 1)) {
        // these are "proper" euler axes, where the first and the last one are
        // the same... singularity around beta = 0 or PI
        V3D lastAxis = a.cross(b);
        beta = safeACOS(aRot.dot(a));
        gamma = atan2(aRot.dot(b), -aRot.dot(lastAxis));
    } else {
        // dunno what this is.... freak out...
        alpha = beta = gamma = 0;
        assert(false);
        return;
    }

    Quaternion qLeft = getRotationQuaternion(-beta, b) * getRotationQuaternion(-gamma, c) * q;
    alpha = getRotationAngle(qLeft, a);

    //	Quaternion residual = (getRotationQuaternion(gamma, c) *
    // getRotationQuaternion(beta, b) * getRotationQuaternion(alpha, a) *
    // q.getComplexConjugate()); 	printf("residual: %lf (%lf %lf %lf %lf)
    // --> %lf %lf %lf\n", residual.v.length(), residual.s, residual.v[0],
    // residual.v[1], residual.v[2], alpha, beta, gamma);
    assert(IS_ZERO((getRotationQuaternion(gamma, c) * getRotationQuaternion(beta, b) * getRotationQuaternion(alpha, a) * q.inverse()).vec().norm() / 10e5));
}

inline Matrix3x3 getRotationMatrixFromEulerAngles(const V3D &eulerAngles, const V3D &axis_1, const V3D &axis_2, const V3D &axis_3) {
    Quaternion q0 =
        getRotationQuaternion(eulerAngles[0], axis_1) * getRotationQuaternion(eulerAngles[1], axis_2) * getRotationQuaternion(eulerAngles[2], axis_3);

    return q0.toRotationMatrix();
}

inline bool sameRotation(const Quaternion &q1, const Quaternion &q2) {
    // we know that q and -q represent the same rotation, so take that into
    // account...
    if (q1.isApprox(q2, 1e-7))
        return true;
    Quaternion qTmp = q2;
    qTmp.w() *= -1;
    qTmp.vec() *= -1;
    return q1.isApprox(qTmp, 1e-7);
}

// TODO: want to write only upper (or lower?) values for symmetric matrices
template <class MATType>
void writeSparseMatrixDenseBlock(SparseMatrix &hes, int startX, int startY, const MATType &block, bool writeOnlyLowerDiagonalValues = false) {
    for (int i = 0; i < block.rows(); i++)
        for (int j = 0; j < block.cols(); j++)
            if (startX + i >= startY + j || !writeOnlyLowerDiagonalValues)
                hes.coeffRef(startX + i, startY + j) = block(i, j);
}

template <class MATType>
void writeSparseMatrixDenseBlockAdd(SparseMatrix &hes, int startX, int startY, const MATType &block, bool writeOnlyLowerDiagonalValues = false) {
    for (int i = 0; i < block.rows(); i++)
        for (int j = 0; j < block.cols(); j++)
            if (startX + i >= startY + j || !writeOnlyLowerDiagonalValues)
                hes.coeffRef(startX + i, startY + j) += block(i, j);
}

template <class MATType>
void addSparseMatrixDenseBlockToTriplet(std::vector<MTriplet> &triplets, int startX, int startY, const MATType &block,
                                        bool writeOnlyLowerDiagonalValues = false) {
    for (int i = 0; i < block.rows(); i++)
        for (int j = 0; j < block.cols(); j++)
            if (startX + i >= startY + j || !writeOnlyLowerDiagonalValues)
                triplets.push_back(MTriplet(startX + i, startY + j, block(i, j)));
}

template <class MATType>
void addSparseMatrixDenseBlockToTripletAtIndex(std::vector<MTriplet> &triplets, int &pos_idx_io, int startX, int startY, const MATType &block,
                                               bool writeOnlyLowerDiagonalValues = false) {
    // int i_entry = 0;
    for (int i = 0; i < block.rows(); i++)
        for (int j = 0; j < block.cols(); j++)
            if (startX + i >= startY + j || !writeOnlyLowerDiagonalValues) {
                triplets[pos_idx_io++] = MTriplet(startX + i, startY + j, block(i, j));
                //++i_entry;
            }
    // std::cout << i_entry;
}

// if the element at (row, col) is above the diagonal, it is skipped
inline void addMTripletToList_ignoreUpperElements(std::vector<MTriplet> &triplets, int row, int col, double val) {
    if (row >= col)
        triplets.push_back(MTriplet(row, col, val));
}

// if the element at (row, col) is above the diagonal, it is reflected on the
// lower diagonal
inline void addMTripletToList_reflectUpperElements(std::vector<MTriplet> &triplets, int row, int col, double val) {
    if (IS_ZERO(val))
        return;
    if (row >= col)
        triplets.push_back(MTriplet(row, col, val));
    else
        triplets.push_back(MTriplet(col, row, val));
}

// the element at (row, col) is mirrored, so it will be written symmetrically
// above and below the diagonal
inline void addMTripletToList_mirror(std::vector<MTriplet> &triplets, int row, int col, double val) {
    if (row == col) {
        triplets.push_back(MTriplet(row, col, val));
    } else {
        triplets.push_back(MTriplet(row, col, val));
        triplets.push_back(MTriplet(col, row, val));
    }
}

// write out the element as it comes
inline void addMTripletToList(std::vector<MTriplet> &triplets, int row, int col, double val) {
    triplets.push_back(MTriplet(row, col, val));
}

#define ADD_HES_ELEMENT(list, i, j, v) addMTripletToList_reflectUpperElements(list, i, j, (v))

// computes two vectors (a and b) that are orhtogonal to the vector vec.
inline void getOrthogonalVectors(const V3D &vec, V3D &a, V3D &b) {
    // try to choose a vector in the y-z plane, if the z-coordinate is
    // significant enough
    if (vec[0] * vec[0] / (vec.squaredNorm()) < 0.5)  // if the x component of the current vector is not too large,
                                                      // relatively speaking, then choose x as the main component for a
                                                      // (they won't be aligned perfectly)
        a = V3D(1, 0, 0);
    else  // if the x component of the current vector is large, then the y (or
          // z) components cannot be the only non-zero values, so a is not
          // conlinear with the current vector
        a = V3D(0, 1, 0);
    b = vec.cross(a);  // b is orthogonal to both the current vector and a
    a = b.cross(vec);  // and a is orthogonal to both a and this...
    a.normalize();
    b.normalize();
}

}  // namespace crl