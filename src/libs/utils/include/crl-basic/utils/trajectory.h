#pragma once

#include <limits>

#include "crl-basic/utils/mathDefs.h"
#include "crl-basic/utils/mathUtils.h"

namespace crl {

/**
 * This class is used to represent generic trajectories. The class parameter T
 * can be anything that provides basic operation such as addition and
 * subtraction. We'll define a trajectory that can be parameterized by a 1d
 * parameter (t). Based on a set of knots ( tuples <t, T>), we can evaluate the
 * trajectory at any t through interpolation. Outside the range of the data, the
 * closest stored value is returned.
 */
template <class T>
class GenericTrajectory {
protected:
    // A caching variable to optimize searching for knots
    mutable int lastIndex;

protected:
    DynamicArray<double> tValues;
    DynamicArray<T> values;

    /**
     * This method returns the index of the first knot whose value is larger
     * than the parameter value t. If no such index exists (i.e. t is larger
     * than any of the values stored), then values.size() is returned.
     *
     * TODO: might want to use a binary search here... or something like
     * http://www.cplusplus.com/reference/algorithm/lower_bound/
     */
    inline int getFirstLargerIndex(double t) const {
        int size = (int)tValues.size();
        if (size == 0)
            return 0;

        if (t < tValues[(lastIndex + size - 1) % size])
            lastIndex = 0;
        for (int i = 0; i < size; i++) {
            int index = (i + lastIndex) % size;
            if (t < tValues[index]) {
                lastIndex = index;
                return index;
            }
        }
        return size;
    }

public:
    GenericTrajectory(void) {
        lastIndex = 0;
    }

    GenericTrajectory(const GenericTrajectory<T> &other) {
        lastIndex = 0;
        copy(other);
    }

    virtual ~GenericTrajectory(void) {
        clear();
    }

    /**
     * Call this function if the number of knots is known. This will reduce overhead of push_back() function.
     */
    inline void reserve(int N) {
        tValues.reserve(N);
        values.reserve(N);
    }

    /**
     * This method returns a piece-wise constant value of the trajectory at time
     * t (step function)
     */
    inline T evaluate_piecewise_constant(double t, double point = 0.5) const {
        T res;
        evaluate_piecewise_constant(t, res, point);
        return res;
    }

    /**
     * This method returns a piece-wise constant value of the trajectory at time
     * t (step function)
     */
    inline void evaluate_piecewise_constant(double t, T &res, double point = 0.5) const {
        int size = (int)tValues.size();
        if (t <= tValues[0]) {
            res = values[0];
            return;
        }
        if (t >= tValues[size - 1]) {
            res = values[size - 1];
            return;
        }
        int index = getFirstLargerIndex(t);

        // now figure out where t falls in the correct interval
        t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);
        if (t < point)
            res = values[index - 1];
        else
            res = values[index];
    }

    /**
     * This method performs linear interpolation to evaluate the trajectory at
     * the point t
     */
    inline T evaluate_linear(double t) const {
        T res;
        evaluate_linear(t, res);
        return res;
    }

    /**
     * This method performs linear interpolation to evaluate the trajectory at
     * the point t
     */
    inline void evaluate_linear(double t, T &res) const {
        int size = (int)tValues.size();
        if (t <= tValues[0]) {
            res = values[0];
            return;
        }
        if (t >= tValues[size - 1]) {
            res = values[size - 1];
            return;
        }
        int index = getFirstLargerIndex(t);

        // now linearly interpolate between index-1 and index
        t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);
        _interp(values[index - 1], values[index], 1 - t, t, res);
    }

    T getSlopeEstimateAtKnot(int index, bool equalEndpointSlopes = false) const {
        if (getKnotCount() < 2)
            return T();

        if (index == 0 || index == getKnotCount() - 1) {
            T startSlope = static_cast<T>((values[1] - values[0]) / (tValues[1] - tValues[0]));
            T endSlope =
                static_cast<T>((values[getKnotCount() - 1] - values[getKnotCount() - 2]) / (tValues[getKnotCount() - 1] - tValues[getKnotCount() - 2]));

            if (equalEndpointSlopes)
                return (startSlope + endSlope) / 2.0;

            if (index == 0)
                return startSlope;
            return endSlope;
        }

        T slopeBefore = static_cast<T>((values[index] - values[index - 1]) / (tValues[index] - tValues[index - 1]));
        T slopeAfter = static_cast<T>((values[index + 1] - values[index]) / (tValues[index + 1] - tValues[index]));

        return (slopeBefore + slopeAfter) / 2.0;
    }

    /**
     * For now, do not consider arc length...
     */
    double length() const {
        int size = tValues.size();
        double t1 = tValues[size - 1];
        double t0 = tValues[0];

        double numSteps = 10;
        double dt = (t1 - t0) / (numSteps - 1.0);
        double t = t0;
        double l = 0;
        for (int i = 0; i < numSteps; ++i) {
            l += evaluate_catmull_rom(t);
            t += dt;
        }
        return l;
    }

    /**
     * Evaluate using catmull rom interpolation
     */
    T evaluate_catmull_rom(double t, bool equalEndpointSlopes = false) const {
        int size = (int)tValues.size();
        if (tValues.size() == 0)
            return T(0);
        if (t <= tValues[0])
            return values[0];
        if (t >= tValues[size - 1])
            return values[size - 1];
        int index = getFirstLargerIndex(t);

        // now that we found the interval, get a value that indicates how far we
        // are along it
        t = (t - tValues[index - 1]) / (tValues[index] - tValues[index - 1]);

        // approximate the derivatives at the two ends

        T p1 = values[index - 1];
        T p2 = values[index];

        T m1 = getSlopeEstimateAtKnot(index - 1, equalEndpointSlopes) * (tValues[index] - tValues[index - 1]);
        T m2 = getSlopeEstimateAtKnot(index, equalEndpointSlopes) * (tValues[index] - tValues[index - 1]);

        double t2, t3;
        t2 = t * t;
        t3 = t2 * t;

        // and now perform the interpolation using the four hermite basis
        // functions from wikipedia
        return p1 * (2 * t3 - 3 * t2 + 1) + m1 * (t3 - 2 * t2 + t) + p2 * (-2 * t3 + 3 * t2) + m2 * (t3 - t2);
    }

    /**
     * Returns the value of the ith knot. It is assumed that i is within the
     * correct range.
     */
    T getKnotValue(int i) const {
        return values[i];
    }

    /**
     * Returns the position of the ith knot. It is assumed that i is within the
     * correct range.
     */
    double getKnotPosition(int i) const {
        return tValues[i];
    }

    /**
     * Sets the value of the ith knot to val. It is assumed that i is within the
     * correct range.
     */
    void setKnotValue(int i, const T &val) {
        values[i] = val;
    }

    /**
     * Sets the position of the ith knot to pos. It is assumed that i is within
     * the correct range.
     */
    void setKnotPosition(int i, double pos) {
        if (i - 1 >= 0 && tValues[i - 1] >= pos)
            return;
        if ((uint)(i + 1) < tValues.size() - 1 && tValues[i + 1] <= pos)
            return;
        tValues[i] = pos;
    }

    /**
     * Return the smallest tValue or infinity if none
     */
    double getMinPosition() const {
        if (tValues.empty())
            return std::numeric_limits<double>::infinity();
        return tValues.front();
    }

    /**
     * Return the largest tValue or -infinity if none
     */
    double getMaxPosition() const {
        if (tValues.empty())
            return -std::numeric_limits<double>::infinity();
        return tValues.back();
    }

    /**
     * Returns the number of knots in this trajectory
     */
    int getKnotCount() const {
        return (int)tValues.size();
    }

    /**
     * This method is used to insert a new knot in the current trajectory
     */
    void addKnot(double t, T val) {
        // first we need to know where to insert it, based on the t-values
        int index = getFirstLargerIndex(t);

        tValues.insert(tValues.begin() + index, t);
        values.insert(values.begin() + index, val);
    }

    /**
     * This method is used to remove a knot from the current trajectory.
     * It is assumed that i is within the correct range.
     */
    void removeKnot(int i) {
        tValues.erase(tValues.begin() + i);
        values.erase(values.begin() + i);
    }

    /**
     * This method removes everything from the trajectory.
     */
    void clear() {
        tValues.clear();
        values.clear();
    }

    void copy(const GenericTrajectory<T> &other) {
        tValues.clear();
        values.clear();
        int size = other.getKnotCount();

        tValues.reserve(size);
        values.reserve(size);
        for (int i = 0; i < size; ++i) {
            tValues.push_back(other.tValues[i]);
            values.push_back(other.values[i]);
        }
    }

private:
    inline static void _interp(double p1, double p2, double t1, double t2, double &res) {
        res = p1 * t1 + p2 * t2;
    }

    inline static void _interp(const V3D &v1, const V3D &v2, double t1, double t2, V3D &res) {
        res[0] = v1[0] * t1 + v2[0] * t2;
        res[1] = v1[1] * t1 + v2[1] * t2;
        res[2] = v1[2] * t1 + v2[2] * t2;
    }

    inline static void _interp(const P3D &p1, const P3D &p2, double t1, double t2, P3D &res) {
        res[0] = p1[0] * t1 + p2[0] * t2;
        res[1] = p1[1] * t1 + p2[1] * t2;
        res[2] = p1[2] * t1 + p2[2] * t2;
    }

    inline static void _interp(const Eigen::Matrix<double, 6, 1> &p1, const Eigen::Matrix<double, 6, 1> &p2, double t1, double t2,
                               Eigen::Matrix<double, 6, 1> &res) {
        res = p1 * t1 + p2 * t2;
    }

    inline static void _interp(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2, double t1, double t2, Eigen::VectorXd &res) {
        res = p1 * t1 + p2 * t2;
    }
};

typedef GenericTrajectory<double> Trajectory1D;
typedef GenericTrajectory<V3D> Trajectory3D;
typedef GenericTrajectory<Eigen::Matrix<double, 6, 1>> Trajectory6D;
typedef GenericTrajectory<Eigen::VectorXd> TrajectoryXD;

}  // namespace crl
