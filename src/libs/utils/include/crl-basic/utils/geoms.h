#pragma once

#include "crl-basic/utils/mathUtils.h"

namespace crl {

class Segment {
public:
    P3D a = P3D(0, 0, 0);
    P3D b = P3D(0, 0, 0);

    Segment() {}

    Segment(const P3D &a_, const P3D &b_) {
        this->origin = a_;
        this->dir = b_;
    }

private:
    P3D origin;
    P3D dir;
};

class Plane {
public:
    // a plane is defined by its normal, and a point that lies on it
    V3D n = V3D(0, 1, 0);
    P3D p = P3D(0, 0, 0);

public:
    Plane(void);
    Plane(const P3D &p, const V3D &n) {
        this->n = n.normalized();
        this->p = p;
    }
    Plane(const P3D &p1, const P3D &p2, const P3D &p3) {
        this->p = p1;
        this->n = (V3D(p1, p2).cross(V3D(p2, p3))).normalized();
    }
    ~Plane(void) {}

    double getSignedDistanceToPoint(const P3D &pt) const {
        return V3D(p, pt).dot(n);
    }

    Plane &operator=(const Plane &other);

    // get the coefficients that define the cartesian equation of the plane: ax
    // + by + cz + d = 0
    void getCartesianEquationCoefficients(double &a, double &b, double &c, double &d) {
        a = n[0];
        b = n[1];
        c = n[2];
        d = -(n[0] * p.x + n[1] * p.y + n[2] * p.z);
    }
};

class Ray {
public:
    P3D origin = P3D(0, 0, 0);
    V3D dir = V3D(0, 0, 1);

    Ray() {}

    Ray(const P3D &o, const V3D &d) {
        this->origin = o;
        this->dir = d;
    }

    // returns the point on the ray with distance 't' from the origin
    P3D getPointAt(double t) const {
        return origin + dir * t;
    }

    // returns the 't' value away from origin where p can be found
    double getRayParameterFor(const P3D &p) const {
        return dir.dot(V3D(origin, p));
    }

    // returns the smallest distance between the line segment and the ray.
    // If ClosestPtOnRay is not nullptr, it will be set to the point on the ray
    // that is closest to the segment
    double getDistanceToSegment(const P3D &p1, const P3D &p2, P3D *closestPtOnRay) const {
        // (c) Dan Sunday, http://geomalgorithms.com/a07-_distance.html
        V3D u = V3D(p1, origin) + dir * 10000;
        V3D v(p1, p2);
        V3D w(p1, origin);

        double a = u.squaredNorm();
        double b = u.dot(v);
        double c = v.squaredNorm();
        double d = u.dot(w);
        double e = v.dot(w);
        double denom = a * c - b * b;

        double sN, sD = denom;
        double tN, tD = denom;

        if (denom <= 1e-10) {
            // lines are almost parallel
            sN = 0;
            sD = 1;
            tN = e;
            tD = c;
        } else {
            sN = (b * e - c * d);
            tN = (a * e - b * d);
            if (sN < 0) {
                // the closest point is behind the ray's starting point
                sN = 0;
                tN = e;
                tD = c;
            }
        }

        if (tN < 0) {
            // the closest point is behind t1
            tN = 0;
            if (-d < 0)
                sN = 0;
            else if (-d > a)
                sN = sD;
            else {
                sN = -d;
                sD = a;
            }
        } else if (tN > tD) {
            // the closest point is behind t2
            tN = tD;
            if ((-d + b) < 0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else {
                sN = -d + b;
                sD = a;
            }
        }

        double sc = (abs(sN) < 1e-10) ? 0 : sN / sD;
        double tc = (abs(tN) < 1e-10) ? 0 : tN / tD;

        if (closestPtOnRay != nullptr)
            *closestPtOnRay = origin + dir * (sc * 10000);

        return (w + u * sc - v * tc).norm();
    }

    // returns the point on the ray that is closest to c.
    // If ClosestPtOnRay is not nullptr, it will be set to the point on the ray
    // that is closest to c
    double getDistanceToPoint(const P3D &p, P3D *closestPtOnRay) const {
        double d = dir.dot(V3D(origin, p));
        P3D res = origin;
        if (d > 0)
            res = getPointAt(d);

        if (closestPtOnRay)
            *closestPtOnRay = res;

        return V3D(p, res).norm();
    }

    // returns the smallest distance between the line segment and the ray.
    // and if ClosestPtOnRay is not NULL, it will be set to the point on the ray
    // that is closest to c
    double getDistanceToPlane(const Plane &plane, P3D *closestPtOnRay) const {
        // check to see if the ray is parallel to the plane...
        if (fabs(dir.dot(plane.n)) < 10e-10) {
            if (closestPtOnRay)
                *closestPtOnRay = origin;
            return fabs(plane.getSignedDistanceToPoint(origin));
        }

        // we know that p = origin + t * direction lies on the plane, which
        // means that dot product between plane p and p, dotted with n is 0...
        V3D tmpV(origin, plane.p);
        double t = tmpV.dot(plane.n) / dir.dot(plane.n);

        // check the solution now...
        assert(IS_ZERO(V3D(plane.p, origin + dir * t).dot(plane.n)));

        if (t < 0) {
            if (closestPtOnRay)
                *closestPtOnRay = origin;
            return fabs(plane.getSignedDistanceToPoint(origin));
        }

        if (closestPtOnRay)
            *closestPtOnRay = origin + dir * t;

        return t;
    }
};

};  // namespace crl