#pragma once

#include "crl-basic/gui/guiMath.h"
#include "crl-basic/gui/model.h"
#include "crl-basic/utils/logger.h"

namespace crl {
namespace gui {

void drawSphere(const P3D &p, double r, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

/**
 * p is center of ellipsode
 * orientation is orientation of ellipsoid
 * dims is radius along each axis (in ellipsoid's frame)
 */
void drawEllipsoid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCuboid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawWireFrameCuboid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                         float alpha = 1.0);

void drawCylinder(const P3D &startPosition, const P3D &endPosition, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                  float alpha = 1.0);

void drawCylinder(const P3D &startPosition, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                  float alpha = 1.0);

void drawCone(const P3D &origin, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawArrow3d(const P3D &origin, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCapsule(const P3D &sP, const P3D &eP, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCapsule(const P3D &startPosition, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                 float alpha = 1.0);

void drawCircle(const P3D &center, const V3D &normal, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawRectangle(const P3D &p, const V3D &normal, const double &angle, const Vector2d &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                   float alpha = 1.0);

/**
 * we could go from the vector "from" the long way, or the short way. The Vector up will tell us which one is meant
 */
void drawSector(const P3D &p, const V3D &from, const V3D &to, const V3D &up, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

Model getGroundModel(double s = 100);

class SimpleGroundModel {
public:
    Model ground = getGroundModel(20);
    Model grid1 = Model(CRL_DATA_FOLDER "/meshes/grid1.obj");
    Model grid2 = Model(CRL_DATA_FOLDER "/meshes/grid2.obj");

    void draw(const Shader &shader, const V3D &col = V3D(0.7, 0.7, 0.9));
};

class SizableGroundModel {
public:
    SizableGroundModel(int size);

    void setSize(int size);

    int getSize() const;

    void draw(const Shader &shader, const double &intensity = 1.0, const V3D &groundColor = V3D(0.95, 0.95, 0.95),
              const V3D &gridColor = V3D(0.78431, 0.78431, 0.78431));

private:
    int size;
    Model ground;

public:
    double gridThickness = 0.025;
    bool showGrid = true;
};

namespace rendering {

struct RenderingContext {
    // subcontext
    MeshRenderingContext *mctx = nullptr;

    // reusable primitive models
    Model sphere = Model(CRL_DATA_FOLDER "/meshes/sphere.obj");
    Model cube = Model(CRL_DATA_FOLDER "/meshes/cube.obj");
    Model cubeFrame = Model(CRL_DATA_FOLDER "/meshes/cube_frame.obj");
    Model cylinder = Model(CRL_DATA_FOLDER "/meshes/cylinder.obj");
    Model cone = Model(CRL_DATA_FOLDER "/meshes/cone.obj");
    Model sector = Model(CRL_DATA_FOLDER "/meshes/sector.obj");
};

RenderingContext *CreateContext();

// if ctx is null, it destroy current context
void DestroyContext(RenderingContext *ctx = NULL);

// getter for current context
RenderingContext *GetCurrentContext();

// setter for current context
void SetCurrentContext(RenderingContext *ctx);

}  // namespace rendering
}  // namespace gui
}  // namespace crl