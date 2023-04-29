#pragma once

#include "crl-basic/gui/guiMath.h"
#include "crl-basic/gui/mesh.h"
#include "crl-basic/gui/shader.h"

namespace crl {
namespace gui {

class Model {
public:
    // stores all the textures loaded so far, optimization to make sure textures aren't loaded more than once.
    std::vector<Mesh::TextureMap> material_textures;

    std::vector<Mesh> meshes;

    // this is the name of the model, in case it was loaded from a file
    std::string mName;

    // scale about the x, y, and z axes, applied before any other transformations
    mutable V3D scale = V3D(1, 1, 1);
    // the position of the model in world coordinates
    mutable P3D position = P3D(0, 0, 0);
    // the orientation of the model - takes vectors from local coordinates to world coordinates
    mutable Quaternion orientation = Quaternion::Identity();

    bool highlighted = false;
    bool selected = false;

public:
    Model() = default;

    Model(const std::string &path);

    virtual ~Model() = default;

    void draw(const Shader &shader, const V3D &color, float alpha = 1.f, bool showMaterials = true) const;

    glm::mat4 getTransform() const;

    // helper function
    static void calculateFaceNormals(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::MatrixXd &FN);

protected:
    //Loads a model based on the corresponding file extension
    void loadModel(const std::string &path);

    // loads a model with tinyobjloader from file and stores the resulting meshes in the meshes vector
    void loadObjModel(const std::string &path);

    // loads a model with stl_reader from file and stores the resulting meshes in the meshes vector
    void loadStlModel(const std::string &path);

public:
    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, double &t, V3D &n) const;
    bool hitByRay(const P3D &r_o, const V3D &r_v) const;
    bool hitByRay(const P3D &r_o, const V3D &r_v, double &t) const;
    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint) const;
    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, V3D &hitNormal) const;
};

}  // namespace gui
}  // namespace crl