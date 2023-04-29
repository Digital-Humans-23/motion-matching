#include "crl-basic/gui/model.h"

#include "crl-basic/utils/logger.h"
#include "crl-basic/utils/utils.h"

#define TINYOBJLOADER_IMPLEMENTATION  // define this in only *one* .cc
#include <stl_reader.h>
#include <tiny_obj_loader.h>

#include <glm/gtx/intersect.hpp>
#include <unordered_map>

namespace std {
template <>
struct hash<crl::gui::Vertex> {
    size_t operator()(crl::gui::Vertex const &vertex) const {
        return ((hash<glm::vec3>()(vertex.position) ^ (hash<glm::vec3>()(vertex.normal) << 1)) >> 1) ^ (hash<glm::vec2>()(vertex.texCoords) << 1);
    }
};
}  // namespace std

namespace crl {
namespace gui {

// model class member functions
Model::Model(const std::string &path) {
    loadModel(path);
}

void Model::calculateFaceNormals(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::MatrixXd &FN) {
    FN.resize(F.rows(), 3);

    for (int i = 0; i < F.rows(); i++) {
        const Eigen::RowVector3d v1 = V.row(F(i, 1)) - V.row(F(i, 0));
        const Eigen::RowVector3d v2 = V.row(F(i, 2)) - V.row(F(i, 0));
        FN.row(i) = v1.cross(v2).normalized();
    }
}

void Model::draw(const Shader &shader, const V3D &color, float alpha, bool showMaterials) const {
    shader.use();
    shader.setMat4("model", getTransform());
    for (unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].draw(shader, color, alpha, showMaterials);
    }
}

glm::mat4 Model::getTransform() const {
    return getGLMTransform(scale, orientation, position);
}

void calculateVertexNormals(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes, Eigen::MatrixXd &VN) {
    // parse to eigen matrix
    Eigen::MatrixXd V;
    V.resize(attrib.vertices.size() / 3, 3);
    for (int i = 0; i < attrib.vertices.size() / 3; i += 1) {
        V(i, 0) = attrib.vertices[3 * i + 0];
        V(i, 1) = attrib.vertices[3 * i + 1];
        V(i, 2) = attrib.vertices[3 * i + 2];
    }

    VN = Eigen::MatrixXd::Zero(V.rows(), 3);

    for (const auto shape : shapes) {
        Eigen::MatrixXi F;
        F.resize(shape.mesh.num_face_vertices.size(), 3);

        // shapeshape.mesh.num_face_vertices
        for (int i = 0; i < shape.mesh.num_face_vertices.size(); i++) {
            F(i, 0) = shape.mesh.indices[3 * i + 0].vertex_index;
            F(i, 1) = shape.mesh.indices[3 * i + 1].vertex_index;
            F(i, 2) = shape.mesh.indices[3 * i + 2].vertex_index;
        }

        Eigen::MatrixXd FN;
        Model::calculateFaceNormals(V, F, FN);

        // now loop over all faces again and accumulate face normals and then normalize
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                VN.row(F(i, j)) += FN.row(i);
            }
        }
        // normalize each row
        VN.rowwise().normalize();
    }
}

void Model::loadModel(const std::string &path) {
    mName = path;
    if (checkFileExtension(path.c_str(), "obj") || checkFileExtension(path.c_str(), "OBJ"))
        loadObjModel(path);
    else if (checkFileExtension(path.c_str(), "stl") || checkFileExtension(path.c_str(), "STL"))
        loadStlModel(path);
    else
        crl::throwError("Model -> unknown file extension: model could not be loaded!");
}

void Model::loadObjModel(const std::string &path) {
    std::string newPath = path;
    std::replace(newPath.begin(), newPath.end(), '\\', '/');
    std::string directory = newPath.substr(0, newPath.find_last_of('/'));

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    // we shall retriangulate the mesh...
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, newPath.c_str(), directory.c_str(), true);

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        throw std::runtime_error("error loading obj: " + err);
    }

    if (!ret) {
        throw std::runtime_error("error loading obj.");
    }

    // Load materials
    std::vector<Material> material_vector;
    material_textures.resize(materials.size());
    for (uint i = 0; i < materials.size(); ++i) {
        const auto &mat = materials[i];
        if (!mat.diffuse_texname.empty())
            material_textures[i][Mesh::DIFFUSE].push_back({mat.diffuse_texname.c_str(), directory});

        material_vector.push_back(Material{glm::vec3(mat.ambient[0], mat.ambient[1], mat.ambient[2]), glm::vec3(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]),
                                           glm::vec3(mat.specular[0], mat.specular[1], mat.specular[2])});
    }

    Eigen::MatrixXd VN;
    bool no_normals_loaded = (attrib.normals.size() == 0);
    if (no_normals_loaded) {
        // calculate simple normals
        calculateVertexNormals(attrib, shapes, VN);
    }

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Mesh::TextureMap textures;
        Material material;

        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            // To avoid duplicating vertices (as we know each vertex will appear
            // once for each triangle that contains it) we'll use this hash map
            // that lets us know if a vertex has already been seen...
            std::unordered_map<Vertex, uint32_t> uniqueVertices;

            int fv = shapes[s].mesh.num_face_vertices[f];

            // Loop over vertices in the face.
            for (size_t v = 0; v < (size_t)fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                Vertex vertex;

                for (int i = 0; i < 3; i++) {
                    vertex.position[i] = attrib.vertices[3 * idx.vertex_index + i];

                    if (attrib.normals.size() > 3 * idx.normal_index + i)
                        vertex.normal[i] = attrib.normals[3 * idx.normal_index + i];
                }

                if (attrib.texcoords.size() > 0) {
                    tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
                    tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
                    vertex.texCoords = {tx, 1 - ty};
                }

                if (no_normals_loaded) {
                    vertex.normal = glm::vec3(VN(idx.vertex_index, 0), VN(idx.vertex_index, 1), VN(idx.vertex_index, 2));
                }

                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vertex);
                }
                indices.push_back(uniqueVertices[vertex]);
            }
            index_offset += fv;
        }

        int matId = shapes[s].mesh.material_ids[0];
        if (matId >= 0) {
            textures = material_textures[matId];

            if (matId < (int)material_vector.size()) {
                material = material_vector[matId];
                material.isInUse = true;
            }
        }

        meshes.push_back(Mesh(vertices, indices, textures, material));
    }
}

void Model::loadStlModel(const std::string &path) {
    try {
        stl_reader::StlMesh<float, unsigned int> mesh(path);

        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        for (size_t itri = 0; itri < mesh.num_tris(); ++itri) {
            for (size_t icorner = 0; icorner < 3; ++icorner) {
                const float *c = mesh.tri_corner_coords(itri, icorner);
                const float *n = mesh.tri_normal(itri);

                Vertex vertex;
                for (int i = 0; i < 3; i++) {
                    vertex.position[i] = c[i];
                    vertex.normal[i] = n[i];
                }
                vertices.push_back(vertex);
                indices.push_back((unsigned int)(3 * itri + icorner));
            }
        }

        meshes = {Mesh(vertices, indices)};

    } catch (std::exception &e) {
        std::cout << "Model::loadStlModel -> " << e.what() << std::endl;
    }
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, double &t, V3D &n) const {
    using namespace glm;

    glm::vec3 orig = toGLM(r_o);
    glm::vec3 dir = toGLM(r_v);

    auto modelInv = glm::inverse(getTransform());
    vec4 origModelTmp = modelInv * vec4(orig, 1);
    vec3 origModel = vec3(origModelTmp / origModelTmp.w);
    vec3 dirModel = vec3(modelInv * vec4(dir, 0));

    bool hit = false;
    t = HUGE_VALF;
    vec2 bary;

    for (const auto &m : meshes) {
        for (unsigned int i = 0; i < m.indices.size() / 3; ++i) {
            vec3 v0 = m.vertices[m.indices[3 * i + 0]].position;
            vec3 v1 = m.vertices[m.indices[3 * i + 1]].position;
            vec3 v2 = m.vertices[m.indices[3 * i + 2]].position;

            float t_ = 0;
            bool tHit = glm::intersectRayTriangle(origModel, dirModel, v0, v1, v2, bary, t_);

            if (tHit && t_ > 1e-8 && t_ < t) {
                hit = true;
                t = t_;

                // handle the scaling here, otherwise the normal is a bit messed
                // up...
                for (int idx = 0; idx < 3; idx++) {
                    v0[idx] *= scale[idx];
                    v1[idx] *= scale[idx];
                    v2[idx] *= scale[idx];
                }

                hitPoint = toP3D(v0 * (1 - bary.x - bary.y) + v1 * bary.x + v2 * bary.y);
                n = toV3D(v1 - v0).cross(toV3D(v2 - v0)).normalized();
            }
        }
    }

    if (hit) {
        // the point is now in local coordinates, so switch it over to world
        // coords...
        hitPoint = position + V3D(orientation * V3D(P3D(), hitPoint));
        n = orientation * n;
    }
    return hit;
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v) const {
    P3D hitPoint(0, 0, 0);
    V3D n(0, 0, 0);
    double t = 0;

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, double &t) const {
    P3D hitPoint(0, 0, 0);
    V3D n(0, 0, 0);

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint) const {
    double t = 0;
    V3D n(0, 0, 0);

    return hitByRay(r_o, r_v, hitPoint, t, n);
}

bool Model::hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, V3D &hitNormal) const {
    double t = 0;

    return hitByRay(r_o, r_v, hitPoint, t, hitNormal);
}

}  // namespace gui
}  // namespace crl