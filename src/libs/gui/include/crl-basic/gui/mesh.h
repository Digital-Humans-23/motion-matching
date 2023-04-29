#pragma once

#include "crl-basic/gui/guiMath.h"
#include "crl-basic/gui/shader.h"

// possible loss of data in conversion between double and float
#pragma warning(disable : 4244)
// deprecated/unsafe functions such as fopen
#pragma warning(disable : 4996)

#define GLM_ENABLE_EXPERIMENTAL
#include <map>
#include <vector>

#include "glm/gtx/hash.hpp"

namespace crl {
namespace gui {

struct Vertex {
    glm::vec3 position = glm::vec3(0, 0, 0);
    glm::vec3 normal = glm::vec3(0, 0, 0);
    glm::vec2 texCoords = glm::vec2(0, 0);

    bool operator==(const Vertex &other) const {
        return position == other.position && normal == other.normal && texCoords == other.texCoords;
    }
};

struct Texture {
    std::string path;
    std::string directory;
};

struct Material {
    glm::vec3 ambient = glm::vec3(0.3f);
    glm::vec3 diffuse = glm::vec3(0.8f);
    glm::vec3 specular = glm::vec3(0.5f);
    float shininess = 32.f;
    bool isInUse = false;
};

class Mesh {
    static unsigned int textureFromFile(const char *path, const std::string &directory);

public:
    enum TextureType { DIFFUSE, SPECULAR, NORMAL, AMBIENT };
    typedef std::map<TextureType, std::vector<Texture>> TextureMap;

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    TextureMap textures;
    Material material;

public:
    Mesh(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices, const std::map<TextureType, std::vector<Texture>> &textures);
    Mesh(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices, const std::map<TextureType, std::vector<Texture>> &textures,
         const Material &material);
    Mesh(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices);

    ~Mesh();

    //Render the mesh
    void draw(const Shader &shader, const V3D &color, float alpha, bool showMaterials) const;

    // initializes all the buffer objects/arrays
    void reinitialize(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices);
    void setupMesh() const;
};

namespace rendering {

struct MeshRenderingBuffer {
    unsigned int VAO, VBO, EBO;
};

struct TextureBuffer {
    unsigned int id;
};

// this bit of code imitates the way IMGUI handles context
struct MeshRenderingContext {
    std::map<Mesh const *, MeshRenderingBuffer> buffer;
    std::map<Texture const *, TextureBuffer> texture;

    MeshRenderingContext() = default;

    ~MeshRenderingContext() {
        // delete all buffer
        for (auto it = buffer.begin(); it != buffer.end(); it++) {
            GLCall(glDeleteVertexArrays(1, &it->second.VAO));
            GLCall(glDeleteBuffers(1, &it->second.VBO));
            GLCall(glDeleteBuffers(1, &it->second.EBO));
        }
        for (auto it = texture.begin(); it != texture.end(); it++) {
            GLCall(glDeleteTextures(1, &it->second.id));
        }
    }

    MeshRenderingBuffer &getMeshRenderingBuffer(Mesh const *m) {
        return buffer[m];
    }

    TextureBuffer &getTextureRenderingBuffer(Texture const *t) {
        return texture[t];
    }

    bool removeMeshRenderingBuffer(Mesh const *m) {
        if (!isMeshRenderingBufferExist(m))
            // don't need to erase entry from ctx
            return false;

        auto &b = getMeshRenderingBuffer(m);
        GLCall(glDeleteVertexArrays(1, &b.VAO));
        GLCall(glDeleteBuffers(1, &b.VBO));
        GLCall(glDeleteBuffers(1, &b.EBO));
        buffer.erase(m);
        return true;
    }

    bool removeTextureRenderingBuffer(Texture const *t) {
        if (!isTextureRenderingBufferExist(t))
            // don't need to erase entry from ctx
            return false;

        auto &b = getTextureRenderingBuffer(t);
        GLCall(glDeleteTextures(1, &b.id));
        texture.erase(t);
        return true;
    }

    bool isMeshRenderingBufferExist(Mesh const *m) const {
        return buffer.find(m) != buffer.end();
    }

    bool isTextureRenderingBufferExist(Texture const *t) {
        return texture.find(t) != texture.end();
    }
};

MeshRenderingContext *CreateMeshRederingContext();

// if ctx is null, it destroy current context
void DestroyMeshRenderingContext(MeshRenderingContext *ctx = nullptr);

// getter for current context
MeshRenderingContext *GetCurrentMeshRenderingContext();

// setter for current context
void SetCurrentMeshRenderingContext(MeshRenderingContext *ctx);

}  // namespace rendering
}  // namespace gui
}  // namespace crl