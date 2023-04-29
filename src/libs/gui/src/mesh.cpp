#include "crl-basic/gui/mesh.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace crl {
namespace gui {

Mesh::Mesh(const std::vector<Vertex> &vertices,       //
           const std::vector<unsigned int> &indices,  //
           const std::map<TextureType, std::vector<Texture>> &textures)
    : vertices(vertices), indices(indices), textures(textures) {}

Mesh::Mesh(const std::vector<Vertex> &vertices,                          //
           const std::vector<unsigned int> &indices,                     //
           const std::map<TextureType, std::vector<Texture>> &textures,  //
           const Material &material)
    : vertices(vertices), indices(indices), textures(textures), material(material) {}

Mesh::Mesh(const std::vector<Vertex> &vertices,  //
           const std::vector<unsigned int> &indices)
    : vertices(vertices), indices(indices) {}

Mesh::~Mesh() {
    auto *ctx = rendering::GetCurrentMeshRenderingContext();
    if (ctx) {
        ctx->removeMeshRenderingBuffer(this);
        // remove textures
        for (auto &it : textures) {
            for (auto &t : it.second)
                ctx->removeTextureRenderingBuffer(&t);
        }
    }
}

void Mesh::draw(const Shader &shader, const V3D &color, float alpha, bool showMaterials) const {
    // update shader
    shader.setFloat("alpha", alpha);

    // check mesh is already loaded
    auto *ctx = rendering::GetCurrentMeshRenderingContext();
    if (!ctx->isMeshRenderingBufferExist(this)) {
        // if mesh buffer does not exist, setup mesh first
        setupMesh();
    }

    // bind texture
    if (showMaterials && textures.find(DIFFUSE) != textures.end()) {
        shader.setBool("use_textures", true);
        // bind appropriate textures
        for (unsigned int i = 0; i < textures.at(DIFFUSE).size(); i++) {
            GLCall(glActiveTexture(GL_TEXTURE0 + i));  // active proper texture unit before binding
            // retrieve texture number (the N in diffuse_textureN)
            const Texture &texture = textures.at(DIFFUSE)[i];
            // now set the sampler to the correct texture unit
            GLCall(glUniform1i(glGetUniformLocation(shader.ID, ("texture_diffuse" + std::to_string(i + 1)).c_str()), i));
            // and finally bind the texture
            auto &t = ctx->getTextureRenderingBuffer(&texture);
            GLCall(glBindTexture(GL_TEXTURE_2D, t.id));
        }
    } else if (showMaterials && material.isInUse) {
        shader.setBool("use_material", true);
        shader.setVec3("material.ambient", material.ambient);
        shader.setVec3("material.diffuse", material.diffuse);
        shader.setVec3("material.specular", material.specular);
        shader.setFloat("material.shininess", material.shininess);
    } else {
        // there are no textures... indicate as much...
        shader.setBool("use_textures", false);
        shader.setBool("use_material", false);
        shader.setVec3("objectColor", toGLM(color));  //set color
    }

    // bind mesh
    auto &b = ctx->getMeshRenderingBuffer(this);
    GLCall(glBindVertexArray(b.VAO));
    GLCall(glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, nullptr));
    GLCall(glBindVertexArray(0));

    // always good practice to set everything back to defaults once configured.
    GLCall(glActiveTexture(GL_TEXTURE0));
}

void Mesh::reinitialize(const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indices) {
    auto *ctx = rendering::GetCurrentMeshRenderingContext();
    if (ctx)
        ctx->removeMeshRenderingBuffer(this);
    this->vertices = vertices;
    this->indices = indices;
    setupMesh();
}

void Mesh::setupMesh() const {
    auto *ctx = rendering::GetCurrentMeshRenderingContext();
    auto &b = ctx->getMeshRenderingBuffer(this);

    // create buffers/arrays
    GLCall(glGenVertexArrays(1, &b.VAO));
    GLCall(glGenBuffers(1, &b.VBO));
    GLCall(glGenBuffers(1, &b.EBO));

    GLCall(glBindVertexArray(b.VAO));
    // load data into vertex buffers
    GLCall(glBindBuffer(GL_ARRAY_BUFFER, b.VBO));
    // A great thing about structs is that their memory layout is sequential
    // for all its items. The effect is that we can simply pass a pointer to
    // the struct and it translates perfectly to a glm::vec3/2 array which
    // again translates to 3/2 floats which translates to a byte array.
    GLCall(glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW));

    GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.EBO));
    GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW));

    // set the vertex attribute pointers
    // vertex Positions
    GLCall(glEnableVertexAttribArray(0));
    GLCall(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)nullptr));
    // vertex normals
    GLCall(glEnableVertexAttribArray(1));
    GLCall(glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, normal)));
    // vertex texture coords
    GLCall(glEnableVertexAttribArray(2));
    GLCall(glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, texCoords)));

    GLCall(glBindVertexArray(0));

    // setup texture
    if (textures.find(DIFFUSE) != textures.end()) {
        // bind appropriate textures
        for (unsigned int i = 0; i < textures.at(DIFFUSE).size(); i++) {
            // retrieve texture number (the N in diffuse_textureN)
            const Texture &texture = textures.at(DIFFUSE)[i];
            if (!ctx->isTextureRenderingBufferExist(&texture)) {
                // if texture buffer does not exist, setup texture first
                ctx->getTextureRenderingBuffer(&texture).id = textureFromFile(texture.path.c_str(), texture.directory);
            }
        }
    }
}

unsigned int Mesh::textureFromFile(const char *path, const std::string &directory) {
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    GLCall(glGenTextures(1, &textureID));

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format = 0;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        GLCall(glBindTexture(GL_TEXTURE_2D, textureID));
        GLCall(glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data));
        GLCall(glGenerateMipmap(GL_TEXTURE_2D));

        GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
        GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));
        GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

        stbi_image_free(data);
    } else {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

namespace rendering {

// global mesh renderer context
MeshRenderingContext *GCRLMeshRender = nullptr;

MeshRenderingContext *GetCurrentMeshRenderingContext() {
    return GCRLMeshRender;
}

// setter for current context
void SetCurrentMeshRenderingContext(MeshRenderingContext *ctx) {
    GCRLMeshRender = ctx;
}

MeshRenderingContext *CreateMeshRederingContext() {
    auto *ctx = new MeshRenderingContext();
    if (GCRLMeshRender == nullptr)
        SetCurrentMeshRenderingContext(ctx);
    return ctx;
}

// if ctx is null, it destroy current context
void DestroyMeshRenderingContext(MeshRenderingContext *ctx) {
    if (ctx == nullptr)
        ctx = GCRLMeshRender;

    // delete context
    if (GCRLMeshRender == ctx)
        SetCurrentMeshRenderingContext(nullptr);
    delete ctx;
}

}  // namespace rendering

}  // namespace gui
}  // namespace crl