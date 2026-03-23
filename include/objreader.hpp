

#pragma once

#include <GL/glew.h> // GLuint

#include "./tinym.hpp"

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <regex>
#include <string>
#include <unistd.h>

struct ObjData
{
    struct MtlData
    {
        std::string materialName;

        tinym::vec3 ambientColor;
        tinym::vec3 diffuseColor;
        tinym::vec3 specularColor;

        MtlData(const std::string mName) : materialName(mName) {}

        friend std::ostream &operator<<(std::ostream &os, const MtlData &mtl)
        {
            os << "Material Name: " << mtl.materialName << std::endl;
            os << "Ambient: " << mtl.ambientColor.r << " " << mtl.ambientColor.g << " " << mtl.ambientColor.b << std::endl;
            os << "Diffuse: " << mtl.diffuseColor.r << " " << mtl.diffuseColor.g << " " << mtl.diffuseColor.b << std::endl;
            os << "Specular: " << mtl.specularColor.r << " " << mtl.specularColor.g << " " << mtl.specularColor.b;
            return os;
        }
    };
    
    std::string prefix = "";
    std::string fileName = "";
    std::string materialFile = "";
    std::string material = "";
    GLuint nVertices = 0;
    GLuint nElements3 = 0;
    GLuint nElements4 = 0;
    GLuint nNormals = 0;
    GLuint nSyncedNormals = 0;
    GLuint nTextures = 0;
    tinym::vec3 maxPos;
    tinym::vec3 minPos;
    tinym::vec3 center;
    tinym::vec3 scale;

    std::vector<tinym::vec3> vertices;
    std::vector<tinym::vec3> texCoords;
    std::vector<tinym::vec3> normals;
    std::vector<tinym::vec3> syncedNormals;
    std::vector<tinym::vec3ui> elements3;
    std::vector<tinym::vec4ui> elements4;

    std::vector<MtlData> materialData;
    
    GLuint vao = 0;
    GLuint vertexBuffer, syncedNormalBuffer, element3Buffer, texCoordBuffer;
    bool isOk = false;
    
    tinym::mat4 modelMat;

    void setPrefix(const std::string &prefixName) {
        if(prefixName.length() == 0) {
            this->prefix = "";
        }
        char *dir = getcwd(NULL, 0);
        std::cout << "Directory: " << dir << std::endl;
        this->prefix = prefixName + '/';
    }
    
    void loadMtl(const std::string &mtlFileName) {
        std::fstream file(prefix + mtlFileName);
        if (!file.is_open()) {
            std::cerr << "No .mtl file" << std::endl;
            return;
        }
        std::cout << "Read " << mtlFileName << std::endl;

        std::string type;
        while (!file.eof()) {
            file >> type;
            if (file.eof())
                break;
            else if (type == "newmtl") {
                std::string mName;
                file >> mName;
                this->materialData.push_back(MtlData(mName));
            } else if (type == "Ka") {
                float r, g, b;
                file >> r >> g >> b;
                this->materialData.back().ambientColor = {r, g, b};
            } else if (type == "Kd") {
                float r, g, b;
                file >> r >> g >> b;
                this->materialData.back().diffuseColor = {r, g, b};
            } else if (type == "Ks") {
                float r, g, b;
                file >> r >> g >> b;
                this->materialData.back().specularColor = {r, g, b};
            }
        }

        std::cout << "Material Count: " << this->materialData.size() << std::endl;
        for (auto m : this->materialData) {
            std::cout << m << std::endl;
        }
    }
    
    void loadObject(const std::string &objFileName) {
        isOk = false;
        maxPos = tinym::vec3(-987654321.f);
        minPos = tinym::vec3( 987654321.f);
        fileName = objFileName;

        std::fstream file(prefix + objFileName);
        if (!file.is_open()) {
            std::cerr << "No .obj file" << std::endl;
            return;
        }
        std::cout << "Read " << prefix + objFileName << std::endl;

        std::vector<std::string> faces;

        std::string type;
        while (!file.eof()) {
            file >> type;
            if (file.eof())
                break;
            if (type == "mtllib") {
                file >> this->materialFile;
                this->loadMtl(this->materialFile);
            } else if (type == "usemtl") {
                file >> this->material;
            } else if (type == "o" || type == "g") {
                std::string dummy;
                if (!std::getline(file, dummy)) {
                    std::cerr << "Group/Object line skipping failed!" << std::endl;
                }
            } else if (type == "v") {
                float x, y, z;
                file >> x >> y >> z;
                this->vertices.push_back({x, y, z});
                
                if(maxPos.x < x) maxPos.x = x;
                else if(minPos.x > x) minPos.x = x;
                if(maxPos.y < y) maxPos.y = y;
                else if(minPos.y > y) minPos.y = y;
                if(maxPos.z < z) maxPos.z = z;
                else if(minPos.z > z) minPos.z = z;
            } else if (type == "vt") {
                float tx, ty;
                file >> tx >> ty;
                this->texCoords.push_back({tx, ty, 0});
            } else if (type == "vn") {
                float nx, ny, nz;
                file >> nx >> ny >> nz;
                this->normals.push_back({nx, ny, nz});
            } else if (type == "f") {
                std::string f;
                std::getline(file, f);

                faces.push_back(f);
            } else if (type == "l") {
                // not in this case
                if (!file.ignore(std::numeric_limits<std::streamsize>::max(),
                                 file.widen('\n'))) {
                    std::cerr << "Line polygon skip!" << std::endl;
                }
            } else {
                if (!file.ignore(std::numeric_limits<std::streamsize>::max(),
                                 file.widen('\n'))) {
                    std::cerr << "Weird situation! input " << type
                              << " is not supported." << std::endl;
                    return;
                }
            }
            // std::cout << "Processing type " << type << std::endl;
        }

        this->nVertices = (int)this->vertices.size();

        std::vector<std::vector<tinym::vec3>> sNormals(this->nVertices);

        bool hasNormal = false;
        for (auto f : faces) {
            // case by case?
            std::vector<GLuint> elem;

            std::regex re("-?\\d+(/-?\\d+|/)*");
            auto start = std::sregex_iterator(f.begin(), f.end(), re);
            auto end = std::sregex_iterator();
            while (start != end) {
                std::string str = start->str();
                int slashCount = 0;
                for(auto& c : str) if(c == '/') slashCount++;
                if(slashCount == 2) { // v/t/n or v//n
                    GLint vertex = std::stoi(str.substr(0, str.find('/'))) - 1;
                    if(vertex < 0) vertex += vertices.size(); // -1 to be the last index
                    elem.push_back(vertex);

                    GLint normal = std::stoi(str.substr(str.find_last_of('/') + 1)) - 1;
                    if(normal < 0) normal += normals.size();
                    sNormals[vertex].push_back(this->normals[normal]);
                    hasNormal = true;
                }
                else if(slashCount == 1) { // v/t
                    GLint vertex = std::stoi(str.substr(0, str.find('/'))) - 1;
                    if(vertex < 0) vertex += vertices.size(); // -1 to be the last index
                    elem.push_back(vertex);
                } else if(slashCount == 0) { // no slash, only vertices
                    GLint vertex = std::stoi(str) - 1;
                    if(vertex < 0) vertex += vertices.size();
                    elem.push_back(vertex);
                }
                start++;
            }

            if (elem.size() == 4) {
                //this->elements4.push_back({elem[0], elem[1], elem[2], elem[3]});
                this->elements3.push_back({elem[0], elem[1], elem[2]});
                this->elements3.push_back({elem[0], elem[2], elem[3]});
            } else if (elem.size() == 3)
                this->elements3.push_back({elem[0], elem[1], elem[2]});
            else {
                std::cerr << "Weird situation! f elements size is not 3 or 4."
                          << std::endl;
                return;
            }
        }
        
        if(!hasNormal) {
            for (auto& e : elements3) {
                tinym::vec3 p0 = vertices[e.x];
                tinym::vec3 p1 = vertices[e.y];
                tinym::vec3 p2 = vertices[e.z];
                tinym::vec3 n = tinym::normalize(tinym::cross(p1-p0, p2-p0));
                sNormals[e.x].push_back(n);
                sNormals[e.y].push_back(n);
                sNormals[e.z].push_back(n);
            }
        }
        
        for (auto sn : sNormals) {
            tinym::vec3 sum(0);
            for (auto n : sn)
                sum += n;
            sum /= sn.size();
            this->syncedNormals.push_back(sum);
        }
    
        this->nElements3 = (int)this->elements3.size();
        this->nElements4 = (int)this->elements4.size();
        this->nNormals = (int)this->normals.size();
        this->nSyncedNormals = (int)this->syncedNormals.size();
        this->nTextures = (int)this->texCoords.size();
        center = (maxPos + minPos) * 0.5f;
        scale = maxPos - minPos;


        std::cout << "nVertices: " << this->nVertices << std::endl;
        std::cout << "nElements3: " << this->nElements3 << std::endl;
        std::cout << "nElements4: " << this->nElements4 << std::endl;
        std::cout << "nNormals: " << this->nNormals << std::endl;
        std::cout << "nSyncedNormals: " << this->nSyncedNormals << std::endl;
        std::cout << "nTextures: " << this->nTextures << std::endl;
        
        std::cout << "maxPos: " << maxPos.x << ", " << maxPos.y << ", " << maxPos.z << std::endl;
        std::cout << "minPos: " << minPos.x << ", " << minPos.y << ", " << minPos.z << std::endl;
        std::cout << "center: " << center.x << ", " << center.y << ", " << center.z << std::endl;
        std::cout << "scale: " << scale.x << ", " << scale.y << ", " << scale.z << std::endl;
        
        
        file.close();
        
        std::cout << "--- Wavefront Object Loaded ---" << std::endl;

        return;
    }


    void loadObject(const std::string &prefixName,
                    const std::string &objFileName) {
        this->setPrefix(prefixName);
        return this->loadObject(objFileName);
    }

    //template <typename T, typename Index>
    //void loadObjectDirect(
    //        const std::string &prefixName,
    //        const std::string &objFileName,
    //        T* posPtr,
    //        Index* facetPtr) {

    //    std::fstream file(prefix + objFileName);
    //    if (!file.is_open()) {
    //        std::cerr << "No .obj file" << std::endl;
    //        return;
    //    }
    //    std::cout << "Read " << prefix + objFileName << std::endl;

    //    std::vector<std::string> faces;

    //    Index vdataid = 0;

    //    std::string type;
    //    while (!file.eof()) {
    //        file >> type;
    //        if (file.eof())
    //            break;
    //        if (type == "mtllib") {
    //            file >> this->materialFile;
    //            this->loadMtl(this->materialFile);
    //        } else if (type == "usemtl") {
    //            file >> this->material;
    //        } else if (type == "o" || type == "g") {
    //            std::string dummy;
    //            if (!std::getline(file, dummy)) {
    //                std::cerr << "Group/Object line skipping failed!" << std::endl;
    //            }
    //        } else if (type == "v") {
    //            float x, y, z;
    //            file >> x >> y >> z;
    //            //this->vertices.push_back({x, y, z});
    //            posPtr[vdataid++] = x;
    //            posPtr[vdataid++] = y;
    //            posPtr[vdataid++] = z;
    //        } else if (type == "vt") {
    //            // ignore
    //            float tx, ty;
    //            file >> tx >> ty;
    //        } else if (type == "vn") {
    //            // ignore
    //            float nx, ny, nz;
    //            file >> nx >> ny >> nz;
    //        } else if (type == "f") {
    //            std::string f;
    //            std::getline(file, f);

    //            faces.push_back(f);
    //        } else if (type == "l") {
    //            // not in this case
    //            if (!file.ignore(std::numeric_limits<std::streamsize>::max(),
    //                             file.widen('\n'))) {
    //                std::cerr << "Line polygon skip!" << std::endl;
    //            }
    //        } else {
    //            if (!file.ignore(std::numeric_limits<std::streamsize>::max(),
    //                             file.widen('\n'))) {
    //                std::cerr << "Weird situation! input " << type
    //                          << " is not supported." << std::endl;
    //                return;
    //            }
    //        }
    //        // std::cout << "Processing type " << type << std::endl;
    //    }

    //    this->nVertices = (int)this->vertices.size();

    //    //std::vector<std::vector<tinym::vec3>> sNormals(this->nVertices);

    //    Index fdataid = 0;
    //    //bool hasNormal = false;
    //    for (auto f : faces) {
    //        // case by case?
    //        std::vector<uint> elem;

    //        std::regex re("-?\\d+(/-?\\d+|/)*");
    //        auto start = std::sregex_iterator(f.begin(), f.end(), re);
    //        auto end = std::sregex_iterator();
    //        while (start != end) {
    //            std::string str = start->str();
    //            int slashCount = 0;
    //            for(auto& c : str) if(c == '/') slashCount++;
    //            if(slashCount == 2) { // v/t/n or v//n
    //                GLint vertex = std::stoi(str.substr(0, str.find('/'))) - 1;
    //                if(vertex < 0) vertex += vertices.size(); // -1 to be the last index
    //                elem.push_back(vertex);

    //                GLint normal = std::stoi(str.substr(str.find_last_of('/') + 1)) - 1;
    //                if(normal < 0) normal += normals.size();
    //                //sNormals[vertex].push_back(this->normals[normal]);
    //                //hasNormal = true;
    //            }
    //            else if(slashCount == 1) { // v/t
    //                GLint vertex = std::stoi(str.substr(0, str.find('/'))) - 1;
    //                if(vertex < 0) vertex += vertices.size(); // -1 to be the last index
    //                elem.push_back(vertex);
    //            } else if(slashCount == 0) { // no slash, only vertices
    //                GLint vertex = std::stoi(str) - 1;
    //                if(vertex < 0) vertex += vertices.size();
    //                elem.push_back(vertex);
    //            }
    //            start++;
    //        }

    //        if (elem.size() == 4) {
    //            //this->elements4.push_back({elem[0], elem[1], elem[2], elem[3]});
    //            //this->elements3.push_back({elem[0], elem[1], elem[2]});
    //            //this->elements3.push_back({elem[0], elem[2], elem[3]});
    //            facetPtr[fdataid++] = elem[0];
    //            facetPtr[fdataid++] = elem[1];
    //            facetPtr[fdataid++] = elem[2];
    //            facetPtr[fdataid++] = elem[0];
    //            facetPtr[fdataid++] = elem[2];
    //            facetPtr[fdataid++] = elem[3];
    //        } else if (elem.size() == 3) {
    //            //this->elements3.push_back({elem[0], elem[1], elem[2]});
    //            facetPtr[fdataid++] = elem[0];
    //            facetPtr[fdataid++] = elem[1];
    //            facetPtr[fdataid++] = elem[2];
    //        } else {
    //            std::cerr << "Weird situation! f elements size is not 3 or 4."
    //                      << std::endl;
    //            return;
    //        }
    //    }
    //    file.close();
    //    
    //    std::cout << "--- Wavefront Object Loaded Directly ---" << std::endl;
    //}
    

    // not completed. cannot use.
    void writeObject(const std::string& outFileName) {
        std::ofstream ofs(outFileName);
        if (nVertices > 0) {
            for(const auto& v : vertices) ofs << "v " << v.x << " " << v.y << " " << v.z << std::endl;
        }
        if (nSyncedNormals) {
            for(const auto& n : syncedNormals) ofs << "vn " << n.x << " " << n.y << " " << n.z << std::endl;
        }
        if (nTextures) {
            for(const auto& t : texCoords) ofs << "vt " << t.x << " " << t.y << std::endl;
        }
        if(nElements3) {
            for(const auto& f : elements3) ofs << "f " << f.x << " " << f.y << " " << f.z << std::endl;
        }
        ofs.close();
    }
    
    void generateBuffers() {
        if(vao == 0) {
            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);
            
            glGenBuffers(1, &vertexBuffer);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
            glBufferData(GL_ARRAY_BUFFER,
                         nVertices * sizeof(tinym::vec3),
                         vertices.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
            
            glGenBuffers(1, &syncedNormalBuffer);
            glBindBuffer(GL_ARRAY_BUFFER, syncedNormalBuffer);
            glBufferData(GL_ARRAY_BUFFER,
                         nSyncedNormals * sizeof(tinym::vec3),
                         syncedNormals.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
            
            if(nTextures != 0) {
                glGenBuffers(1, &texCoordBuffer);
                glBindBuffer(GL_ARRAY_BUFFER, texCoordBuffer);
                glBufferData(GL_ARRAY_BUFFER,
                             nTextures * sizeof(tinym::vec3),
                             texCoords.data(),
                             GL_STATIC_DRAW);
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
            }
            
            glGenBuffers(1, &element3Buffer);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element3Buffer);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                         nElements3 * sizeof(tinym::vec3ui),
                         elements3.data(),
                         GL_STATIC_DRAW);

            isOk = true;
        }
    }
    
    void adjustCenter(bool invScaling = false) {
        float sf = tinym::max(scale);
        for(int i = 0; i < vertices.size(); i++) {
            vertices[i] -= center;
            if(invScaling) vertices[i] /= sf;
        }
    }

    bool renderErrLogOnce = false;
    void render() {
        if(!isOk) {
            if(!renderErrLogOnce) {
                renderErrLogOnce = true;
                std::cout << "Obj Not Set" << std::endl;
            }
            return;
        }

        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element3Buffer);
        
        glDrawElements(GL_TRIANGLES, nElements3 * 3, GL_UNSIGNED_SHORT, 0);
    }
};
