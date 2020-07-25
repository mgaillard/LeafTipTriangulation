#include "OBJWriter.h"

#include <algorithm>
#include <fstream>

const std::vector<glm::vec3>& OBJWriter::vertices() const
{
    return m_vertices;
}

const std::vector<glm::vec3>& OBJWriter::normals() const
{
    return m_normals;
}

const std::vector<std::pair<int, int>>& OBJWriter::lines() const
{
    return m_lines;
}

const std::vector<std::tuple<int, int, int>>& OBJWriter::faces() const
{
    return m_faces;
}

bool OBJWriter::save(const std::string& filename) const
{
    // Write vertices to a file
    std::ofstream file(filename, std::fstream::out);

    if (!file.is_open())
    {
        return false;
    }

    for (const auto& vertex : m_vertices)
    {
        // Vertex
        file << "v " << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }

    for (const auto& normal : m_normals)
    {
        // Normal
        file << "vn " << normal.x << " " << normal.y << " " << normal.z << "\n";
    }

    for (const auto& line : m_lines)
    {
        // Line segment
        file << "l " << line.first + 1 << " " << line.second + 1 << "\n";
    }

    for (const auto& face : m_faces)
    {
        if (m_normals.empty())
        {
            // Face without normals
            file << "f " << std::get<0>(face) + 1 << " " << std::get<1>(face) + 1 << " " << std::get<2>(face) + 1 << "\n";
        }
        else
        {
            // Face with normals
            file << "f " << std::get<0>(face) + 1 << "//" << std::get<0>(face) + 1 << " "
                << std::get<1>(face) + 1 << "//" << std::get<1>(face) + 1 << " "
                << std::get<2>(face) + 1 << "//" << std::get<2>(face) + 1 << "\n";
        }
    }

    file.close();

    return true;
}

void OBJWriter::clear()
{
    m_vertices.clear();
    m_lines.clear();
    m_faces.clear();
}

void OBJWriter::setVertices(std::vector<glm::vec3> vertices)
{
    m_vertices = std::move(vertices);
}

void OBJWriter::setNormals(std::vector<glm::vec3> normals)
{
    m_normals = std::move(normals);
}

void OBJWriter::setLines(std::vector<std::pair<int, int>> lines)
{
    m_lines = std::move(lines);
}

void OBJWriter::setFaces(std::vector<std::tuple<int, int, int>> faces)
{
    m_faces = std::move(faces);
}

int OBJWriter::findVertex(const glm::vec3& vertex) const
{
    const auto it = std::find(m_vertices.begin(), m_vertices.end(), vertex);

    if (it != m_vertices.end())
    {
        return std::distance(m_vertices.begin(), it);
    }

    return -1;
}

int OBJWriter::addVertexSafe(const glm::vec3& vertex)
{
    const auto index = findVertex(vertex);

    if (index >= 0)
    {
        return index;
    }

    return addVertex(vertex);
}

int OBJWriter::addVertex(const glm::vec3& vertex)
{
    m_vertices.push_back(vertex);

    return m_vertices.size() - 1;
}

int OBJWriter::addNormal(const glm::vec3& normal)
{
    m_normals.push_back(normal);

    return m_normals.size() - 1;
}

void OBJWriter::addLine(int a, int b)
{
    assert(a >= 0 && a < m_vertices.size());
    assert(b >= 0 && b < m_vertices.size());

    m_lines.emplace_back(a, b);
}

void OBJWriter::addLine(const glm::vec3& a, const glm::vec3& b)
{
    const auto ia = addVertex(a);
    const auto ib = addVertex(b);

    addLine(ia, ib);
}

void OBJWriter::addFace(int a, int b, int c)
{
    assert(a >= 0 && a < m_vertices.size());
    assert(b >= 0 && b < m_vertices.size());
    assert(c >= 0 && c < m_vertices.size());

    m_faces.emplace_back(a, b, c);
}

void OBJWriter::addFace(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
{
    const auto ia = addVertex(a);
    const auto ib = addVertex(b);
    const auto ic = addVertex(c);

    addFace(ia, ib, ic);
}
