#include "util/Intrinsics.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace util {

// ── Minimale JSON-parser voor het specifieke intrinsics-formaat ───────────────
// Vermijdt extra dependencies (nlohmann/json). Leest alleen wat we nodig hebben.

static double jsonScalar(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos)
        throw std::runtime_error("Intrinsics JSON: sleutel '" + key + "' niet gevonden");
    pos = json.find(':', pos);
    return std::stod(json.substr(pos + 1));
}

static std::vector<double> jsonArray(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos)
        throw std::runtime_error("Intrinsics JSON: array '" + key + "' niet gevonden");
    auto open  = json.find('[', pos);
    auto close = json.find(']', open);
    std::vector<double> vals;
    std::istringstream ss(json.substr(open + 1, close - open - 1));
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        // verwijder spaties / newlines
        tok.erase(std::remove_if(tok.begin(), tok.end(),
                  [](char c){ return std::isspace(c); }), tok.end());
        if (!tok.empty())
            vals.push_back(std::stod(tok));
    }
    return vals;
}

Intrinsics loadIntrinsics(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Kan intrinsics niet openen: " + path);
    std::string json((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());

    double fx = jsonScalar(json, "fx");
    double fy = jsonScalar(json, "fy");
    double cx = jsonScalar(json, "cx");
    double cy = jsonScalar(json, "cy");
    auto   d  = jsonArray(json, "dist");

    Intrinsics intr;
    intr.K = (cv::Mat_<double>(3, 3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1);
    intr.dist = cv::Mat(d, true).reshape(1, 1);  // 1×N CV_64F
    return intr;
}

// ── Sterkaart CSV (id,x,y,z) ──────────────────────────────────────────────────

StarMap loadStarMap(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Kan sterkaart niet openen: " + path);

    StarMap map;
    std::string line;
    std::getline(f, line); // header overslaan
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        std::vector<std::string> cols;
        while (std::getline(ss, tok, ',')) cols.push_back(tok);
        if (cols.size() < 3) continue;
        map.emplace_back(std::stof(cols[1]), std::stof(cols[2]));
    }
    return map;
}

void saveStarMap(const StarMap& map, const std::string& path) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Kan sterkaart niet schrijven: " + path);
    f << "id,x,y,z\n";
    for (size_t i = 0; i < map.size(); ++i)
        f << i << "," << map[i].x << "," << map[i].y << ",0.000000\n";
}

// ── StarMap3D ─────────────────────────────────────────────────────────────────

StarMap3D loadStarMap3D(const std::string& path, StarMap3DMetadata* meta) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Kan sterkaart3D niet openen: " + path);

    StarMap3D map;
    std::string header;
    std::getline(f, header);

    // Detect format by checking header
    bool is_metric = (header.find("x_m") != std::string::npos);
    ScaleStatus status = is_metric ? ScaleStatus::Metric : ScaleStatus::Legacy;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string tok;
        std::vector<std::string> cols;
        while (std::getline(ss, tok, ',')) cols.push_back(tok);
        if (cols.size() < 4) continue;
        MapMarker m;
        m.id           = std::stoi(cols[0]);
        m.p_world_m.x  = std::stof(cols[1]);
        m.p_world_m.y  = std::stof(cols[2]);
        m.p_world_m.z  = std::stof(cols[3]);
        map.push_back(m);
    }

    if (meta) {
        meta->scale_status = status;
    }
    return map;
}

void saveStarMap3D(const StarMap3D& map, const std::string& path,
                   const StarMap3DMetadata* meta) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Kan sterkaart3D niet schrijven: " + path);
    f << "id,x_m,y_m,z_m\n";
    for (const auto& m : map) {
        f << m.id << ","
          << m.p_world_m.x << ","
          << m.p_world_m.y << ","
          << m.p_world_m.z << "\n";
    }
    // Anchors as JSON comment block if present
    if (meta && !meta->anchors.empty()) {
        // Write sidecar JSON next to the CSV
        std::string jsonPath = path.substr(0, path.rfind('.')) + "_anchors.json";
        std::ofstream jf(jsonPath);
        if (jf) {
            jf << "{\"scale_anchors\":[\n";
            for (size_t i = 0; i < meta->anchors.size(); ++i) {
                const auto& a = meta->anchors[i];
                jf << "  {\"id_a\":" << a.id_a
                   << ",\"id_b\":" << a.id_b
                   << ",\"distance_m\":" << a.distance_m << "}";
                if (i + 1 < meta->anchors.size()) jf << ",";
                jf << "\n";
            }
            jf << "]}\n";
        }
    }
}

StarMap3D starMap3DFromLegacy(const StarMap& map) {
    StarMap3D result;
    result.reserve(map.size());
    for (size_t i = 0; i < map.size(); ++i) {
        MapMarker m;
        m.id          = static_cast<int>(i);
        m.p_world_m.x = map[i].x;
        m.p_world_m.y = map[i].y;
        m.p_world_m.z = 0.f;
        result.push_back(m);
    }
    return result;
}

StarMap starMapFromStarMap3D(const StarMap3D& map) {
    StarMap result;
    result.reserve(map.size());
    for (const auto& m : map)
        result.emplace_back(m.p_world_m.x, m.p_world_m.y);
    return result;
}

} // namespace util
