#include "sim/SceneCsv.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace sim {

namespace {

struct CsvColumns {
    std::size_t id = std::numeric_limits<std::size_t>::max();
    std::size_t x = std::numeric_limits<std::size_t>::max();
    std::size_t y = std::numeric_limits<std::size_t>::max();
    std::size_t r = std::numeric_limits<std::size_t>::max();
    std::size_t g = std::numeric_limits<std::size_t>::max();
    std::size_t b = std::numeric_limits<std::size_t>::max();
    std::size_t radius = std::numeric_limits<std::size_t>::max();
};

struct SceneMetadata {
    bool hasBounds = false;
    WorldBounds bounds;
    std::vector<Wall> walls;
};

std::string trim(std::string text) {
    const auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
    text.erase(text.begin(), std::find_if(text.begin(), text.end(), [&](char c) {
                   return !isSpace(static_cast<unsigned char>(c));
               }));
    text.erase(std::find_if(text.rbegin(), text.rend(), [&](char c) {
                   return !isSpace(static_cast<unsigned char>(c));
               }).base(),
               text.end());
    return text;
}

std::string lower(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

std::vector<std::string> parseCsvLine(const std::string& line) {
    std::vector<std::string> cells;
    std::string current;
    bool inQuotes = false;

    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (inQuotes) {
            if (ch == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    current.push_back('"');
                    ++i;
                } else {
                    inQuotes = false;
                }
            } else {
                current.push_back(ch);
            }
        } else if (ch == '"') {
            inQuotes = true;
        } else if (ch == ',') {
            cells.push_back(trim(current));
            current.clear();
        } else {
            current.push_back(ch);
        }
    }

    cells.push_back(trim(current));
    return cells;
}

bool parseDouble(const std::string& text, double& value) {
    const std::string trimmed = trim(text);
    if (trimmed.empty()) {
        return false;
    }
    const char* begin = trimmed.c_str();
    char* end = nullptr;
    value = std::strtod(begin, &end);
    return end != nullptr && *end == '\0';
}

bool parseInt(const std::string& text, int& value) {
    const std::string trimmed = trim(text);
    if (trimmed.empty()) {
        return false;
    }
    const char* begin = trimmed.c_str();
    char* end = nullptr;
    const long parsed = std::strtol(begin, &end, 10);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

std::size_t findColumn(const std::vector<std::string>& headers, std::string_view name) {
    for (std::size_t index = 0; index < headers.size(); ++index) {
        if (lower(trim(headers[index])) == name) {
            return index;
        }
    }
    return std::numeric_limits<std::size_t>::max();
}

ColorRGBA makeColor(int r, int g, int b) {
    return {
        static_cast<std::uint8_t>(std::clamp(r, 0, 255)),
        static_cast<std::uint8_t>(std::clamp(g, 0, 255)),
        static_cast<std::uint8_t>(std::clamp(b, 0, 255)),
        255};
}

void parseMetadataLine(const std::string& trimmedLine, SceneMetadata& metadata) {
    if (trimmedLine.empty() || trimmedLine.front() != '#') {
        return;
    }

    const std::string content = trim(trimmedLine.substr(1));
    if (content.empty()) {
        return;
    }

    const std::vector<std::string> cells = parseCsvLine(content);
    if (cells.empty()) {
        return;
    }

    const std::string tag = lower(trim(cells.front()));
    if (tag == "bounds") {
        if (cells.size() != 5) {
            throw std::runtime_error("bounds metadata must be # bounds,minX,minY,maxX,maxY");
        }

        WorldBounds bounds;
        if (!parseDouble(cells[1], bounds.minX) ||
            !parseDouble(cells[2], bounds.minY) ||
            !parseDouble(cells[3], bounds.maxX) ||
            !parseDouble(cells[4], bounds.maxY)) {
            throw std::runtime_error("failed to parse bounds metadata");
        }
        metadata.hasBounds = true;
        metadata.bounds = bounds;
        return;
    }

    if (tag == "wall") {
        if (cells.size() != 5) {
            throw std::runtime_error("wall metadata must be # wall,ax,ay,bx,by");
        }

        Wall wall;
        if (!parseDouble(cells[1], wall.a.x) ||
            !parseDouble(cells[2], wall.a.y) ||
            !parseDouble(cells[3], wall.b.x) ||
            !parseDouble(cells[4], wall.b.y)) {
            throw std::runtime_error("failed to parse wall metadata");
        }
        metadata.walls.push_back(wall);
    }
}

}  // namespace

void loadSceneCsv(const std::filesystem::path& path, Scene& scene) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open scene csv: " + path.string());
    }

    std::string line;
    std::vector<std::string> headers;
    CsvColumns columns;
    bool parsedHeader = false;
    std::vector<Ball> balls;
    SceneMetadata metadata;

    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const std::string trimmed = trim(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed.front() == '#') {
            parseMetadataLine(trimmed, metadata);
            continue;
        }
        if (!parsedHeader) {
            headers = parseCsvLine(trimmed);
            columns.id = findColumn(headers, "ball_id");
            columns.x = findColumn(headers, "x");
            columns.y = findColumn(headers, "y");
            columns.r = findColumn(headers, "r");
            columns.g = findColumn(headers, "g");
            columns.b = findColumn(headers, "b");
            columns.radius = findColumn(headers, "radius");
            if (columns.x == std::numeric_limits<std::size_t>::max() ||
                columns.y == std::numeric_limits<std::size_t>::max() ||
                columns.r == std::numeric_limits<std::size_t>::max() ||
                columns.g == std::numeric_limits<std::size_t>::max() ||
                columns.b == std::numeric_limits<std::size_t>::max()) {
                throw std::runtime_error("scene csv header must contain x,y,r,g,b columns");
            }
            parsedHeader = true;
            continue;
        }

        const std::vector<std::string> cells = parseCsvLine(trimmed);
        if (cells.size() < headers.size()) {
            throw std::runtime_error("scene csv row has fewer fields than the header");
        }

        double x = 0.0;
        double y = 0.0;
        int r = 0;
        int g = 0;
        int b = 0;
        double radius = 6.0;
        if (!parseDouble(cells[columns.x], x) || !parseDouble(cells[columns.y], y) ||
            !parseInt(cells[columns.r], r) || !parseInt(cells[columns.g], g) ||
            !parseInt(cells[columns.b], b)) {
            throw std::runtime_error("failed to parse scene csv row");
        }
        if (columns.radius != std::numeric_limits<std::size_t>::max() &&
            !cells[columns.radius].empty() &&
            !parseDouble(cells[columns.radius], radius)) {
            throw std::runtime_error("failed to parse radius in scene csv row");
        }

        Ball ball;
        ball.position = {x, y};
        ball.previousPosition = ball.position;
        ball.velocity = {0.0, 0.0};
        ball.radius = radius;
        ball.inverseMass = 1.0 / std::max(1.0, radius * radius);
        ball.color = makeColor(r, g, b);
        balls.push_back(ball);
    }

    if (!parsedHeader) {
        throw std::runtime_error("scene csv did not contain a header row");
    }

    if (metadata.hasBounds) {
        scene.bounds = metadata.bounds;
    }
    if (!metadata.walls.empty()) {
        scene.walls = std::move(metadata.walls);
    }
    scene.balls = std::move(balls);
}

void saveSceneCsv(const std::filesystem::path& path, const Scene& scene) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        throw std::runtime_error("failed to open scene csv for writing: " + path.string());
    }

    out << std::setprecision(17);
    out << "# bounds," << scene.bounds.minX << ',' << scene.bounds.minY << ','
        << scene.bounds.maxX << ',' << scene.bounds.maxY << '\n';
    for (const Wall& wall : scene.walls) {
        out << "# wall," << wall.a.x << ',' << wall.a.y << ','
            << wall.b.x << ',' << wall.b.y << '\n';
    }
    out << "ball_id,x,y,r,g,b,radius\n";
    for (std::size_t index = 0; index < scene.balls.size(); ++index) {
        const Ball& ball = scene.balls[index];
        out << index << ','
            << ball.position.x << ',' << ball.position.y << ','
            << static_cast<int>(ball.color.r) << ','
            << static_cast<int>(ball.color.g) << ','
            << static_cast<int>(ball.color.b) << ','
            << ball.radius << '\n';
    }
}

}  // namespace sim
