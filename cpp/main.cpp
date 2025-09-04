#include "utils.hpp"
#include "shortest_path.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

SequenceOfBundles load_sequence_from_file(const std::string& filename, bool preprocess = true) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    float radius = 0.0f;
    std::vector<Point> vertices;
    std::vector<std::pair<int, Point>> line_segments;
    std::string section; // Track current section

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Detect section headers
        if (line.find("Radius:") == 0) {
            section = "Radius";
            std::string radius_str = line.substr(line.find(":") + 1);
            radius = std::stof(radius_str);
        } 
        else if (line.find("Vertices:") == 0) {
            section = "Vertices";
        }
        else if (line.find("LineSegments:") == 0) {
            section = "LineSegments";
        }
        else {
            // Parse data based on current section
            std::istringstream iss(line);
            if (section == "Vertices") {
                double x, y;
                if (iss >> x >> y) {
                    vertices.push_back(Point(x, y));
                }
            }
            else if (section == "LineSegments") {
                int vertex_index;
                double x, y;
                if (iss >> vertex_index >> x >> y) {
                    line_segments.push_back({vertex_index, Point(x, y)});
                }
            }
        }
    }
    
    file.close();
    
    // Validate required data
    if (radius <= 0.0f || vertices.empty()) {
        throw std::runtime_error("Input file is missing required data: Radius or Vertices");
    }
    
    // Initialize the sequence
    SequenceOfBundles sequence(vertices, radius);
    
    // Add line segments to the sequence
    for (const auto& [vertex_index, outer_endpoint] : line_segments) {
        if (vertex_index >= 0 && vertex_index < vertices.size()) {
            sequence.addOuterEndpoint(vertices[vertex_index], outer_endpoint);
        } else {
            std::cerr << "Invalid vertex index " << vertex_index << " for line segment " 
                      << outer_endpoint.x << "," << outer_endpoint.y << std::endl;
        }
    }
    
    if (preprocess) {
        sequence.preprocess();
    }
    
    return sequence;
}

void write_shortest_path_to_file(const std::vector<Point>& shortest_path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    
    // Write header
    file << "# Shortest path coordinates\n";
    file << "# Format: x y\n";
    
    // Write each point's coordinates
    for (const auto& point : shortest_path) {
        file << point.x << " " << point.y << "\n";
    }
    
    file.close();
    
    std::cout << "Shortest path written to: " << filename << std::endl;
}

int main() {
    SequenceOfBundles sequence = load_sequence_from_file("D:/MARS_Workspace/Convex-Hull/Incremental-Convex-Hull-etc_Implementation/incremental-convex-hull-on-bundles-of-line-segments/input/input_1.txt", false);
    SimplePolygonFromSequenceOfBundle polygon(sequence);
    std::vector<Point> shortest_path = polygon.findShortestPath(true);
    write_shortest_path_to_file(shortest_path, "shortest_path_log.txt");
    std::cout<<"run successully";
    return 0;
}