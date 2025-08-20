#include<shortest_path.hpp>

#include <algorithm>
#include <iostream>
#include <cmath>

SequenceOfBundles::SequenceOfBundles(const std::vector<Point>& skeleton, float radius) 
    : skeleton(skeleton), radius(radius) {}

SequenceOfBundles::SequenceOfBundles(std::vector<Point>&& skeleton, float radius) 
    : skeleton(std::move(skeleton)), radius(radius) {}
    
void SequenceOfBundles::addOuterEndpoint(const Point& vertex, const Point& outer_endpoint) {
    if (std::find(skeleton.begin(), skeleton.end(), vertex) == skeleton.end()) {
        // Vertex not in the skeleton
        std::cerr << "Vertex [" << vertex.x << ", " << vertex.y 
                  << "] not in the skeleton." << std::endl;
        return;        
    }
    if (vertex == skeleton.front() || vertex == skeleton.back()) {
        // Cannot add segments to the first and last bundles
        std::cerr << "Cannot add line segments [" << vertex.x << ", " << vertex.y 
                  << "] into the first and last bundles." << std::endl;
        return;
    }
    
    float distance = calculateDistance(vertex, outer_endpoint);
    if (distance > radius) {
        // Preprocess the outer endpoint to fit within the radius
        Point diff = {outer_endpoint.x - vertex.x, outer_endpoint.y - vertex.y};
        float magnitude = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        diff.x /= magnitude;
        diff.y /= magnitude;
        outer_endpoint = {vertex.x + diff.x * radius, vertex.y + diff.y * radius};
    }

    size_t index = std::distance(skeleton.begin(), std::find(skeleton.begin(), skeleton.end(), vertex));
    Point prev_vertex = skeleton[index - 1];
    Point next_vertex = skeleton[index + 1];

    // Ensure the added segment lies in the sector that is smaller than pi
    // This part of the logic is omitted for brevity

    if (outer_endpoints[index].empty()) {
        outer_endpoints[index].push_back(outer_endpoint);
    } else {
        for (size_t i = 0; i < outer_endpoints[index].size(); ++i) {
            Point p = outer_endpoints[index][i];
            // Angle comparison logic is omitted for brevity
            // Insert or skip based on angle comparison
        }
        outer_endpoints[index].push_back(outer_endpoint);
    }
}