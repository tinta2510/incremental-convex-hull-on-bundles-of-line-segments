#include"shortest_path.hpp"

#include <algorithm>
#include <iostream>
#include <cmath>

// --- Struct SequenceOfBundles - START ---
SequenceOfBundles::SequenceOfBundles(const std::vector<Point>& skeleton, float radius) 
    : skeleton(skeleton), radius(radius), outer_endpoints(skeleton.size()) {}

SequenceOfBundles::SequenceOfBundles(std::vector<Point>&& skeleton, float radius) 
    : skeleton(std::move(skeleton)), radius(radius), outer_endpoints(skeleton.size()) {}
    
void SequenceOfBundles::addOuterEndpoint(const Point& vertex, const Point& outer_endpoint) {
    auto iter = std::find(skeleton.begin(), skeleton.end(), vertex);
    if (iter == skeleton.end()) {
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

    size_t index = std::distance(skeleton.begin(), iter);
    Point prev_vertex = skeleton[index - 1];
    Point next_vertex = skeleton[index + 1];

    // Ensure the added segment lies in the sector that is smaller tha
    if (!isEqual(calculateAngle(prev_vertex, vertex, next_vertex),
                 calculateAngle(prev_vertex, vertex, outer_endpoint) 
                    + calculateAngle(outer_endpoint, vertex, next_vertex))) {
        std::cerr << "The segment [" << vertex.x << ", " << vertex.y 
                  << "] must lie in the sector that is smaller than pi." << std::endl;
        return;
    }
    if (outer_endpoints[index].empty()) {
        outer_endpoints[index].push_back(outer_endpoint);
    } else {
        size_t size = outer_endpoints[index].size();
        for (size_t i = 0; i < size; ++i) {
            Point p = outer_endpoints[index][i];
            float angle1 = calculateAngle(prev_vertex, vertex, p);
            float angle2 = calculateAngle(prev_vertex, vertex, outer_endpoint);

            if (angle1 > angle2) {
                outer_endpoints[index].insert(outer_endpoints[index].begin() + i, outer_endpoint);
                return;
            } else if (isEqual(angle1, angle2)) {
                // If angles are equal, do not insert the endpoint
                return;
            }
        }
        outer_endpoints[index].push_back(outer_endpoint);
    }
}

void SequenceOfBundles::preprocess() {
    float min_dist = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < skeleton.size(); ++i) {
        for (size_t j = i + 1; j < skeleton.size(); ++j) {
            float dist = calculateDistance(skeleton[i], skeleton[j]);
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
    }
    float min_radius = 0.5f * min_dist;
    
    // Adjust outer endpoints if they exceed the minimum radius
    for (size_t i = 0; i < skeleton.size(); ++i) {
        Point& vertex = skeleton[i];
        for (size_t j = 0; j < outer_endpoints[i].size(); ++j) {
            const Point& outer_endpoint = outer_endpoints[i][j];
            float dist = calculateDistance(vertex, outer_endpoint);
            if (dist > min_radius) {
                Point diff = outer_endpoint - vertex;
                float mag = diff.magnitude();
                if (mag != 0.0f) {
                    Point direction = diff / mag;
                    outer_endpoints[i][j] = vertex + direction * min_radius;
                }
            }
        }
    }
}
// --- Struct SequenceOfBundles - END ---

// --- Struct SimplePolygon - START ---
SimplePolygon::SimplePolygon()
    : polyline_P(std::vector<Point>()), polyline_Q(std::vector<Point>()) {}
void SimplePolygon::initialize() {
    if (polyline_P.empty() || polyline_Q.empty()) {
        throw std::invalid_argument("Polylines cannot be empty.");
    }
    if (polyline_P.front() != polyline_Q.front() || polyline_P.back() != polyline_Q.back()) {
        throw std::invalid_argument("Start and end point of two polylines must be the same.");
    }
}

SimplePolygon::SimplePolygon(const std::vector<Point>& polyline_P, const std::vector<Point>& polyline_Q) 
    : polyline_P(polyline_P), polyline_Q(polyline_Q) {
    initialize();
}

SimplePolygon::SimplePolygon(std::vector<Point>&& polyline_P, std::vector<Point>&& polyline_Q) 
    : polyline_P(std::move(polyline_P)), polyline_Q(std::move(polyline_Q)) {
    initialize();
}

bool SimplePolygon::isInsideNewHull(const Point& left_tp, const Point& right_tp,
                                   const Point& added_pt, const Point& checking_pt, bool direction) const {
    return (isLeft(left_tp, added_pt, checking_pt, direction) &&
            isLeft(added_pt, right_tp, checking_pt, direction) &&
            isLeft(right_tp, left_tp, checking_pt, direction));
}

bool SimplePolygon::verifyLink(const std::vector<Point>& Xstar, const std::vector<Point>& Ystar,
                              const Point& Ustar, const Point& Vstar, bool direction) const {
    for (const Point& pt : Xstar) {
        if (!isLeftOn(Ustar, Vstar, pt, direction)) {
            return false;
        }
    }
    for (const Point& pt : Ystar) {
        if (!isLeftOn(Ustar, Vstar, pt, !direction)) {
            return false;
        }
    }
    return true;
}

std::pair<std::optional<Point>, std::optional<Point>> SimplePolygon::findLink(
    const std::vector<Point>& Xstar, const std::vector<Point>& Ystar, bool direction) const {
    for (const Point& Ustar : Xstar) {
        for (const Point& Vstar : Ystar) {
            if (verifyLink(Xstar, Ystar, Ustar, Vstar, direction)) {
                return {Ustar, Vstar};
            }
        }
    }
    return {std::nullopt, std::nullopt};
}

std::vector<Point> SimplePolygon::getTangentLine(const std::vector<Point>& tangent_polyline,
                                                size_t start_tp_idx, size_t end_tp_idx) const {
    std::vector<Point> result;
    
    if (end_tp_idx < start_tp_idx) {
        // Wrap around case
        for (size_t i = start_tp_idx; i < tangent_polyline.size(); ++i) {
            result.push_back(tangent_polyline[i]);
        }
        for (size_t i = 1; i <= end_tp_idx; ++i) {
            result.push_back(tangent_polyline[i]);
        }
    } else {
        // Normal case
        for (size_t i = start_tp_idx; i <= end_tp_idx; ++i) {
            result.push_back(tangent_polyline[i]);
        }
    }
    return result;
}

std::pair<size_t, size_t> SimplePolygon::findTangentPoints(const std::vector<Point>& tangent_polyline,
                                                          const Point& added_pt, bool direction) const {
    size_t left_tp_idx = tangent_polyline.size() - 1;
    while (left_tp_idx > 0 && !isLeft(tangent_polyline[left_tp_idx - 1],
                                     tangent_polyline[left_tp_idx],
                                     added_pt, direction)) {
        --left_tp_idx;
    }

    size_t right_tp_idx = 0;
    while (right_tp_idx < tangent_polyline.size() - 1 && 
           !isLeft(added_pt, tangent_polyline[right_tp_idx],
                   tangent_polyline[right_tp_idx + 1], direction)) {
        ++right_tp_idx;
    }

    return {left_tp_idx, right_tp_idx};
}

size_t SimplePolygon::findLeftTangentPoint(const std::vector<Point>& tangent_polyline,
                                          const Point& added_pt, bool direction) const {
    size_t left_tp_idx = tangent_polyline.size() - 1;
    while (left_tp_idx > 0 && !isLeft(tangent_polyline[left_tp_idx - 1],
                                     tangent_polyline[left_tp_idx],
                                     added_pt, direction)) {
        --left_tp_idx;
    }
    return left_tp_idx;
}

std::vector<Point> SimplePolygon::findShortestPath(bool direction) {
    size_t count = 0;
    std::vector<Point> shortest_path;
    size_t start_index = 0;
    size_t checking_pt_index = 0;
    
    const std::vector<Point>* curr_polyline = direction ? &polyline_P : &polyline_Q;
    
    while (true) {
        const std::vector<Point>* dual_polyline = (curr_polyline == &polyline_P) ? &polyline_Q : &polyline_P;
        std::vector<Point> tangent_polyline;

        if ((*curr_polyline)[start_index] == polyline_P.back()) {
            shortest_path.push_back((*curr_polyline)[start_index]);
            return shortest_path;
        } else if (start_index + 1 < curr_polyline->size() && 
                  (*curr_polyline)[start_index + 1] == polyline_P.back()) {
            shortest_path.push_back((*curr_polyline)[start_index]);
            shortest_path.push_back((*curr_polyline)[start_index + 1]);
            return shortest_path;
        }

        // Initialize the tangent polyline
        tangent_polyline.push_back((*curr_polyline)[start_index]);
        tangent_polyline.push_back((*curr_polyline)[start_index + 1]);

        // Increment the convex hull
        for (size_t i = start_index + 2; i < curr_polyline->size(); ++i) {
            const Point& added_pt = (*curr_polyline)[i];

            // Find the tangent points
            size_t left_tp_idx = findLeftTangentPoint(tangent_polyline, added_pt, direction);

            // Check intersection
            ++count;
            std::vector<Point> Ystar;
            bool intersection = false;
            
            for (size_t j = checking_pt_index; j < dual_polyline->size() - 1; ++j) {
                const Point& prev_pt = (*dual_polyline)[j];
                const Point& pt = (*dual_polyline)[j + 1];
                
                if (!isLeft(prev_pt, tangent_polyline[left_tp_idx], added_pt, direction) &&
                    isLeft(pt, tangent_polyline[left_tp_idx], added_pt, direction) &&
                    doIntersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt)) {
                    intersection = true;
                    Ystar.push_back(pt);
                } else if (isLeft(pt, tangent_polyline[left_tp_idx], added_pt, direction) && intersection) {
                    Ystar.push_back(pt);
                } else if (!isLeft(pt, tangent_polyline[left_tp_idx], added_pt, direction) &&
                          doIntersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt)) {
                    intersection = false;
                }
            }

            if (!Ystar.empty()) {
                // Find link
                std::vector<Point> Xstar(tangent_polyline.begin() + left_tp_idx, tangent_polyline.end());
                auto [Ustar_opt, Vstar_opt] = findLink(Xstar, Ystar, direction);
                
                if (!Ustar_opt.has_value()) {
                    throw std::runtime_error("Cannot find link [u*, v*]");
                }

                Point Ustar = Ustar_opt.value();
                Point Vstar = Vstar_opt.value();

                auto Ustar_it = std::find(tangent_polyline.begin(), tangent_polyline.end(), Ustar);
                size_t Ustar_idx = std::distance(tangent_polyline.begin(), Ustar_it);

                shortest_path.insert(shortest_path.end(), 
                                     tangent_polyline.begin(), 
                                     tangent_polyline.begin() + Ustar_idx + 1);

                auto Vstar_it = std::find(dual_polyline->begin(), dual_polyline->end(), Vstar);
                start_index = std::distance(dual_polyline->begin(), Vstar_it);
                checking_pt_index = Ustar_idx;
                curr_polyline = dual_polyline;
                direction = !direction;
                break;
            }

            // No intersection
            tangent_polyline.erase(tangent_polyline.begin() + left_tp_idx + 1, tangent_polyline.end());
            tangent_polyline.push_back(added_pt);
            
            if (added_pt == polyline_P.back()) {
                std::cout << "Reach goal" << std::endl;
                shortest_path.insert(shortest_path.end(), tangent_polyline.begin(), tangent_polyline.end());
                convex_hulls.push_back(tangent_polyline);
                std::cout << "Count original version: " << count << std::endl;
                return shortest_path;
            }
        }
        convex_hulls.push_back(tangent_polyline); // For illustration only
    }
}
// --- Struct SimplePolygon - END ---

SimplePolygonFromSequenceOfBundle::SimplePolygonFromSequenceOfBundle(const SequenceOfBundles& sequence)
    : sequence(sequence), start_direction(true) {
    
    std::vector<Point> polyline_P;
    std::vector<Point> polyline_Q;
    
    // Initialize with first skeleton point
    polyline_P.push_back(this->sequence.skeleton[0]);
    polyline_Q.push_back(this->sequence.skeleton[0]);
    
    bool vertex_on_P = true;
    partitions_of_P.push_back(-1);
    partitions_of_Q.push_back(-1);
    int label = 0;
    
    // Process intermediate skeleton points (excluding first and last)
    for (size_t i = 1; i < this->sequence.skeleton.size() - 1; ++i) {
        const Point& vertex = this->sequence.skeleton[i];
        
        // WARNING: Not degenerate bundles
        if (this->sequence.outer_endpoints[i].empty()) {
            polyline_P.push_back(vertex);
            polyline_Q.push_back(vertex);
            continue;
        }
        
        if (isLeft(this->sequence.skeleton[i-1], vertex, this->sequence.outer_endpoints[i][0])) {
            if (vertex_on_P) {
                ++label;
                vertex_on_P = false;
            }
            polyline_Q.push_back(vertex);
            partitions_of_Q.push_back(-1);
            for (const Point& outer_pt : this->sequence.outer_endpoints[i]) {
                polyline_P.push_back(outer_pt);
                partitions_of_P.push_back(label);
            }
        } else {
            if (!vertex_on_P) {
                ++label;
                vertex_on_P = true;
            }
            polyline_P.push_back(vertex);
            partitions_of_P.push_back(-1);
            for (const Point& outer_pt : this->sequence.outer_endpoints[i]) {
                polyline_Q.push_back(outer_pt);
                partitions_of_Q.push_back(label);
            }
        }
        
        if (i == 1) {
            start_direction = !vertex_on_P;
        }
    }
    
    // Add the last vertex
    polyline_P.push_back(this->sequence.skeleton.back());
    polyline_Q.push_back(this->sequence.skeleton.back());
    partitions_of_P.push_back(-1);
    partitions_of_Q.push_back(-1);
    
    // Initialize base class
    this->polyline_P = std::move(polyline_P);
    this->polyline_Q = std::move(polyline_Q);
}

// Constructor with move semantics
SimplePolygonFromSequenceOfBundle::SimplePolygonFromSequenceOfBundle(SequenceOfBundles&& sequence)
    : sequence(std::move(sequence)), start_direction(true) {
    
    std::vector<Point> polyline_P;
    std::vector<Point> polyline_Q;
    
    // Initialize with first skeleton point
    polyline_P.push_back(this->sequence.skeleton[0]);
    polyline_Q.push_back(this->sequence.skeleton[0]);
    
    bool vertex_on_P = true;
    partitions_of_P.push_back(-1);
    partitions_of_Q.push_back(-1);
    int label = 0;
    
    // Process intermediate skeleton points (excluding first and last)
    for (size_t i = 1; i < this->sequence.skeleton.size() - 1; ++i) {
        const Point& vertex = this->sequence.skeleton[i];
        
        // WARNING: Not degenerate bundles
        if (this->sequence.outer_endpoints[i].empty()) {
            polyline_P.push_back(vertex);
            polyline_Q.push_back(vertex);
            continue;
        }
        
        if (isLeft(this->sequence.skeleton[i-1], vertex, this->sequence.outer_endpoints[i][0])) {
            if (vertex_on_P) {
                ++label;
                vertex_on_P = false;
            }
            polyline_Q.push_back(vertex);
            partitions_of_Q.push_back(-1);
            for (const Point& outer_pt : this->sequence.outer_endpoints[i]) {
                polyline_P.push_back(outer_pt);
                partitions_of_P.push_back(label);
            }
        } else {
            if (!vertex_on_P) {
                ++label;
                vertex_on_P = true;
            }
            polyline_P.push_back(vertex);
            partitions_of_P.push_back(-1);
            for (const Point& outer_pt : this->sequence.outer_endpoints[i]) {
                polyline_Q.push_back(outer_pt);
                partitions_of_Q.push_back(label);
            }
        }
        
        if (i == 1) {
            start_direction = !vertex_on_P;
        }
    }
    
    // Add the last vertex
    polyline_P.push_back(this->sequence.skeleton.back());
    polyline_Q.push_back(this->sequence.skeleton.back());
    partitions_of_P.push_back(-1);
    partitions_of_Q.push_back(-1);
    
    // Initialize base class
    this->polyline_P = std::move(polyline_P);
    this->polyline_Q = std::move(polyline_Q);
}

std::vector<Point> SimplePolygonFromSequenceOfBundle::findShortestPath(std::optional<bool> direction) {
    bool dir = direction.has_value() ? direction.value() : start_direction;
    
    size_t count = 0;
    std::cout << "Improved version" << std::endl;
    std::vector<Point> shortest_path;
    size_t start_index = 0;
    size_t checking_pt_index = 0;
    
    const std::vector<Point>* curr_polyline = dir ? &polyline_P : &polyline_Q;
    const std::vector<int>* curr_partitions = dir ? &partitions_of_P : &partitions_of_Q;
    
    while (true) {
        const std::vector<Point>* dual_polyline = (curr_polyline == &polyline_P) ? &polyline_Q : &polyline_P;
        std::vector<Point> tangent_polyline;

        if ((*curr_polyline)[start_index] == polyline_P.back()) {
            shortest_path.push_back((*curr_polyline)[start_index]);
            return shortest_path;
        } else if (start_index + 1 < curr_polyline->size() && 
                  (*curr_polyline)[start_index + 1] == polyline_P.back()) {
            shortest_path.push_back((*curr_polyline)[start_index]);
            shortest_path.push_back((*curr_polyline)[start_index + 1]);
            return shortest_path;
        }

        // Initialize the tangent polyline
        tangent_polyline.push_back((*curr_polyline)[start_index]);
        tangent_polyline.push_back((*curr_polyline)[start_index + 1]);

        int starting_partition = (*curr_partitions)[start_index + 1];

        // Increment the convex hull
        for (size_t i = start_index + 2; i < curr_polyline->size(); ++i) {
            const Point& added_pt = (*curr_polyline)[i];

            // Find the tangent points
            size_t left_tp_idx = findLeftTangentPoint(tangent_polyline, added_pt, dir);
            
            int partition_of_added_pt = (*curr_partitions)[i];
            
            // Check intersection
            if (partition_of_added_pt != starting_partition || starting_partition == -1) {
                ++count; // test
                std::vector<Point> Ystar;
                bool intersection = false;
                
                for (size_t j = checking_pt_index; j < dual_polyline->size() - 1; ++j) {
                    const Point& prev_pt = (*dual_polyline)[j];
                    const Point& pt = (*dual_polyline)[j + 1];
                    
                    if (!isLeft(prev_pt, tangent_polyline[left_tp_idx], added_pt, dir) &&
                        isLeft(pt, tangent_polyline[left_tp_idx], added_pt, dir) &&
                        doIntersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt)) {
                        intersection = true;
                        Ystar.push_back(pt);
                    } else if (isLeft(pt, tangent_polyline[left_tp_idx], added_pt, dir) && intersection) {
                        Ystar.push_back(pt);
                    } else if (!isLeft(pt, tangent_polyline[left_tp_idx], added_pt, dir) &&
                              doIntersect(prev_pt, pt, tangent_polyline[left_tp_idx], added_pt)) {
                        intersection = false;
                    }
                }
                
                if (!Ystar.empty()) {
                    // Find link
                    std::vector<Point> Xstar(tangent_polyline.begin() + left_tp_idx, tangent_polyline.end());
                    auto [Ustar_opt, Vstar_opt] = findLink(Xstar, Ystar, dir);
                    
                    if (!Ustar_opt.has_value()) {
                        throw std::runtime_error("Cannot find link [u*, v*]");
                    }
                    
                    Point Ustar = Ustar_opt.value();
                    Point Vstar = Vstar_opt.value();
                    
                    auto Ustar_it = std::find(tangent_polyline.begin(), tangent_polyline.end(), Ustar);
                    size_t Ustar_idx = std::distance(tangent_polyline.begin(), Ustar_it);
                    
                    shortest_path.insert(shortest_path.end(), 
                                       tangent_polyline.begin(), 
                                       tangent_polyline.begin() + Ustar_idx + 1);
                    
                    auto Vstar_it = std::find(dual_polyline->begin(), dual_polyline->end(), Vstar);
                    start_index = std::distance(dual_polyline->begin(), Vstar_it);
                    checking_pt_index = Ustar_idx;
                    curr_polyline = dual_polyline;
                    curr_partitions = (curr_polyline == &polyline_P) ? &partitions_of_P : &partitions_of_Q;
                    dir = !dir;
                    break;
                }
            }
            
            // No intersection
            tangent_polyline.erase(tangent_polyline.begin() + left_tp_idx + 1, tangent_polyline.end());
            tangent_polyline.push_back(added_pt);
            
            if (added_pt == polyline_P.back()) {
                std::cout << "Reach goal" << std::endl;
                shortest_path.insert(shortest_path.end(), tangent_polyline.begin(), tangent_polyline.end());
                convex_hulls.push_back(tangent_polyline);
                std::cout << "Count improved version: " << count << std::endl;
                return shortest_path;
            }
        }
        convex_hulls.push_back(tangent_polyline); // For illustration only
    }
}