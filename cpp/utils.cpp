#include<utils.hpp>

#include <cmath>
#include <stdexcept>   // for std::invalid_argument


// --- Struct Point - START ---
Point::Point(float x, float y) : x(x), y(y) {}
bool Point::operator==(const Point& other) const {
    return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
}
// --- Struct Point - END ---

float calculateDistance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

bool isLeft(const Point& p0, const Point& p1, const Point& p2, bool direction = true) {
    float crossProduct = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    if (direction) {
        return crossProduct > EPSILON; // Left turn
    } else {
        return crossProduct < EPSILON; // Right turn
    }
}

bool isEqual(float a, float b) {
    return std::abs(a - b) < EPSILON;
}

float calculateAngle(const Point& p1, const Point& p2, const Point& p3) {
    float BA_x = p1.x - p2.x;
    float BA_y = p1.y - p2.y;
    float BC_x = p3.x - p2.x;
    float BC_y = p3.y - p2.y;

    // Dot product
    float dot_product = BA_x * BC_x + BA_y * BC_y;

    // Magnitudes
    float magnitude_AB = std::sqrt(BA_x * BA_x + BA_y * BA_y);
    float magnitude_BC = std::sqrt(BC_x * BC_x + BC_y * BC_y);

    // Avoid division by zero
    if (magnitude_AB == 0 || magnitude_BC == 0) {
        throw std::invalid_argument("One of the vectors has zero length.");
    }

    // Cosine of the angle
    float cos_theta = dot_product / (magnitude_AB * magnitude_BC);

    // Ensure the value is within the valid range for acos
    cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));

    // Calculate angle in radians
    float angle_radians = std::acos(cos_theta);
    return angle_radians;
}