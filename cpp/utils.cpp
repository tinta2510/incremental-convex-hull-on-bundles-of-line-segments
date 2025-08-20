#include<utils.hpp>

#include <cmath>
#include <stdexcept>   // for std::invalid_argument


// --- Struct Point - START ---
Point::Point(float x, float y) : x(x), y(y) {}

bool Point::operator==(const Point& other) const {
    return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
}

bool Point::operator!=(const Point& other) const {
    return !(*this == other);
}

Point Point::operator+(const Point& other) const {
    return Point(x + other.x, y + other.y);
}

Point Point::operator-(const Point& other) const {
    return Point(x - other.x, y - other.y);
}

Point Point::operator*(float scalar) const {
    return Point(x * scalar, y * scalar);
}

Point Point::operator/(float scalar) const {
    if (std::abs(scalar) < EPSILON) {
        throw std::invalid_argument("Division by zero in Point operator/");
    }
    return Point(x / scalar, y / scalar);
}

float Point::magnitude() const {
    return std::sqrt(x * x + y * y);
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

bool isLeftOn(const Point& p0, const Point& p1, const Point& p, bool direction) {
    float cross_product = (p1.x - p0.x) * (p.y - p0.y) - (p.x - p0.x) * (p1.y - p0.y);
    float result = cross_product * (direction ? 1.0f : -1.0f);
    return result >= -EPSILON;
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

bool doIntersect(const Point& A, const Point& B, const Point& C, const Point& D) {
    /**
     * Returns true if segments AB and CD intersect.
     */
    int o1 = orientation(A, B, C);
    int o2 = orientation(A, B, D);
    int o3 = orientation(C, D, A);
    int o4 = orientation(C, D, B);

    // General case: opposite orientations
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    // Special case: Check collinear points and if they overlap
    if (o1 == 0 && onSegment(A, B, C)) return true;
    if (o2 == 0 && onSegment(A, B, D)) return true;
    if (o3 == 0 && onSegment(C, D, A)) return true;
    if (o4 == 0 && onSegment(C, D, B)) return true;

    return false;
}

int orientation(const Point& p, const Point& q, const Point& r) {
    /**
     * Determines the orientation of the triplet (p, q, r).
     * Returns:
     * 0 -> Collinear
     * 1 -> Clockwise
     * -1 -> Counterclockwise
     */
    float val = (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    if (std::abs(val) < EPSILON) {
        return 0;  // Collinear
    }
    return (val > 0) ? 1 : -1;  // Clockwise or Counterclockwise
}

bool onSegment(const Point& p, const Point& q, const Point& r) {
    /**
     * Checks if point r lies on segment pq (assuming collinear condition is met).
     */
    return (std::min(p.x, q.x) - EPSILON <= r.x && r.x <= std::max(p.x, q.x) + EPSILON) &&
           (std::min(p.y, q.y) - EPSILON <= r.y && r.y <= std::max(p.y, q.y) + EPSILON);
}

