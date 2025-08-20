#ifndef UTILS_HPP
#define UTILS_HPP

const float EPSILON = 1e-6;

struct Point {
    float x, y;
    Point(float, float);
    bool operator==(const Point&) const;
};

float calculateDistance(const Point&, const Point&);

bool isLeft(const Point&, const Point&, const Point&, bool);

bool isEqual(float, float);

float calculateAngle(const Point&, const Point&, const Point&);

#endif // UTILS_HPP