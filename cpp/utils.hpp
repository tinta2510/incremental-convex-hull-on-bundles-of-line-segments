#ifndef UTILS_HPP
#define UTILS_HPP

const float EPSILON = 1e-6;

struct Point {
    float x, y;
    Point(float, float);

    bool operator==(const Point&) const;
    bool operator!=(const Point&) const;

    Point operator+(const Point&) const;
    Point operator-(const Point&) const;
    Point operator*(float) const;
    Point operator/(float) const;

    float magnitude() const;
};

float calculateDistance(const Point&, const Point&);

bool isLeft(const Point&, const Point&, const Point&, bool);

bool isLeftOn(const Point& p0, const Point& p1, const Point& p, bool direction);

bool isEqual(float, float);

float calculateAngle(const Point&, const Point&, const Point&);

bool doIntersect(const Point&, const Point&, const Point&, const Point&);

int orientation(const Point&, const Point&, const Point&);

bool onSegment(const Point&, const Point&, const Point&);

#endif // UTILS_HPP