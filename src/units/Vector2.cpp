#include "units\Vector2.h"

//constructor
Vector2::Vector2() { x = 0; y = 0; }

//constructor
Vector2::Vector2(double xin, double yin) {
	x = xin; y = yin;
}

//constructor
Vector2::Vector2(Distance d, Angle a) {
	x = std::cos(a.toRadians()) * d.toFeet();
	y = std::sin(a.toRadians()) * d.toFeet();
}

//deconstructor
Vector2::~Vector2() {}

//dot product
double Vector2::dotProduct(Vector2 v2) {
	return (x * v2.x) + (y * v2.y);
}
//normalize
double Vector2::normalize() {
	return std::sqrt(dotProduct(*this));
}

//operators 
Vector2 Vector2::operator - (Vector2 const & v) const {
	return Vector2(x - v.x, y - v.y);
}

//scalar multiply
Vector2 Vector2::scalarMult(double const & d) const {
	return Vector2(d * x, d * y);
}

Vector2 Vector2::operator + (Vector2 const & v) const {
	return Vector2(x + v.x, y + v.y);
}

//another version of length
double Vector2::magnitude() const {
	return sqrt(x * x + y * y);
}

//getting the right perpendicular of a vector 
Vector2 Vector2::rightPerpendicular() {
	return Vector2(y, -1 * x);
}