#include "Vec2.h"

//constructor
Vec2::Vec2(Vec2 const & v) : x(v.x), y(v.y) {}

//constructor
Vec2::Vec2(double x, double y) : x(x), y(y) {}

//operators
Vec2 Vec2::operator + (Vec2 const & v) const {
	return add(v.x, v.y);
}

//different way to add up vectors using the change in x and y
Vec2 Vec2::add(double dx, double dy) const {
	return Vec2(x + dx, y + dy);
}

Vec2 Vec2::operator - (Vec2 const & v) const {
	return sub(v.x, v.y);
}

//opposite of add
Vec2 Vec2::sub(double dx, double dy) const {
	return Vec2(x - dx, y - dy);
}

Vec2 Vec2::operator * (Vec2 const & v) const {
	return mult(v.x, v.y);
}

//multiply a vector using the change in x and y
Vec2 Vec2::mult(double dx, double dy) const {
	return Vec2(x * dx, y * dy);
}

//get the length of a vector
double Vec2::len() const {
	return sqrt(x * x + y * y);
}

//normalize the vector
Vec2 Vec2::nor() const {
	double length = len();
	return Vec2(x / length, y / length);
}

void Vec2::operator = (Vec2 const & v) {
	x = v.x;
	y = v.y;
}