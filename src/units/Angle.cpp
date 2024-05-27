#include "Angle.h"

Angle const Angle::ZERO {0.0, AngleUnits::DEGREES };

Angle const Angle::k0Degrees = Angle(0.0, AngleUnits::DEGREES);
Angle const Angle::k90Degrees = Angle(90.0, AngleUnits::DEGREES);
Angle const Angle::k180Degrees = Angle(180.0, AngleUnits::DEGREES);
Angle const Angle::k360Degrees = Angle(360.0, AngleUnits::DEGREES);

double const Angle::kMinDegrees = -360.0;
double const Angle::kMaxDegrees = 360.0;

//constructor
Angle::Angle(double val, Angle::AngleUnits units) : valueDegrees_(units == AngleUnits::RADIANS ? degreesFromRadians(val) : val) {}

//convert to degrees from radians
double Angle::degreesFromRadians(double degrees){
	return degrees * (k180Degrees.toDegrees() / M_PI);
}

//get radians from degrees
double Angle::radiansFromDegrees(double radians) {
	return radians * (M_PI / k180Degrees.toDegrees());
}

//returns degrees
double Angle::toDegrees() const {
	return valueDegrees_;
}

//returns the degrees converted to radians
double Angle::toRadians() const {
	return valueDegrees_ * (M_PI / k180Degrees.toDegrees());
}

//normalize the angle
void Angle::normalize() {
	while (valueDegrees_ < 0.0) 
		valueDegrees_ += kMaxDegrees;

	while (valueDegrees_ > kMaxDegrees)
		valueDegrees_ -= kMaxDegrees;
}

//defines the + operator
Angle Angle::operator + (Angle const & a) const {
	return Angle(valueDegrees_ + a.valueDegrees_, AngleUnits::DEGREES);
}

//defines the subtraction operator
Angle Angle::operator - (Angle const & a) const {
	return Angle(valueDegrees_ - a.valueDegrees_, AngleUnits::DEGREES);
}

//defines the equal operator
void Angle::operator = (Angle const & a) {
	valueDegrees_ = a.valueDegrees_;
}

//convert a bearing to cartesian angle
Angle Angle::bearingToCartesianAngle(Angle const * const bearing) {
	if (bearing->toDegrees() < 0.0) {
		return Angle(fabs(bearing->toDegrees()) + 90.0, Angle::AngleUnits::DEGREES);
	}
	else {
		Angle cartesianAngle = { 90.0 - bearing->toDegrees(), Angle::AngleUnits::DEGREES };
		cartesianAngle.normalize();
		return cartesianAngle;
	}
}

//defines the less than operator
bool Angle::operator < (Angle const & that) const {
	return valueDegrees_ < that.valueDegrees_;
}

//defines the greater than operator
bool Angle::operator > (Angle const & that) const {
	return valueDegrees_ > that.valueDegrees_;
}