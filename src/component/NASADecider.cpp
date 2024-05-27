#include "NASADecider.h"

//constructor
NASADecider::NASADecider(Aircraft* thisAircraft, concurrency::concurrent_unordered_map<std::string, ResolutionConnection*>* connections) {
	thisAircraft_ = thisAircraft; activeConnections_ = connections; hasRA_ = false;
}

//deconstructor
NASADecider::~NASADecider() {}


void NASADecider::analyze(Aircraft* intruder) {
	thisAircraft_->lock.lock();
	thisAircraftAltitude_ = thisAircraft_->positionCurrent.altitude.toFeet();
	thisAircraft_->lock.unlock();

	//sub method in this class
	setSensitivityLevel();

	intruder->lock.lock();
	Aircraft intrCopy = *(intruder);
	intruder->lock.unlock();

	//create a connection object in this folder (custom object)
	ResolutionConnection* connection = (*activeConnections_)[intrCopy.id];

	//another sub method called in this class
	doCalculations(&intrCopy, connection);

	//creating a threat class object and setting equal to the output of this sub-method define bellow
	Aircraft::ThreatClassification threatClass = NASADecider::determineThreatClass(&intrCopy, connection);
	Sense mySense = tempSenseMap_[intrCopy.id];

	RecommendationRange green, red;

	//if threat class needs a resolution advisory then change the sense
	if (threatClass == Aircraft::ThreatClassification::RESOLUTION_ADVISORY) {
		connection->lock.lock();
		if (connection->currentSense == Sense::UPWARD || connection->currentSense == Sense::DOWNWARD) {
			mySense = connection->currentSense;
		} else if (tempSenseMap_[intrCopy.id] == Sense::UNKNOWN) {
			//computing a resolution if there is no resolution
			tempSenseMap_[intrCopy.id] = mySense = senseutil::senseFromInt(raSense(connection->userPosition.altitude.toFeet(),
				calculationsMap_[intruder->id].userVSpeed, intrCopy.positionCurrent.altitude.toFeet(), calculationsMap_[intruder->id].intrVSpeed,
				calculationsMap_[intruder->id].userVSpeed, calculationsMap_[intruder->id].userVAccel, calculationsMap_[intruder->id].deltaTime)); // CHECK TARGET VERTICAL SPEED PARAMETER!!! Flight plan perhaps????
			connection->sendSense(mySense);
		}
		connection->lock.unlock();

		//if the intruder id equals intruder id and has RA then generate a rectangle pair then set some colors
		//intruder provided and the raIntruderId_. equal the provided intruder and has a RA then generate a new one and set the colors
		if (raIntruderId_.compare(intruder->id) == 0 && hasRA_) {
			RecommendationRangePair recRange = getRecRangePair(mySense, calculationsMap_[intruder->id].userVvel, calculationsMap_[intruder->id].intrVvel, calculationsMap_[intruder->id].userPosition.altitude.toFeet(), intrCopy.positionCurrent.altitude.toFeet(), calculationsMap_[intruder->id].modTau);
			green = recRange.positive;
			red = recRange.negative;
			strictestRA_ = green;
			recommendationRangeLock.lock();
			positiveRecommendationRange = green;
			negativeRecommendationRange = red;
			recommendationRangeLock.unlock();
			//if the ra intruder does not equal the provided intruder id then
			//generating a new range rec pair and if they are not equal 
		} else if (hasRA_) {
			RecommendationRangePair recRange = getRecRangePair(mySense, calculationsMap_[intruder->id].userVvel, calculationsMap_[intruder->id].intrVvel, calculationsMap_[intruder->id].userPosition.altitude.toFeet(), intrCopy.positionCurrent.altitude.toFeet(), calculationsMap_[intruder->id].modTau);
			//internal method call comparing RA's if they are different then do the bellow code of setting colors
			if (compareRA(recRange.positive) > 0) {
				raIntruderId_ = intruder->id;
				green = recRange.positive;
				red = recRange.negative;
				strictestRA_ = green;
				recommendationRangeLock.lock();
				positiveRecommendationRange = green;
				negativeRecommendationRange = red;
				recommendationRangeLock.unlock();
			}
		//if the ra does not equal the intruder id and the does not have a RA
		//most common first run since at the start wont have a rec range pair 
		} else {
			//generate rec range pair
			RecommendationRangePair recRange = getRecRangePair(mySense, calculationsMap_[intruder->id].userVvel, calculationsMap_[intruder->id].intrVvel, calculationsMap_[intruder->id].userPosition.altitude.toFeet(), intrCopy.positionCurrent.altitude.toFeet(), calculationsMap_[intruder->id].modTau);
			//set has RA true
			hasRA_ = true;
			raIntruderId_ = intruder->id;
			green = recRange.positive;
			red = recRange.negative;
			strictestRA_ = green;
			recommendationRangeLock.lock();
			positiveRecommendationRange = green;
			negativeRecommendationRange = red;
			recommendationRangeLock.unlock();
		}
		
	//if the threat class in a  non threat then set the sense to unkown
	} else if (threatClass == Aircraft::ThreatClassification::NON_THREAT_TRAFFIC) {
		tempSenseMap_[intrCopy.id] = Sense::UNKNOWN;
		connection->lock.lock();
		connection->currentSense = Sense::UNKNOWN;
		connection->lock.unlock();
		//if it does not have a RA then set it to the corrects colors
		if (!hasRA_) {
			red.valid = false;
			green.valid = false;
			recommendationRangeLock.lock();
			positiveRecommendationRange = green;
			negativeRecommendationRange = red;
			recommendationRangeLock.unlock();
		}
	//if nots a RA but is not a non threat then run this block and if it does not have a Ra then set colors
	} else {
		if (!hasRA_) {
			recommendationRangeLock.lock();
			positiveRecommendationRange = green;
			negativeRecommendationRange = red;
			recommendationRangeLock.unlock();
		}
	}

	intruder->lock.lock();
	intruder->threatClassification = threatClass;
	intruder->lock.unlock();

}

//called in above method just determine if its a RA,TA or non-threat
Aircraft::ThreatClassification NASADecider::determineThreatClass(Aircraft* intrCopy, ResolutionConnection* conn) {

	Aircraft::ThreatClassification prevThreatClass = intrCopy->threatClassification;

	bool zthrFlag = calculationsMap_[intrCopy->id].altSepFt < zthr(intrCopy->id) ? true : false;

	Aircraft::ThreatClassification newThreatClass;
	// if within proximity range
	if (calculationsMap_[intrCopy->id].slantRangeNmi < 6 && abs(calculationsMap_[intrCopy->id].altSepFt) < 1200) {
		// if passes TA threshold
		if (calculationsMap_[intrCopy->id].closingSpeedKnots > 0
			&& (prevThreatClass >= Aircraft::ThreatClassification::TRAFFIC_ADVISORY
				|| (calculationsMap_[intrCopy->id].modTau < tau(intrCopy->id) && zthrFlag))) {
			taModMap_[intrCopy->id] = true;
			zthrFlag = calculationsMap_[intrCopy->id].altSepFt < zthr(intrCopy->id) ? true : false;
			// if passes RA threshold
			if (prevThreatClass == Aircraft::ThreatClassification::RESOLUTION_ADVISORY
				|| (calculationsMap_[intrCopy->id].modTau < tau(intrCopy->id) && zthrFlag)) {
				newThreatClass = Aircraft::ThreatClassification::RESOLUTION_ADVISORY;
				if (!hasRA_) {
					hasRA_ = true;
					raIntruderId_ = intrCopy->id;
				}
			} else {
				// did not pass RA threshold -- Traffic Advisory
				newThreatClass = Aircraft::ThreatClassification::TRAFFIC_ADVISORY;
				if (hasRA_ && intrCopy->id.compare(raIntruderId_) == 0) {
					hasRA_ = false;
					strictestRA_ = RecommendationRange();
				}
			}
		} else {
			// did not pass TA threshold -- just Proximity Traffic
			newThreatClass = Aircraft::ThreatClassification::PROXIMITY_INTRUDER_TRAFFIC;
			taModMap_[intrCopy->id] = false;
			if (hasRA_ && intrCopy->id.compare(raIntruderId_) == 0) {
				hasRA_ = false;
				strictestRA_ = RecommendationRange();
			}
		}
	} else {
		// is not within proximity range
		newThreatClass = Aircraft::ThreatClassification::NON_THREAT_TRAFFIC;
		if (hasRA_ && intrCopy->id.compare(raIntruderId_) == 0) {
			hasRA_ = false;
			strictestRA_ = RecommendationRange();
		}
	}

	return newThreatClass;
}

//this a repeat method in the Decider class
RecommendationRangePair NASADecider::getRecRangePair(Sense sense, double userVvelFtPerM, double intrVvelFtPerM, double userAltFt,
	double intrAltFt, double rangeTauS) {

	RecommendationRange positive, negative;

	//if in TA range
	if (sense != Sense::UNKNOWN && rangeTauS > 0.0) {
		double alimFt = getAlimFt(userAltFt);
		double intrProjectedAltAtCpa = intrAltFt + intrVvelFtPerM * (rangeTauS / 60.0);
		double userProjectedAltAtCpa = userAltFt + userVvelFtPerM * (rangeTauS / 60.0);
		double vsepAtCpaFt = abs(intrProjectedAltAtCpa - userProjectedAltAtCpa);

		// Corrective RA
		Velocity absoluteMinVvelToAchieveAlim = Velocity(getVvelForAlim(sense, userAltFt, vsepAtCpaFt, intrProjectedAltAtCpa, rangeTauS), Velocity::VelocityUnits::FEET_PER_MIN);

		if (sense == Sense::UPWARD) {
			// upward
			positive.maxVerticalSpeed = kMaxGaugeVerticalVelocity_;
			positive.minVerticalSpeed = absoluteMinVvelToAchieveAlim;
			negative.maxVerticalSpeed = absoluteMinVvelToAchieveAlim;
			negative.minVerticalSpeed = kMinGaugeVerticalVelocity_;
		} else {
			// downward
			negative.maxVerticalSpeed = kMaxGaugeVerticalVelocity_;
			negative.minVerticalSpeed = absoluteMinVvelToAchieveAlim;
			positive.maxVerticalSpeed = absoluteMinVvelToAchieveAlim;
			positive.minVerticalSpeed = kMinGaugeVerticalVelocity_;
		}

		positive.valid = true;
		negative.valid = true;
	} else {
		positive.valid = false;
		negative.valid = false;
	}

	return RecommendationRangePair{ positive, negative };
}

int NASADecider::compareRA(RecommendationRange intrRange) {
	double intrRAMagnitude = abs(intrRange.maxVerticalSpeed.toFeetPerMin()) - abs(intrRange.minVerticalSpeed.toFeetPerMin()) < 0 ? abs(intrRange.maxVerticalSpeed.toFeetPerMin()) : abs(intrRange.minVerticalSpeed.toFeetPerMin());
	double currentRAMagnitude = abs(strictestRA_.maxVerticalSpeed.toFeetPerMin()) - abs(strictestRA_.minVerticalSpeed.toFeetPerMin()) < 0 ? abs(strictestRA_.maxVerticalSpeed.toFeetPerMin()) : abs(strictestRA_.minVerticalSpeed.toFeetPerMin());
	if (intrRAMagnitude > currentRAMagnitude)
		return 1;
	else if (currentRAMagnitude > intrRAMagnitude)
		return -1;
	else
		return 0;
}


//really not sure same problem in normal decider
void NASADecider::setSensitivityLevel() {
	if (thisAircraftAltitude_ < 1000)
		sensitivityLevel_ = 2;
	else if (thisAircraftAltitude_ >= 1000 && thisAircraftAltitude_ < 2350)
		sensitivityLevel_ = 3;
	else if (thisAircraftAltitude_ >= 2350 && thisAircraftAltitude_ < 5000)
		sensitivityLevel_ = 4;
	else if (thisAircraftAltitude_ >= 5000 && thisAircraftAltitude_ < 10000)
		sensitivityLevel_ = 5;
	else if (thisAircraftAltitude_ >= 10000 && thisAircraftAltitude_ < 20000)
		sensitivityLevel_ = 6;
	else if (thisAircraftAltitude_ >= 20000 && thisAircraftAltitude_ < 42000)
		sensitivityLevel_ = 7;
	else
		sensitivityLevel_ = 0;
}

//what is tau? helper method
int NASADecider::tau(std::string id) {
	if (taModMap_[id])
		switch (sensitivityLevel_) {
		case 3: return 15;
		case 4: return 20;
		case 5: return 25;
		case 6: return 30;
		case 7: return 35;
		}
	else
		switch (sensitivityLevel_) {
		case 2: return 20;
		case 3: return 25;
		case 4: return 30;
		case 5: return 40;
		case 6: return 45;
		case 7: return 58;
		}
}

//what is alim sub method
int NASADecider::alim() {
	switch (sensitivityLevel_) {
	case 3: return 300;
	case 4: return 300;
	case 5: return 350;
	case 6: return 400;
	case 7: if (thisAircraftAltitude_ < 42000) return 600;
			else return 700;
	}
}

//sub method dependent on sensitivity Level
double NASADecider::dmod(std::string id) {
	if (taModMap_[id])
		switch (sensitivityLevel_) {
		case 3: return 0.20;
		case 4: return 0.35;
		case 5: return 0.55;
		case 6: return 0.80;
		case 7: return 1.10;
		}
	else
		switch (sensitivityLevel_) {
		case 2: return 0.30;
		case 3: return 0.33;
		case 4: return 0.48;
		case 5: return 0.75;
		case 6: return 1.00;
		case 7: return 1.30;
		}
}

//same as above comment
double NASADecider::hmd() {
	switch (sensitivityLevel_) {
	case 3: return 0.4;
	case 4: return 0.57;
	case 5: return 0.74;
	case 6: return 0.82;
	case 7: return 0.98;
	}
}

//same as above method
double NASADecider::zthr(std::string id) {
	if (taModMap_[id])
		switch (sensitivityLevel_) {
		case 3: return 600;
		case 4: return 600;
		case 5: return 600;
		case 6: return 600;
		case 7: if (thisAircraftAltitude_ < 42000) return 700;
				else return 800;
		}
	else
		switch (sensitivityLevel_) {
		case 2: return 850;
		case 3: return 850;
		case 4: return 850;
		case 5: return 850;
		case 6: return 850;
		case 7: if (thisAircraftAltitude_ < 42000) return 850;
				else return 1200;
		}
}

//getting Horizontal postion based of LLA and dist per degree at that LLA which is variable 
Vector2 NASADecider::getHorPos(LLA position) {
	return Vector2(position.distPerDegreeLat().toFeet(), position.distPerDegreeLon().toFeet());
}

//getting horizontal velocity  based of old and new LLA positions and change in time
Vector2 NASADecider::getHorVel(LLA position, LLA positionOld, double deltaTime) {
	return Vector2(position.range(&positionOld), position.bearing(&positionOld)).scalarMult(1 / deltaTime);
}

//get a vector from user to intruder
Vector2 NASADecider::getRelativePos(LLA userPos, LLA intrPos) {
	Distance d = userPos.range(&intrPos);
	Angle a = userPos.bearing(&intrPos);
	return Vector2(d, a);
}

//get a velocity vector based off the relative postion old and new
Vector2 NASADecider::getRelativeVel(Vector2 relativePos, Vector2 relativePosOld, double deltaTime) {
	return (relativePos - relativePosOld).scalarMult(1 / deltaTime);
}

//
void NASADecider::doCalculations(Aircraft* intrCopy, ResolutionConnection* conn) {
	
	//getting data concurency secure
	conn->lock.lock();
	calculationsMap_[intrCopy->id].userPosition = conn->userPosition;
	calculationsMap_[intrCopy->id].userPositionOld = conn->userPositionOld;
	std::chrono::milliseconds userPositionTime = conn->userPositionTime;
	std::chrono::milliseconds userPositionOldTime = conn->userPositionOldTime;
	conn->lock.unlock();

	//get relative pos/vel
	//using alot of the above methods
	calculationsMap_[intrCopy->id].userHorPos = getHorPos(calculationsMap_[intrCopy->id].userPosition);
	calculationsMap_[intrCopy->id].intrHorPos = getHorPos(intrCopy->positionCurrent);
	calculationsMap_[intrCopy->id].relativeHorPos = getRelativePos(calculationsMap_[intrCopy->id].userPosition, intrCopy->positionCurrent);
	calculationsMap_[intrCopy->id].relativeHorPosOld = getRelativePos(calculationsMap_[intrCopy->id].userPositionOld, intrCopy->positionOld);
	calculationsMap_[intrCopy->id].deltaTime = (double)(intrCopy->positionCurrentTime - intrCopy->positionOldTime).count() / 1000;
	calculationsMap_[intrCopy->id].userVSpeed = std::abs((calculationsMap_[intrCopy->id].userPosition.altitude.toFeet() - calculationsMap_[intrCopy->id].userPositionOld.altitude.toFeet()) / calculationsMap_[intrCopy->id].deltaTime);
	calculationsMap_[intrCopy->id].intrVSpeed = std::abs((intrCopy->positionCurrent.altitude.toFeet() - intrCopy->positionOld.altitude.toFeet()) / calculationsMap_[intrCopy->id].deltaTime);
	calculationsMap_[intrCopy->id].userHorVel = getHorVel(calculationsMap_[intrCopy->id].userPosition, calculationsMap_[intrCopy->id].userPositionOld, calculationsMap_[intrCopy->id].deltaTime);
	calculationsMap_[intrCopy->id].intrHorVel = getHorVel(intrCopy->positionCurrent, intrCopy->positionOld, calculationsMap_[intrCopy->id].deltaTime);
	calculationsMap_[intrCopy->id].relativeVel = getRelativeVel(calculationsMap_[intrCopy->id].relativeHorPos, calculationsMap_[intrCopy->id].relativeHorPosOld, calculationsMap_[intrCopy->id].deltaTime);
	calculationsMap_[intrCopy->id].modTau = tMod(intrCopy->id, calculationsMap_[intrCopy->id].relativeHorPos, calculationsMap_[intrCopy->id].relativeVel);

	calculationsMap_[intrCopy->id].slantRangeNmi = abs(calculationsMap_[intrCopy->id].userPosition.range(&intrCopy->positionCurrent).toNmi());
	calculationsMap_[intrCopy->id].deltaDistanceM = abs(calculationsMap_[intrCopy->id].userPositionOld.range(&intrCopy->positionOld).toMeters())
		- abs(calculationsMap_[intrCopy->id].userPosition.range(&intrCopy->positionCurrent).toMeters());
	calculationsMap_[intrCopy->id].closingSpeedKnots = Velocity(calculationsMap_[intrCopy->id].deltaDistanceM / calculationsMap_[intrCopy->id].deltaTime, Velocity::VelocityUnits::METERS_PER_S).toUnits(Velocity::VelocityUnits::KNOTS);
	calculationsMap_[intrCopy->id].altSepFt = abs(intrCopy->positionCurrent.altitude.toFeet() - calculationsMap_[intrCopy->id].userPosition.altitude.toFeet());

	double userDeltaAlt = calculationsMap_[intrCopy->id].userPosition.altitude.toFeet() - calculationsMap_[intrCopy->id].userPositionOld.altitude.toFeet();
	double intrDeltaAlt = intrCopy->positionCurrent.altitude.toFeet() - intrCopy->positionOld.altitude.toFeet();
	calculationsMap_[intrCopy->id].userVvelOld = calculationsMap_[intrCopy->id].userVvel;
	calculationsMap_[intrCopy->id].userVvel = userDeltaAlt / calculationsMap_[intrCopy->id].deltaTime;
	calculationsMap_[intrCopy->id].intrVvel = intrDeltaAlt / calculationsMap_[intrCopy->id].deltaTime;
	calculationsMap_[intrCopy->id].userVAccel = (calculationsMap_[intrCopy->id].userVvel - calculationsMap_[intrCopy->id].userVvelOld) / calculationsMap_[intrCopy->id].deltaTime;

}

double NASADecider::tCpa(Vector2 relativePosition, Vector2 relativeVelocity) {
	return -1 * (relativePosition.dotProduct(relativeVelocity) / std::pow(relativeVelocity.normalize(), 2));
}

double NASADecider::t(Vector2 relativePosition, Vector2 relativeVelocity) {
	return -1 * (std::pow(relativePosition.normalize(), 2) / relativePosition.dotProduct(relativeVelocity));
}

double NASADecider::tMod(std::string id, Vector2 relativePosition, Vector2 relativeVelocity) {
	return (((std::pow(dmod(id), 2)) - (relativePosition.normalize() * relativePosition.normalize())) / (relativePosition.dotProduct(relativeVelocity)));
}

bool NASADecider::horizontalRA(std::string id, Vector2 relativePosition, Vector2 relativeVelocity) {
	if (relativePosition.dotProduct(relativeVelocity) >= 0)
		return relativePosition.normalize() < dmod(id);
	else
		return tMod(id, relativePosition, relativeVelocity) <= tau(id);
}

double NASADecider::tCoa(double relativeAlt, double relativeVSpeed) {
	return -1 * (relativeAlt / relativeVSpeed);
}

bool NASADecider::verticalRA(std::string id, double relativeAlt, double relativeVSpeed) {
	if (relativeAlt * relativeVSpeed >= 0) {
		return std::abs(relativeAlt) < zthr(id);
	} else
		return tCoa(relativeAlt, relativeVSpeed) <= tau(id);
}

double NASADecider::delta(Vector2 relativePosition, Vector2 relativeVelocity, double minimumSeperationDistance) {
	return (std::pow(minimumSeperationDistance, 2) * std::pow(relativeVelocity.normalize(), 2)) - relativePosition.dotProduct(relativeVelocity.rightPerpendicular());
}

bool NASADecider::cd2d(Vector2 relativePosition, Vector2 relativeVelocity, double minimumSeperationDistance) {
	if (relativePosition.normalize() < minimumSeperationDistance)
		return true;
	else
		return delta(relativePosition, relativeVelocity, minimumSeperationDistance) > 0 && relativePosition.dotProduct(relativeVelocity) < 0;
}

bool NASADecider::tcasIIRa(std::string id, Vector2 userHorPos, double userAlt, Vector2 userHorVel, double userVSpeed, Vector2 intrHorPos, double intrAlt, Vector2 intrHorVel, double intrVSpeed) {
	Vector2 s = userHorPos - intrHorPos;
	Vector2 v = userHorVel - intrHorVel;
	double sz = userAlt - intrAlt;
	double vz = userVSpeed - intrVSpeed;
	if (!horizontalRA(id, s, v))
		return false;
	else if (!verticalRA(id, sz, vz))
		return false;
	else
		return cd2d(s, v, hmd());
}

bool NASADecider::tcasIIRaAt(std::string id, Vector2 userHorPos, double userAlt, Vector2 userHorVel, double userVSpeed, Vector2 intrHorPos, double intrAlt, Vector2 intrHorVel, double intrVSpeed, double deltaTime) {
	Vector2 s = userHorPos - intrHorPos;
	Vector2 v = userHorVel - intrHorVel;
	double sz = userAlt - intrAlt;
	double vz = userVSpeed - intrVSpeed;
	if (!horizontalRA(id, s + v.scalarMult(deltaTime), v))
		return false;
	else if (!verticalRA(id, sz + (deltaTime*vz), vz))
		return false;
	else
		return cd2d(s + v.scalarMult(deltaTime), v, hmd());
}

double NASADecider::timeMinTauMod(std::string id, Vector2 relativePosition, Vector2 relativeVelocity, double timeBoundStart, double timeBoundEnd) {
	if ((relativePosition + relativeVelocity.scalarMult(timeBoundStart)).dotProduct(relativeVelocity) >= 0) {
		return timeBoundStart;
	} else if (delta(relativePosition, relativeVelocity, dmod(id)) < 0) {
		double tmin = 2 * ((std::sqrt(-1 * delta(relativePosition, relativeVelocity, dmod(id))) / (std::pow(relativeVelocity.normalize(), 2)))); // (Formula 19)
		double min = timeBoundEnd < (tCpa(relativePosition, relativeVelocity) - (tmin / 2)) ? tmin : (tCpa(relativePosition, relativeVelocity) - (tmin / 2)); // next two lines (Formula 20)
		return timeBoundStart > min ? timeBoundStart : min;
	} else if ((relativePosition + relativeVelocity.scalarMult(timeBoundEnd)).dotProduct(relativeVelocity) < 0) {
		return timeBoundEnd;
	} else {
		double min = timeBoundEnd < tCpa(relativePosition, relativeVelocity) ? 0 : tCpa(relativePosition, relativeVelocity); // next two lines (Formula 20)
		return timeBoundStart > min ? timeBoundStart : min;
	}
}

bool NASADecider::ra2d(std::string id, Vector2 horizontalRelativePos, Vector2 horizontalRelativeVel, double lookaheadTimeStart, double lookaheadTimeEnd) {
	if (delta(horizontalRelativePos, horizontalRelativeVel, dmod(id)) >= 0 && (horizontalRelativePos + horizontalRelativeVel.scalarMult(lookaheadTimeStart)).magnitude() < 0 &&
		(horizontalRelativePos + horizontalRelativeVel.scalarMult(lookaheadTimeEnd)).magnitude() >= 0) return true;
	double t2 = timeMinTauMod(id, horizontalRelativePos, horizontalRelativeVel, lookaheadTimeStart, lookaheadTimeEnd);
	return horizontalRA(id, horizontalRelativePos + horizontalRelativeVel.scalarMult(t2), horizontalRelativeVel);
}

double* NASADecider::raTimeInterval(std::string id, double relativeAlt, double relativeVSpeed, double lookaheadTime) {
	double* returnVal = new double[2];
	if (relativeVSpeed == 0) {
		returnVal[0] = 0; returnVal[1] = lookaheadTime;
	} else {
		double h = zthr(id) > tau(id) * std::abs(relativeVSpeed) ? zthr(id) : tau(id) * std::abs(relativeVSpeed);
		returnVal[0] = (-1 * std::signbit(relativeVSpeed) * h) / relativeVSpeed;
		returnVal[1] = (std::signbit(relativeVSpeed) * h) / relativeVSpeed;
	}
	return returnVal;
}

bool NASADecider::ra3d(std::string id, Vector2 userHorizontalPos, double userAlt, Vector2 userHorizontalVel, double userVSpeed, Vector2 intrHorizontalPos, double intrAlt, Vector2 intrHorizontalVel,
	double intrVSpeed, double lookaheadTime) {
	Vector2 s = userHorizontalPos - intrHorizontalPos;
	Vector2 v = userHorizontalVel - intrHorizontalVel;
	double sz = userAlt - intrAlt;
	double vz = userVSpeed - intrVSpeed;
	if (!cd2d(s, v, hmd())) return false;
	if (vz == 0 && std::abs(sz) > zthr(id)) return false;
	double* tInOut = raTimeInterval(id, sz, vz, lookaheadTime);
	double tIn = tInOut[0]; double tOut = tInOut[1]; delete(tInOut);
	if (tIn < 0 || tOut > lookaheadTime) return false;
	return ra2d(id, s, v, tIn > 0 ? tIn : 0, lookaheadTime < tOut ? lookaheadTime : tOut);
}

double NASADecider::sepAt(double userAlt, double userVSpeed, double intrAlt, double intrVSpeed, double targetVSpeed, double userVAccel, int dir, double deltaTime) {
	double o = ownAltAt(userAlt, userVSpeed, std::abs(targetVSpeed), userVAccel, dir*std::signbit(targetVSpeed), deltaTime);
	double i = intrAlt + (deltaTime * intrVSpeed);
	return (dir * (o - i));
}

double NASADecider::ownAltAt(double userAlt, double userVSpeed, double targetVSpeed, double userVAccel, int dir, double deltaTime) {
	double s = stopAccel(userVSpeed, targetVSpeed, userVAccel, dir, deltaTime);
	double q = (deltaTime < s) ? deltaTime : s;
	double l = (deltaTime - s) > 0 ? (deltaTime - s) : 0;
	return (dir * std::pow(q, 2) * (userVAccel / 2) + (q * userVSpeed) + userAlt + (dir * l * targetVSpeed));
}

double NASADecider::stopAccel(double userVSpeed, double targetVSpeed, double userVAccel, int direction, double deltaTime) {
	if (deltaTime <= 0 || (direction * userVSpeed) >= targetVSpeed)
		return 0;
	else
		return ((direction*targetVSpeed) - userVSpeed) / (direction * userVAccel);
}

int NASADecider::raSense(double userAlt, double userVSpeed, double intrAlt, double intrVSpeed, double targetVSpeed, double userVAccel, double deltaTime) {
	double oUp = ownAltAt(userAlt, userVSpeed, targetVSpeed, userVAccel, 1, deltaTime);
	double oDown = ownAltAt(userAlt, userVSpeed, targetVSpeed, userVAccel, -1, deltaTime);
	double i = intrAlt + (deltaTime * intrVSpeed);
	double u = oUp - i;
	double d = i - oDown;
	if (userAlt - intrAlt > 0 && u >= alim())
		return 1;
	else if (userAlt - intrAlt < 0 && d >= alim())
		return -1;
	else if (u >= d)
		return 0; // Not sure is correct, blank in NASA doc
	else
		return -1;
}

bool NASADecider::corrective(std::string id, Vector2 userHorizontalPos, double userAlt, Vector2 userHorizontalVel, double userVSpeed, Vector2 intrHorizontalPos, double intrAlt, Vector2 intrHorizontalVel, double intrVSpeed, double targetVSpeed, double userVAccel) {
	Vector2 s = userHorizontalPos - intrHorizontalPos;
	Vector2 v = userHorizontalVel - intrHorizontalVel;
	double sz = userAlt - intrAlt;
	double vz = userVSpeed - intrVSpeed;
	double t = tMod(id, s, v);
	int dir = raSense(userAlt, userVSpeed, intrAlt, intrVSpeed, targetVSpeed, userVAccel, t);
	return (s.normalize() < dmod(id) || ((s.dotProduct(v) < 0) && (dir * (sz + (t * vz)) < alim())));
}