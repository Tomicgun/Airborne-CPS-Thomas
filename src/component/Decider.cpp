#include "Decider.h"


Velocity const Decider::kMinGaugeVerticalVelocity_ = { -4000.0, Velocity::VelocityUnits::FEET_PER_MIN };
Velocity const Decider::kMaxGaugeVerticalVelocity_ = { 4000.0, Velocity::VelocityUnits::FEET_PER_MIN };

Distance const Decider::kProtectionVolumeRadius_ = { 30.0, Distance::DistanceUnits::NMI };

Distance const Decider::kAlim350_ = { 350.0, Distance::DistanceUnits::FEET };
Distance const Decider::kAlim400_ = { 400.0, Distance::DistanceUnits::FEET };
Distance const Decider::kAlim600_ = { 600.0, Distance::DistanceUnits::FEET };
Distance const Decider::kAlim700_ = { 700.0, Distance::DistanceUnits::FEET };

Distance const Decider::kAltitudeAlim350Threshold_ = { 5000.0, Distance::DistanceUnits::FEET };
Distance const Decider::kAltitudeAlim400Threshold_ = { 10000.0, Distance::DistanceUnits::FEET };
Distance const Decider::kAltitudeAlim600Threshold_ = { 20000.0, Distance::DistanceUnits::FEET };

Velocity const Decider::kVerticalVelocityClimbDescendDelta_ = { 1500.0, Velocity::VelocityUnits::FEET_PER_MIN };

//constructor
Decider::Decider(Aircraft* thisAircraft, concurrency::concurrent_unordered_map<std::string, ResolutionConnection*>* map) : thisAircraft_(thisAircraft), activeConnections_(map) {}

//first class function that call determine action required
void Decider::analyze(Aircraft* intruder) {
	Decider::determineActionRequired(intruder);
}

//given a threat class return a string version of that threat class
std::string Decider::getThreatClassStr(Aircraft::ThreatClassification threatClass) {
	switch (threatClass) {
	case Aircraft::ThreatClassification::NON_THREAT_TRAFFIC:
		return "Non-Threat Traffic";
	case Aircraft::ThreatClassification::PROXIMITY_INTRUDER_TRAFFIC:
		return "Proximity Intruder Traffic";
	case Aircraft::ThreatClassification::TRAFFIC_ADVISORY:
		return "Traffic Advisory";
	case Aircraft::ThreatClassification::RESOLUTION_ADVISORY:
		return "Resolution Advisory";
	default:
		return "Unknown Threat Class";
	}
}

//determine to go up or down to resolve threat, called in the analyze method
void Decider::determineActionRequired(Aircraft* intruder) {
	intruder->lock.lock();
	Aircraft intrCopy = *(intruder);
	intruder->lock.unlock();

	ResolutionConnection* connection = (*activeConnections_)[intrCopy.id];
	Aircraft::ThreatClassification threatClass = Decider::determineThreatClass(&intrCopy, connection);
	Sense mySense = tempSense_;

	RecommendationRangePair recRange;

	//if determined threat class equals resolution advisory then determine resolution
	if (threatClass == Aircraft::ThreatClassification::RESOLUTION_ADVISORY) {
		connection->lock.lock();
		//if the sense already says to go up or down no reason to change
		if (connection->consensusAchieved && (connection->currentSense == Sense::UPWARD || connection->currentSense == Sense::DOWNWARD)) {
			mySense = connection->currentSense;
		//if sense is unkown or no solution to the RA is given then determine the sense
		} else if (tempSense_ == Sense::UNKNOWN) {
			tempSense_ = mySense = Decider::determineResolutionSense(connection->userPosition.altitude.toUnits(Distance::DistanceUnits::FEET),
				intrCopy.positionCurrent.altitude.toUnits(Distance::DistanceUnits::FEET));
			connection->sendSense(mySense);
		}
		connection->lock.unlock();

		//getting / determine data about the intruder and center of gauge
		double userDeltaPosM = connection->userPosition.range(&connection->userPositionOld).toMeters();
		double userDeltaAltM = connection->userPosition.altitude.toMeters() - connection->userPositionOld.altitude.toMeters();
		double intrDeltaPosM = intrCopy.positionCurrent.range(&intrCopy.positionOld).toMeters();
		double intrDeltaAltM = intrCopy.positionCurrent.altitude.toMeters() - intrCopy.positionOld.altitude.toMeters();
		double userElapsedTimeS = (double)(connection->userPositionTime - connection->userPositionOldTime).count() / 1000;
		double intrElapsedTimeS = (double)(intrCopy.positionCurrentTime - intrCopy.positionOldTime).count() / 1000;
		double slantRangeNmi = abs(connection->userPosition.range(&intrCopy.positionCurrent).toUnits(Distance::DistanceUnits::NMI));
		double deltaDistanceM = abs(connection->userPositionOld.range(&intrCopy.positionOld).toUnits(Distance::DistanceUnits::METERS))
			- abs(connection->userPosition.range(&intrCopy.positionCurrent).toUnits(Distance::DistanceUnits::METERS));
		double closingSpeedKnots = Velocity(deltaDistanceM / intrElapsedTimeS, Velocity::VelocityUnits::METERS_PER_S).toUnits(Velocity::VelocityUnits::KNOTS);
		Velocity userVvel = Velocity(userDeltaAltM / userElapsedTimeS, Velocity::VelocityUnits::METERS_PER_S);
		Velocity intrVvel = Velocity(intrDeltaAltM / intrElapsedTimeS, Velocity::VelocityUnits::METERS_PER_S);
		double rangeTauS = getModTauS(slantRangeNmi, closingSpeedKnots, getRADmodNmi(connection->userPosition.altitude.toFeet()));
		recRange = getRecRangePair(mySense, userVvel.toFeetPerMin(), intrVvel.toFeetPerMin(), connection->userPosition.altitude.toFeet(), intrCopy.positionCurrent.altitude.toFeet(), rangeTauS);

	//if its not a threat then ignore it
	} else if (threatClass == Aircraft::ThreatClassification::NON_THREAT_TRAFFIC) {
		tempSense_ = Sense::UNKNOWN;
		connection->lock.lock();
		connection->currentSense = Sense::UNKNOWN;
		connection->lock.unlock();
		recRange.negative.valid = false;
		recRange.positive.valid = false;
	}

	
	recommendationRangeLock.lock();
	positiveRecommendationRange = recRange.positive;
	negativeRecommendationRange = recRange.negative;
	recommendationRangeLock.unlock();

	intruder->lock.lock();
	intruder->threatClassification = threatClass;
	intruder->lock.unlock();
}

//called in determine resolution use to classify a threat
Aircraft::ThreatClassification Decider::determineThreatClass(Aircraft* intrCopy, ResolutionConnection* conn) {
	conn->lock.lock();

	//lla in units folder its a unit
	LLA userPosition = conn->userPosition;
	LLA userPositionOld = conn->userPositionOld;
	std::chrono::milliseconds userPositionTime = conn->userPositionTime;
	std::chrono::milliseconds userPositionOldTime = conn->userPositionOldTime;
	conn->lock.unlock();

	Aircraft::ThreatClassification prevThreatClass = intrCopy->threatClassification; //get previous threat class

	//get abs value of range between threat and user pos and convert it to units NMI
	double slantRangeNmi = abs(userPosition.range(&intrCopy->positionCurrent).toUnits(Distance::DistanceUnits::NMI));

	//get abs delta distance between user and threat almost velocity
	double deltaDistanceM = abs(userPositionOld.range(&intrCopy->positionOld).toUnits(Distance::DistanceUnits::METERS))
		- abs(userPosition.range(&intrCopy->positionCurrent).toUnits(Distance::DistanceUnits::METERS));

	//finding how much elapsed time has occurred
	double elapsedTimeS = (double)(intrCopy->positionCurrentTime - intrCopy->positionOldTime).count() / 1000;

	//taking change in distance divided by time giving us velocity which is then converted to knots
	double closingSpeedKnots = Velocity(deltaDistanceM / elapsedTimeS, Velocity::VelocityUnits::METERS_PER_S).toUnits(Velocity::VelocityUnits::KNOTS);

	//get difference of altitude in feet between target and user position
	double altSepFt = abs(intrCopy->positionCurrent.altitude.toUnits(Distance::DistanceUnits::FEET) -
		userPosition.altitude.toUnits(Distance::DistanceUnits::FEET));

	//get the delta of altitude in feet
	double deltaDistance2Ft = abs(intrCopy->positionOld.altitude.toUnits(Distance::DistanceUnits::FEET) -
		userPositionOld.altitude.toUnits(Distance::DistanceUnits::FEET)) -
		abs(intrCopy->positionCurrent.altitude.toUnits(Distance::DistanceUnits::FEET) -
			userPosition.altitude.toUnits(Distance::DistanceUnits::FEET));

	//get number of elapsed minutes
	double elapsedTimeMin = elapsedTimeS / 60;

	//get vertical velocity of intruder
	double vertClosingSpdFtM = deltaDistance2Ft / elapsedTimeMin;

	//time left before collision horizontal
	double rangeTauS = slantRangeNmi / closingSpeedKnots * 3600;

	//time left to collision vertical
	double verticalTauS = altSepFt / vertClosingSpdFtM * 60;

	//calculating and setting user velocity
	Velocity userVelocity = Velocity(userPosition.range(&userPositionOld).toMeters() / ((userPositionTime.count() - userPositionOldTime.count()) / 1000), Velocity::VelocityUnits::METERS_PER_S);
	
	//calculating and setting intruder velocity
	Velocity intrVelocity = Velocity(intrCopy->positionCurrent.range(&intrCopy->positionOld).toMeters() / ((intrCopy->positionCurrentTime.count() - intrCopy->positionOldTime.count()) / 1000), Velocity::VelocityUnits::METERS_PER_S);
	
	//setup two distance metrics one for user and intruder
	Distance userDistanceByCpa = Distance(userVelocity.toMetersPerS() * rangeTauS, Distance::DistanceUnits::METERS);
	Distance intrDistanceByCpa = Distance(intrVelocity.toMetersPerS() * rangeTauS, Distance::DistanceUnits::METERS);

	//setup two longitude latitude object based of intruder and user position
	LLA userPositionAtCpa = userPosition.translate(&userPositionOld.bearing(&userPosition), &userDistanceByCpa);
	LLA intrPositionAtCpa = intrCopy->positionCurrent.translate(&intrCopy->positionOld.bearing(&intrCopy->positionCurrent), &intrDistanceByCpa);
	
	//range between user and intruders cpa
	double distanceAtCpaFt = userPositionAtCpa.range(&intrPositionAtCpa).toFeet();

	//really not sure
	double taModTauS = getModTauS(slantRangeNmi, closingSpeedKnots, getTADmodNmi(userPosition.altitude.toFeet()));
	double raModTauS = getModTauS(slantRangeNmi, closingSpeedKnots, getRADmodNmi(userPosition.altitude.toFeet()));

	Aircraft::ThreatClassification newThreatClass;
	// if within proximity range
	if (slantRangeNmi < 6 && abs(altSepFt) < 1200) {
		// if passes TA threshold
		if (closingSpeedKnots > 0
			&& (prevThreatClass >= Aircraft::ThreatClassification::TRAFFIC_ADVISORY
			|| tauPassesTAThreshold(userPosition.altitude.toFeet(), taModTauS, verticalTauS, altSepFt))) {
			// if passes RA threshold
			if (prevThreatClass == Aircraft::ThreatClassification::RESOLUTION_ADVISORY
				|| tauPassesRAThreshold(userPosition.altitude.toFeet(), raModTauS, verticalTauS, altSepFt)) {
				newThreatClass = Aircraft::ThreatClassification::RESOLUTION_ADVISORY;
			} else {
				// did not pass RA threshold -- Traffic Advisory
				newThreatClass = Aircraft::ThreatClassification::TRAFFIC_ADVISORY;
			}
		} else {
			// did not pass TA threshold -- just Proximity Traffic
			newThreatClass = Aircraft::ThreatClassification::PROXIMITY_INTRUDER_TRAFFIC;
		}
	} else {
		// is not within proximity range
		newThreatClass = Aircraft::ThreatClassification::NON_THREAT_TRAFFIC;
	}

	return newThreatClass;
}

//state machine
//getting is TA has passed TA threshold 
bool Decider::tauPassesTAThreshold(double altFt, double modTauS, double vertTauS, double vSepFt)
{
	if (vSepFt > getTAZthrFt(altFt)) {
		if (altFt < 1000 && modTauS < 20 && vertTauS < 20)
			return true;
		else if (altFt < 2350 && modTauS < 25 && vertTauS < 25)
			return true;
		else if (altFt < 5000 && modTauS < 30 && vertTauS < 30)
			return true;
		else if (altFt < 10000 && modTauS < 40 && vertTauS < 40)
			return true;
		else if (altFt < 20000 && modTauS < 45 && vertTauS < 45)
			return true;
		else if (altFt >= 20000 && modTauS < 48 && vertTauS < 48)
			return true;
		else
			return false;
	} else {
		if (altFt < 1000 && modTauS < 20)
			return true;
		else if (altFt < 2350 && modTauS < 25)
			return true;
		else if (altFt < 5000 && modTauS < 30)
			return true;
		else if (altFt < 10000 && modTauS < 40)
			return true;
		else if (altFt < 20000 && modTauS < 45)
			return true;
		else if (altFt >= 20000 && modTauS < 48)
			return true;
		else
			return false;
	}
}
//checking if TA has passed RA threshold
//state machine
bool Decider::tauPassesRAThreshold(double altFt, double modTauS, double vertTauS, double vSepFt)
{
	if (vSepFt > getRAZthrFt(altFt)) {
		if (altFt < 1000)
			return false;
		else if (altFt < 2350 && modTauS < 15 && vertTauS < 15)
			return true;
		else if (altFt < 5000 && modTauS < 20 && vertTauS < 20)
			return true;
		else if (altFt < 10000 && modTauS < 25 && vertTauS < 25)
			return true;
		else if (altFt < 20000 && modTauS < 30 && vertTauS < 30)
			return true;
		else if (altFt >= 20000 && modTauS < 35 && vertTauS < 35)
			return true;
		else
			return false;
	} else {
		if (altFt < 1000)
			return false;
		else if (altFt < 2350 && modTauS < 15)
			return true;
		else if (altFt < 5000 && modTauS < 20)
			return true;
		else if (altFt < 10000 && modTauS < 25)
			return true;
		else if (altFt < 20000 && modTauS < 30)
			return true;
		else if (altFt >= 20000 && modTauS < 35)
			return true;
		else
			return false;
	}
}

//determine whether to go up or down based if the user is above or below the intruder
//state machine
Sense Decider::determineResolutionSense(double userAltFt, double intrAltFt) {
	if (userAltFt > intrAltFt)
		return Sense::UPWARD;
	else
		return Sense::DOWNWARD;
}

//not sure should ask bastion
int Decider::getAlimFt(double altFt) {
	if (altFt < 1000)
		return -1;
	else if (altFt < 20000)
		return 600;
	else if (altFt < 42000)
		return 700;
	else
		return 800;
}

//getting RA threshold in feat on the z axis
int Decider::getRAZthrFt(double altFt) {
	if (altFt < 1000)
		return -1;
	else if (altFt < 5000)
		return 300;
	else if (altFt < 10000)
		return 350;
	else if (altFt < 20000)
		return 400;
	else if (altFt < 42000)
		return 600;
	else
		return 700;
}

//get TA threshold on the z axis in feet
int Decider::getTAZthrFt(double altFt) {
	if (altFt < 42000)
		return 850;
	else
		return 1200;
}
//really not sre TODO:figure it out
//something in relation to RA and NMI
double Decider::getRADmodNmi(double altFt) {
	if (altFt < 1000)
		return 0;
	else if (altFt < 2350)
		return .2;
	else if (altFt < 5000)
		return .35;
	else if (altFt < 10000)
		return .55;
	else if (altFt < 20000)
		return .8;
	else
		return 1.1;
}
//something in realtion to RA and NMI
double Decider::getTADmodNmi(double altFt) {
	if (altFt < 1000)
		return .3;
	else if (altFt < 2350)
		return .33;
	else if (altFt < 5000)
		return .48;
	else if (altFt < 10000)
		return .75;
	else if (altFt < 20000)
		return 1.0;
	else
		return 1.3;
}

//getting some measurement 
double Decider::getModTauS(double rangeNmi, double closureRateKnots, double dmodNmi) {
	return ((pow(rangeNmi, 2) - pow(dmodNmi, 2)) / (rangeNmi * closureRateKnots)) * 3600;
}

//TODO:document this method
RecommendationRangePair Decider::getRecRangePair(Sense sense, double userVvelFtPerM, double intrVvelFtPerM, double userAltFt,
	double intrAltFt, double rangeTauS) {

	RecommendationRange positive, negative;

	//making sure sense and rangeTAUs have correct values
	if (sense != Sense::UNKNOWN && rangeTauS > 0.0) {
		double alimFt = getAlimFt(userAltFt);
		double intrProjectedAltAtCpa = intrAltFt + intrVvelFtPerM * (rangeTauS / 60.0);
		double userProjectedAltAtCpa = userAltFt + userVvelFtPerM * (rangeTauS / 60.0);
		double vsepAtCpaFt = abs(intrProjectedAltAtCpa - userProjectedAltAtCpa);

		/*if (vsep_at_cpa_ft < alim_ft) {*/
			// Corrective RA
			Velocity absoluteMinVvelToAchieveAlim = Velocity(getVvelForAlim(sense, userAltFt, vsepAtCpaFt, intrProjectedAltAtCpa, rangeTauS), Velocity::VelocityUnits::FEET_PER_MIN);
			char toPrint[100];
			sprintf(toPrint, "result f/m = %f\n", absoluteMinVvelToAchieveAlim.toFeetPerMin());
			XPLMDebugString(toPrint);

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
		
		//} else {
		//	// Preventative RA
		//	char toPrint[100];
		//	sprintf(toPrint, "Preventative RA: user_vvel_ft_m = %f, sense = %s\n", user_vvel_ft_m, senseToString(sense));
		//	XPLMDebugString(toPrint);
		//	if (sense == Sense::UPWARD) {
		//		// upward
		//		positive.max_vertical_speed = Velocity(user_vvel_ft_m + 500, Velocity::VelocityUnits::FEET_PER_MIN);
		//		positive.min_vertical_speed = Velocity(user_vvel_ft_m, Velocity::VelocityUnits::FEET_PER_MIN);
		//		negative.max_vertical_speed = Velocity(user_vvel_ft_m, Velocity::VelocityUnits::FEET_PER_MIN);
		//		negative.min_vertical_speed = kMinGaugeVerticalVelocity;
		//	} else {
		//		// downward
		//		negative.max_vertical_speed = kMaxGaugeVerticalVelocity;
		//		negative.min_vertical_speed = Velocity(user_vvel_ft_m, Velocity::VelocityUnits::FEET_PER_MIN);
		//		positive.max_vertical_speed = Velocity(user_vvel_ft_m, Velocity::VelocityUnits::FEET_PER_MIN);
		//		positive.min_vertical_speed = Velocity(user_vvel_ft_m - 500, Velocity::VelocityUnits::FEET_PER_MIN);
		//	}
		//}
		positive.valid = true;
		negative.valid = true;
	} else {
		positive.valid = false;
		negative.valid = false;
	}

	char toPrint[100];
	sprintf(toPrint, "Recommended Max = %f f/m, Recommended Min = %f f/m, Danger Max = %f f/m, Danger Min = %f f/m\n", positive.maxVerticalSpeed.toFeetPerMin(), positive.minVerticalSpeed.toFeetPerMin(), negative.maxVerticalSpeed.toFeetPerMin(), negative.minVerticalSpeed.toFeetPerMin());
	XPLMDebugString(toPrint);

	return RecommendationRangePair{ positive, negative };
}

//TODO:document this method
double Decider::getVvelForAlim(Sense sense, double altFt, double vsepAtCpaFt, double intrProjAltFt, double rangeTauS) {
	/*char toPrint[1000];
	sprintf(toPrint, "Sense = %s, alt_ft = %f, vsep_at_cpa_ft = %f, intr_proj_alt_ft = %f, range_tau = %f\n", senseToString(sense), altFt, vsep_at_cpa_ft, intr_proj_alt_ft, range_tau_s);
	XPLMDebugString(toPrint);*/

	double toReturn;
	if (sense == Sense::UPWARD) {
		toReturn = (getAlimFt(altFt) - vsepAtCpaFt) / (rangeTauS / 60);
		if (toReturn > kMaxGaugeVerticalVelocity_.toFeetPerMin()- 500)
			toReturn = kMaxGaugeVerticalVelocity_.toFeetPerMin() - 500;
	} else if (sense == Sense::DOWNWARD) {
		toReturn = -(getAlimFt(altFt) - vsepAtCpaFt) / (rangeTauS / 60);
		if (toReturn < kMinGaugeVerticalVelocity_.toFeetPerMin() + 500)
			toReturn = kMinGaugeVerticalVelocity_.toFeetPerMin() + 500;
	} else
		toReturn = 0;

	return toReturn;
}