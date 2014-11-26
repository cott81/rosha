#include <car_motion/CarMotion.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/timer.hpp>
#include <math.h>
#include <time.h>
#include "ros/ros.h"
#include "car_msgs/DetectedSignals.h"
#include "car_msgs/DetectedDistance.h"
#include "car_msgs/DriveCmd.h"
#include "car_msgs/DrivingSpeed.h"
//#include "car_msgs/ProximityDistance.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "vrep_common/VisionSensorData.h"
#include "error_seeder/ErrorSeederLib.h"
#include "SystemConfig.h"
//#include "v_repLib.h"

#include <exception>

using namespace std;
using namespace car_motion;

// Main Object
CarMotion* carMotion;

// Publisher
ros::Publisher* leftMotorPub;
ros::Publisher* rightMotorPub;
ros::Publisher* leftSteerPub;
ros::Publisher* rightSteerPub;
ros::Publisher* drivingSpeedPub;

// Messages
std_msgs::Float64 leftMotorMsg;
std_msgs::Float64 rightMotorMsg;
std_msgs::Float64 leftSteerMsg;
std_msgs::Float64 rightSteerMsg;

// Global variables:
int robotId = 0;
long refreshTimeInMS = 400;
float desiredSteeringAngle = 0.0; // Remove? Probably not needed
float standardBlobPosRight = 0.0;
float standardBlobPosLeft = 0.0;
float standardBlobPos = 0.0;
bool foundStandardBlobPos = false;
bool avoidingMode = false;
bool onRightSide = true;
bool blobDetected = false;

bool changedSides = false;

// Proximity Values
float flDist = 0.0;
float fmDist = 0.0;
float frDist = 0.0;
float rlDist = 0.0;
float rmDist = 0.0;
float rrDist = 0.0;

// ProximityTS
boost::posix_time::ptime flTS;
boost::posix_time::ptime fmTS;
boost::posix_time::ptime frTS;
boost::posix_time::ptime rlTS;
boost::posix_time::ptime rmTS;
boost::posix_time::ptime rrTS;

// Constants
bool isLeader = false;
float stdBlobPosDifference = 0.6;
float steeringAngleDx = 2 * M_PI / 180;
float d = 1.05575; // 2*d=distance between left and right wheels
float l = 1.5133; // l=distance between front and read wheels

float oldTS = 0.0;

void timeCallback(const std_msgs::Float64::ConstPtr& msg) {
	if (robotId == 0) {
		oldTS = msg->data;
//	cout << "OLD_TS: " << oldTS << endl;
	}
}

void driveInDirection(float desiredSteeringAngle, float drivingSpeed) {
	// publish driving speed
	leftMotorMsg.data = drivingSpeed;
	rightMotorMsg.data = drivingSpeed;
	leftMotorPub->publish(leftMotorMsg);
	rightMotorPub->publish(rightMotorMsg);

	// calculate steering angle
	if (desiredSteeringAngle > 45 * M_PI / 180) {
		desiredSteeringAngle = 45 * M_PI / 180;
	} else if (desiredSteeringAngle < -45 * M_PI / 180) {
		desiredSteeringAngle = -45 * M_PI / 180;
	}

	// We handle the front left and right wheel steerings (Ackermann steering):
	float steeringAngleLeft = atan(l / (-d + l / tan(desiredSteeringAngle)));
	float steeringAngleRight = atan(l / (d + l / tan(desiredSteeringAngle)));

	// Publish steering data
	leftSteerMsg.data = steeringAngleLeft;
	rightSteerMsg.data = steeringAngleRight;
	leftSteerPub->publish(leftSteerMsg);
	rightSteerPub->publish(rightSteerMsg);

//	if (robotId == 0) {
//		clock_t t;
//		t = clock();
//		float timeDiff = t - oldTS;
//		float clocks_per_sec = CLOCKS_PER_SEC;
//		cout << "CLOCKS: " << clocks_per_sec << endl;
//		float realDiff = (((float) timeDiff) / CLOCKS_PER_SEC) * 1000;
//		cout << "Needed time: " << realDiff << endl;
//	}
} // end of driveInDirection function

void driveOnRoad(float blobPosX, float drivingSpeed) {
	float currentDirection = (blobPosX - standardBlobPos) * steeringAngleDx * (-100);
	driveInDirection(currentDirection, drivingSpeed);
} // end of driveOnRoad function

void motionCallback(const car_msgs::DetectedSignals::ConstPtr& msg) {
	float blobSize = msg->blobsize;
	float blobOrientation = msg->blobOrientation;
	float blobPosX = msg->blobPosX;
	float blobPosY = msg->blobPosY;
	float blobDimensionX = msg->blobBoxDimensionX;
	float blobDimensionY = msg->blobBoxDimensionY;

	if (blobSize > 0) {
		blobDetected = true;
	} else {
		blobDetected = false;
	}

	// check if the proximity values are still up-to-date
//	boost::posix_time::ptime currentTS = boost::posix_time::microsec_clock::local_time();
//	boost::posix_time::time_duration flDiff = currentTS - flTS;
//	boost::posix_time::time_duration fmDiff = currentTS - fmTS;
//	boost::posix_time::time_duration frDiff = currentTS - frTS;
//	boost::posix_time::time_duration rlDiff = currentTS - rlTS;
//	boost::posix_time::time_duration rmDiff = currentTS - rmTS;
//	boost::posix_time::time_duration rrDiff = currentTS - rrTS;

	float speed = 10;
	float driveNow = 0.0;

	boost::posix_time::ptime ts =
			boost::posix_time::microsec_clock::local_time();

//	cout << "flDiff: " << flDiff.total_milliseconds() << endl;
//	cout << "fmDiff: " << fmDiff.total_milliseconds() << endl;
//	cout << "frDiff: " << frDiff.total_milliseconds() << endl;
//	cout << "rlDiff: " << rlDiff.total_milliseconds() << endl;
//	cout << "rmDiff: " << rmDiff.total_milliseconds() << endl;
//	cout << "rrDiff: " << rrDiff.total_milliseconds() << endl;

//	if (flDiff.total_milliseconds() >= refreshTimeInMS) {
//		flDist = 0.0;
//	}
//	if (fmDiff.total_milliseconds() >= refreshTimeInMS) {
//		fmDist = 0.0;
//	}
//	if (frDiff.total_milliseconds() >= refreshTimeInMS) {
//		frDist = 0.0;
//	}
//	if (rlDiff.total_milliseconds() >= refreshTimeInMS) {
//		rlDist = 0.0;
//	}
//	if (rmDiff.total_milliseconds() >= refreshTimeInMS) {
//		rmDist = 0.0;
//	}
//	if (rrDiff.total_milliseconds() >= refreshTimeInMS) {
//		rrDist = 0.0;
//	}

//	cout << "flDist: " << flDist << endl;
//	cout << "fmDist: " << fmDist << endl;
//	cout << "frDist: " << frDist << endl;
//	cout << "rlDist: " << rlDist << endl;
//	cout << "rmDist: " << rmDist << endl;
//	cout << "rrDist: " << rrDist << endl;

	if (isLeader) {
		// switch to 'avoiding mode' if the proximity sensors detect any obstacle
//		if (((fr == 0) && (fm == 0) && (rr == 0) && (rm == 0))
//				|| ((rr > 0) && (fm > 0) && (onRightSide == false))
//				|| ((fr == 0) && (fm > 0) && (onRightSide == false))
//				|| ((fr > 0) && (fm > 0) && (rr == 0) && (onRightSide == false))
//				|| /*((rrDist~=nil) &&*/(rrDist > 1.0) /*&& (frDist==nil)*/) {
		if (((frDist == 0) && (fmDist == 0) && (rrDist == 0) /*&& (rmDist == 0)*/)
				|| ((rrDist > 0) && (fmDist > 0) && (onRightSide == false))
				|| ((frDist == 0) && (fmDist > 0) && (onRightSide == false))
				|| ((frDist > 0) && (fmDist > 0) && (rrDist == 0)
						&& (onRightSide == false))
				|| ((rrDist > 1.0) && (frDist == 0))) {
//			cout << "avoid off" << endl;
			avoidingMode = false;
		} else {
//			cout << "avoid on" << endl;
			avoidingMode = true;
		}
		if (blobDetected == false) {
			if (onRightSide == true) {
				driveInDirection(1, speed);
			} else {
				driveInDirection(-1, speed);
			}
//			cout << "blob NOT detected" << endl;
		} else if ((blobPosX <= 0.50) && (avoidingMode == true)) {
			if (((frDist != 0.0) && (frDist <= 1.0))
					|| ((fmDist != 0.0) && (fmDist <= 1.0))
					|| ((flDist != 0.0) && (flDist <= 0.6))) {

				driveNow = 1.0;
			} else if (((frDist != 0.0) && (frDist <= 1.8))
					|| ((fmDist != 0.0) && (fmDist <= 1.8))
					|| ((flDist != 0.0) && (flDist <= 1.2))) {

				driveNow = 0.5;
			} else if ((frDist != 0.0) && (fmDist != 0.0)) {
				driveNow = (0.2);
			} else {
				driveNow = (0.35);
			}
//			cout << "avoiding!" << endl;
			driveInDirection(driveNow, speed);
		} else if ((blobPosX >= 0.50) && (avoidingMode == false)) {
			if (((frDist != 0.0) && (frDist <= 1.0))
					|| ((fmDist != 0.0) && (fmDist <= 1.0))
					|| ((flDist != 0.0) && (flDist <= 0.6))) {

				driveNow = -1.0;
			} else if (((frDist != 0.0) && (frDist <= 1.8))
					|| ((fmDist != 0.0) && (fmDist <= 1.8))
					|| ((flDist != 0.0) && (flDist <= 1.2))) {

				driveNow = -0.5;
			} else if ((frDist != 0) && (fmDist != 0)) {
				driveNow = (-0.2);
			} else {
				driveNow = (-0.35);
			}
//			cout << "avoiding back!" << endl;
			driveInDirection(driveNow, speed);
		} else if ((blobPosX <= 0.40) && (avoidingMode == false)) {
			if (standardBlobPosRight == 0.0) {
				standardBlobPosRight = blobPosX;
			}
			standardBlobPos = standardBlobPosRight;
			onRightSide = true;
//			cout << "driving right side" << endl;
			driveOnRoad(blobPosX, speed);
		} else if ((blobPosX >= 0.60) && (avoidingMode == true)) {
			if (standardBlobPosLeft == 0.0) {
				standardBlobPosLeft = standardBlobPosRight
						+ stdBlobPosDifference;
			}
			standardBlobPos = standardBlobPosLeft;
			onRightSide = false;
//			cout << "driving wrong side" << endl;
			driveOnRoad(blobPosX, speed);
		}
	} else { // else of 'isLeader'
//		cout << "follow car" << endl;
		cout << "blobX: " << blobPosX << endl;
		if (blobDetected == true) {
			if ((fmDist != 0.0) && (fmDist < 1.5)) {
				speed = speed - 5;
			}
			if ((frDist != 0.0) && (frDist < 1.2)) {
				standardBlobPos = 0.8;
			} else {
				standardBlobPos = 0.5;
			}
			driveOnRoad(blobPosX, speed);
		}
	} // end of 'isLeader'
}

//void distanceCallback(const car_msgs::ProximityDistance::ConstPtr& msg) {
//	string sensor = msg->sensor;
//	float distance = msg->distance;
//	boost::posix_time::ptime ts = boost::posix_time::microsec_clock::local_time();
//
//	cout << "DistanceCallback: " << sensor << endl;
//	if (sensor == "frontLeft") {
//		flDist = distance;
//		flTS = ts;
//	} else if ( sensor == "frontMiddle") {
//		fmDist = distance;
//		fmTS = ts;
//	} else if ( sensor == "frontRight") {
//		frDist = distance;
//		frTS = ts;
//	} else if ( sensor == "rearLeft") {
//		rlDist = distance;
//		rlTS = ts;
//	} else if ( sensor == "rearMiddle") {
//		rmDist = distance;
//		rmTS = ts;
//	} else if ( sensor == "rearRight") {
//		rrDist = distance;
//		rrTS = ts;
//	}
//}

void distanceCallback(const car_msgs::DetectedDistance::ConstPtr& msg) {
	flDist = msg->flDist;
	fmDist = msg->fmDist;
	frDist = msg->frDist;
	rlDist = msg->rlDist;
	rmDist = msg->rmDist;
	rrDist = msg->rrDist;
}

void driveCallback(const car_msgs::DriveCmd::ConstPtr& msg) {

	if (msg->robotId != robotId) {
		return;
	}

	// Get data and publish to vrep simulator
	leftMotorMsg.data = msg->drivingSpeed;
	rightMotorMsg.data = msg->drivingSpeed;
	leftSteerMsg.data = msg->steerLeft;
	rightSteerMsg.data = msg->steerRight;

	leftMotorPub->publish(leftMotorMsg);
	rightMotorPub->publish(rightMotorMsg);
	leftSteerPub->publish(leftSteerMsg);
	rightSteerPub->publish(rightSteerMsg);

}

void drivingSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
	car_msgs::DrivingSpeed speedMsg;
	speedMsg.robotId = robotId;
	speedMsg.speed = msg->data;
	drivingSpeedPub->publish(speedMsg);
}

// Main code:
int main(int argc, char* argv[]) {
	bool robotIdByArg = false;
	bool useRobotIdInTopic = false;
	string help =
			"CarMotion\n"
					"Synobsis: car_motion_node OPTIONS\n"
					"Options:\n\n"
					"ROS params: paramter for ROS, check the ROS wiki for more details.\n"
					"-help: prints this help text\n"
					"-compId: specifies the Id of this component. (Used for failure simulation). Default is -1.\n"
					"-useRobotIdInTopic: code the robotId in the topic. E.g. /vrep/carSim12/blobDetectionData instead of /vrep/carSim/blobDetectionData. Default is false. \n"
					"-robotId: specifies the Id of this system/robot. If not set, the config file (Global.conf, in the configuration path) is used. \n";

	string helpParam = "-help";
	string compIdParam = "-compId";
	string robotIdParam = "-robotId";
	string useRobotIdInTopicParam = "-useRobotIdInTopic";
	int compId = -1;
	for (int i = 1; i < argc; i++) {
		cout << argv[i] << " ";
		if (helpParam.compare(argv[i]) == 0) {
			cout << help << endl;
			exit(0);
		} else if (compIdParam.compare(argv[i]) == 0) {
			compId = atoi(argv[i + 1]);
		} else if (robotIdParam.compare(argv[i]) == 0) {
			robotId = atoi(argv[i + 1]);
			robotIdByArg = true;
		} else if (useRobotIdInTopicParam.compare(argv[i]) == 0) {
			useRobotIdInTopic = true;
		} else {
			//ros arguments ...
		}
	}

	cout << "start car_motion" << endl;
	//bring the lib in
	carMotion = new CarMotion();

	//ros::init(argc, argv, "car_motion_node");
	//ros::NodeHandle n;

//	string nodeName = "CarMotion" + boost::lexical_cast<std::string>(robotId);
//	n.remapName(nodeName);

	if (!robotIdByArg) {
		robotId = supplementary::SystemConfig::GetOwnRobotID();
	}

	string nodeName;
	stringstream sss;
	sss << "car_motion_node__" << robotId;
	sss >> nodeName;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ROS_INFO("own robot Id: %d\n", robotId);

	error_seeder::ErrorSeederLib esl(compId);

	// build topic name
	string leftMotorPubTopic;
	string rightMotorPubTopic;
	string leftSteerPubTopic;
	string rightSteerPubTopic;
//	string leftMotorSubTopic;
//	string rightMotorSubTopic;
//	string leftSteerSubTopic;
//	string rightSteerSubTopic;
	string drivingSpeedSubTopic;
//	string blobSubTopic;
//	string distanceSubTopic;
	stringstream ss;
	if (useRobotIdInTopic) {
		ss << "/vrep/carSim" << robotId << "/velocityLeft";
		ss >> leftMotorPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/velocityRight";
		ss >> rightMotorPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/steerLeft";
		ss >> leftSteerPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/steerRight";
		ss >> rightSteerPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/steerRight";
		ss >> rightSteerPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/drivingSpeed_sim";
		ss >> drivingSpeedSubTopic;

//		ss.str("");
//		ss.clear();
//		ss << "/vrep/carSim" << robotId << "/blobValues";
//		ss >> blobSubTopic;
//
//		ss.str("");
//		ss.clear();
//		ss << "/vrep/carSim" << robotId << "/detectedDistance";
//		ss >> distanceSubTopic;
	} else {
		leftMotorPubTopic = "/vrep/carSim/velocityLeft";
		rightMotorPubTopic = "/vrep/carSim/velocityRight";
		leftSteerPubTopic = "/vrep/carSim/steerLeft";
		rightSteerPubTopic = "/vrep/carSim/steerRight";
		drivingSpeedSubTopic = "/vrep/carSim/drivingSpeed_sim";
//		blobSubTopic = "/vrep/carSim/blobValues";
//		distanceSubTopic = "/vrep/carSim/detectedDistance";
	}

	if (robotId == 0) {
		isLeader = true;
	}

//	if (isLeader == false) {
//		refreshTimeInMS = 360;
//	}

	ros::Publisher leftMotorPublisher = n.advertise<std_msgs::Float64>(leftMotorPubTopic, 1);
	leftMotorPub = &leftMotorPublisher;
	ros::Publisher rightMotorPublisher = n.advertise<std_msgs::Float64>(rightMotorPubTopic, 1);
	rightMotorPub = &rightMotorPublisher;
	ros::Publisher leftSteerPublisher = n.advertise<std_msgs::Float64>(leftSteerPubTopic, 1);
	leftSteerPub = &leftSteerPublisher;
	ros::Publisher rightSteerPublisher = n.advertise<std_msgs::Float64>(rightSteerPubTopic, 1);
	rightSteerPub = &rightSteerPublisher;
	ros::Publisher drivingSpeedPublisher = n.advertise<car_msgs::DrivingSpeed>("/vrep/carSim/drivingSpeed_motion", 1);
	drivingSpeedPub = &drivingSpeedPublisher;

//	ros::Subscriber blobDetectSub = n.subscribe(blobSubTopic, 1, motionCallback);
//	ros::Subscriber distanceSub = n.subscribe(distanceSubTopic, 1, distanceCallback);

	ros::Subscriber driveCmdSub = n.subscribe("/vrep/carSim/driveCmd", 1, driveCallback);
	ros::Subscriber drivingSpeedSub = n.subscribe(drivingSpeedSubTopic, 1, drivingSpeedCallback);

	// node time test
//	if (robotId == 0) {
//		ros::Subscriber timeSub = n.subscribe("/vrep/carSim0/startTime", 1, timeCallback);
//	}

	try {
		ros::Rate pub_rate(100.0);

		while (ros::ok()) {
			ros::spinOnce();

			pub_rate.sleep();
		}
	} catch (exception& e) {
		ROS_FATAL("car_motion_node caught exception. Aborting. %s", e.what());
		ROS_BREAK();
	}
}
