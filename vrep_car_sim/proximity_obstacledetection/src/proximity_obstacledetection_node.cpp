#include <proximity_obstacledetection/ProximityObstacledetection.h>

#include "ros/ros.h"
#include "car_msgs/DetectedDistance.h"
#include "car_msgs/ProximityData.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include "vrep_common/ProximitySensorData.h"
//#include "error_seeder/ErrorSeederLib.h"
#include "SystemConfig.h"
//#include "v_repLib.h"
 #include <queue>

#include <exception>

using namespace std;
using namespace proximity_obstacledetection;

// Main Object
ProximityObstacledetection* proximityOD;

// Publisher
ros::Publisher* distancePublisher;
ros::Publisher* frontLeftPublisher;
ros::Publisher* frontMidPublisher;
ros::Publisher* frontRightPublisher;
ros::Publisher* rearLeftPublisher;
ros::Publisher* rearMidPublisher;
ros::Publisher* rearRightPublisher;

// Data queues
queue <float> flQue;
queue <float> fmQue;
queue <float> frQue;
queue <float> rlQue;
queue <float> rmQue;
queue <float> rrQue;

// Global variables:
int robotId = 0;

//float calculateDistance(geometry_msgs::Point32 detectedPoint) {
//	float x = detectedPoint.x;
//	float y = detectedPoint.y;
//	float z = detectedPoint.z;
//	float sum = pow(x, 2) + pow(y, 2) + pow(z, 2);
//
//	return sqrt(sum);
//} // end of calculateDistance function
//
//void frontLeftCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "frontLeft";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}
//
//void frontMiddleCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "frontMiddle";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}
//
//void frontRightCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "frontRight";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}
//
//void rearLeftCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "rearLeft";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}
//
//void rearMiddleCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "rearMiddle";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}
//
//void rearRightCallback(const vrep_common::ProximitySensorData::ConstPtr& msg) {
//	float distance = calculateDistance(msg->detectedPoint);
//	car_msgs::ProximityDistance proxMsg;
//
//	proxMsg.sensor = "rearRight";
//	proxMsg.distance = distance;
//
//	distancePublisher->publish(proxMsg);
//}

void publish() {
	if (flQue.size() > 0 && fmQue.size() > 0 && frQue.size() > 0
			&& rlQue.size() > 0 && rmQue.size() > 0 && rrQue.size() > 0) {

		car_msgs::DetectedDistance distMsg;

		distMsg.flDist = flQue.front();
		distMsg.fmDist = fmQue.front();
		distMsg.frDist = frQue.front();
		distMsg.rlDist = rlQue.front();
		distMsg.rmDist = rmQue.front();
		distMsg.rrDist = rrQue.front();

		distancePublisher->publish(distMsg);

		cout << "flsize0: " << flQue.size() << endl;
		cout << "fmsize0: " << fmQue.size() << endl;
		cout << "frsize0: " << frQue.size() << endl;
		cout << "rlsize0: " << rlQue.size() << endl;
		cout << "rmsize0: " << rmQue.size() << endl;
		cout << "rrsize0: " << rrQue.size() << endl;
		cout << endl;

		flQue.pop();
		fmQue.pop();
		frQue.pop();
		rlQue.pop();
		rmQue.pop();
		rrQue.pop();

		cout << "flsize1: " << flQue.size() << endl;
		cout << "fmsize1: " << fmQue.size() << endl;
		cout << "frsize1: " << frQue.size() << endl;
		cout << "rlsize1: " << rlQue.size() << endl;
		cout << "rmsize1: " << rmQue.size() << endl;
		cout << "rrsize1: " << rrQue.size() << endl;
		cout << endl;
	}
}

void frontLeftCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "FL!" << endl;
//	proxMsg.sensor = "frontLeft";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);
//	cout << "FL_CALLBACK: " << flQue.size() << endl;
//
//	flQue.push(msg->data);
//	publish();
	car_msgs::ProximityData proximityMsg;

	proximityMsg.robotId = robotId;
	proximityMsg.distance = msg->data;
	frontLeftPublisher->publish(proximityMsg);
}

void frontMiddleCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "FM!" << endl;
//	proxMsg.sensor = "frontMiddle";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);

	cout << "FM_CALLBACK: " << fmQue.size() << endl;

	fmQue.push(msg->data);
	publish();
}

void frontRightCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "FR!" << endl;
//	proxMsg.sensor = "frontRight";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);


	cout << "FR_CALLBACK: " << frQue.size() << endl;

	frQue.push(msg->data);
	publish();
}

void rearLeftCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "RL!" << endl;
//	proxMsg.sensor = "rearLeft";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);

	cout << "RL_CALLBACK: " << rlQue.size() << endl;

	rlQue.push(msg->data);
	publish();
}

void rearMiddleCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "RM!" << endl;
//	proxMsg.sensor = "rearMiddle";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);

	cout << "RM_CALLBACK: " << rmQue.size() << endl;

	rmQue.push(msg->data);
	publish();
}

void rearRightCallback(const std_msgs::Float32::ConstPtr& msg) {
//	car_msgs::ProximityDistance proxMsg;
//
//	cout << "RR!" << endl;
//	proxMsg.sensor = "rearRight";
//	proxMsg.distance = msg->data;
//
//	distancePublisher->publish(proxMsg);
	cout << "RR_CALLBACK: " << rrQue.size() << endl;

	rrQue.push(msg->data);
	publish();
}

// Main code:
int main(int argc,char* argv[]) {
	bool robotIdByArg = false;
	bool useRobotIdInTopic = false;
	string help =
			"ProximityObstacledetection\n"
			"Synobsis: proximity_obstacledetection_node OPTIONS\n"
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

	cout << "start proximity_obstacledetection" << endl;
	//bring the lib in
	proximityOD = new ProximityObstacledetection();

	ros::init(argc, argv, "proximity_obstacledetection_node");
	ros::NodeHandle n;

	if (!robotIdByArg) {
		robotId = supplementary::SystemConfig::GetOwnRobotID();
	}
	ROS_INFO("own robot Id: %d\n", robotId);

//	error_seeder::ErrorSeederLib esl(compId);

	// build topic name
	string distancePubTopic;
	string proxSubTopic;
	stringstream ss;
	if (useRobotIdInTopic) {
		ss << "/vrep/carSim" << robotId << "/detectedDistance";
		ss >> distancePubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/";
		ss >> proxSubTopic;
	} else {
		distancePubTopic = "/vrep/carSim/detectedDistance";
		proxSubTopic = "/vrep/carSim/";
	}

	ros::Publisher distancePub = n.advertise<car_msgs::DetectedDistance>(distancePubTopic, 1);
	distancePublisher = &distancePub;

	ros::Publisher frontLeftPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/frontLeftProx", 1);
	frontLeftPublisher = &frontLeftPub;

	ros::Publisher frontMidPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/frontMidProx", 1);
	frontLeftPublisher = &frontMidPub;

	ros::Publisher frontRightPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/frontRightProx", 1);
	frontLeftPublisher = &frontRightPub;

	ros::Publisher rearLeftPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/rearLeftProx", 1);
	frontLeftPublisher = &rearLeftPub;

	ros::Publisher rearMidPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/rearMidProx", 1);
	frontLeftPublisher = &rearMidPub;

	ros::Publisher rearRightPub = n.advertise<car_msgs::ProximityData>("/vrep/carSim/rearRightProx", 1);
	frontLeftPublisher = &rearRightPub;

	ros::Subscriber frontLeftSub = n.subscribe(proxSubTopic + "frontLeftProx", 1, frontLeftCallback);
	ros::Subscriber frontMiddleSub = n.subscribe(proxSubTopic + "frontMiddleProx", 1, frontMiddleCallback);
	ros::Subscriber frontRightSub = n.subscribe(proxSubTopic + "frontRightProx", 1, frontRightCallback);
	ros::Subscriber rearLeftSub = n.subscribe(proxSubTopic + "rearLeftProx", 1, rearLeftCallback);
	ros::Subscriber rearMiddleSub = n.subscribe(proxSubTopic + "rearMiddleProx", 1, rearMiddleCallback);
	ros::Subscriber rearRightSub = n.subscribe(proxSubTopic + "rearRightProx", 1, rearRightCallback);


//	ros::Publisher testPub = n.advertise<std_msgs::Float64>("/carsim//velocityLeft", 1);
//	testPublisher = &testPub;
//	ros::Publisher testPub2 = n.advertise<std_msgs::Float64>("/carsim/velocityRight", 1);
//	testPublisher2 = &testPub2;

	try {
		ros::Rate pub_rate(100.0);

		while (ros::ok()) {
			ros::spinOnce();

			pub_rate.sleep();
		}
	} catch (exception& e) {
		ROS_FATAL("proximity_obstacledetection_node caught exception. Aborting. %s", e.what());
		ROS_BREAK();
	}
}
