#include <camera_cardetection/CameraCardetection.h>

#include "ros/ros.h"
#include "car_msgs/DetectedSignals.h"
#include "std_msgs/Float64.h"
#include "vrep_common/VisionSensorData.h"
#include "error_seeder/ErrorSeederLib.h"
#include "SystemConfig.h"
#include <time.h>
#include <stdio.h>

#include <exception>

using namespace std;
using namespace camera_cardetection;

// Main Object
CameraCardetection* cameraCardetection;

// Publisher
ros::Publisher* camValuesPublisher;

// Global variables:
int robotId = 0;

// Constants
const string CAM_TYPE_CAR = "car";

void sendBlobInformations(std::vector<float> sensorInformations) {

	if (sensorInformations.size() > 0) { // in "sensorInformations" we should have the blob information if the camera was set-up correctly
		int blobCount = sensorInformations[15];
		int dataSizePerBlob = sensorInformations[16];
		if (blobCount == 0) {
			// no blob found => can't find a car to follow
			car_msgs::DetectedSignals signalMsg;

			signalMsg.robotId = robotId;
			signalMsg.camType = CAM_TYPE_CAR;
			signalMsg.blobsize = 0.0;
			signalMsg.blobOrientation = 0.0;
			signalMsg.blobPosX = 0.0;
			signalMsg.blobPosY = 0.0;
			signalMsg.blobBoxDimensionX = 0.0;
			signalMsg.blobBoxDimensionY = 0.0;

			camValuesPublisher->publish(signalMsg);
		} else {
			// Now we get the data from the car before this one
			for (int i = 1; i <= blobCount; i++) {
				float blobSize = sensorInformations[16+(i-1)*dataSizePerBlob+1];
				float blobOrientation = sensorInformations[16+(i-1)*dataSizePerBlob+2];
				float blobPos[] = {sensorInformations[16+(i-1)*dataSizePerBlob+3],sensorInformations[16+(i-1)*dataSizePerBlob+4]};
				float blobBoxDimensions[] = {sensorInformations[16+(i-1)*dataSizePerBlob+5],sensorInformations[16+(i-1)*dataSizePerBlob+6]};

				if (blobBoxDimensions[0]>blobBoxDimensions[1]) {
					float tmp = blobBoxDimensions[0];
					blobBoxDimensions[0] = blobBoxDimensions[1];
					blobBoxDimensions[1] = tmp;
					blobOrientation = blobOrientation + M_PI/2;
				}

				car_msgs::DetectedSignals signalMsg;

				signalMsg.robotId = robotId;
				signalMsg.camType = CAM_TYPE_CAR;
				signalMsg.blobsize = blobSize;
				signalMsg.blobOrientation = blobOrientation;
				signalMsg.blobPosX = blobPos[0];
				signalMsg.blobPosY = blobPos[1];
				signalMsg.blobBoxDimensionX = blobBoxDimensions[0];
				signalMsg.blobBoxDimensionY = blobBoxDimensions[1];

				camValuesPublisher->publish(signalMsg);
			}
		}
	}
} // end of sendBlobInformations function

void carDetectionCallback(const vrep_common::VisionSensorData::ConstPtr& msg) {
	std_msgs::Float32MultiArray array = msg->packetData;
	std::vector<float> data = array.data;
	sendBlobInformations(data);
}

// Main code:
int main(int argc,char* argv[]) {
	bool robotIdByArg = false;
	bool useRobotIdInTopic = false;
	string help =
			"CameraCardetection\n"
			"Synobsis: camera_cardetection_node OPTIONS\n"
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

	cout << "start camera_cardetection" << endl;
	//bring the lib in
	cameraCardetection = new CameraCardetection();

	//ros::init(argc, argv, "camera_cardetection_node");
	//ros::NodeHandle n;

	if (!robotIdByArg) {
		robotId = supplementary::SystemConfig::GetOwnRobotID();
	}

	string nodeName;
	stringstream sss;
	sss << "camera_cardetection_node__" << robotId;
	sss >> nodeName;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ROS_INFO("own robot Id: %d\n", robotId);

	error_seeder::ErrorSeederLib esl(compId);

	// build topic name
	string camValuesPubTopic;
	string carSubTopic;
	stringstream ss;
	if (useRobotIdInTopic) {
		ss << "/vrep/carSim/detectedSignals";
		ss >> camValuesPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/carDetectionData";
		ss >> carSubTopic;
	} else {
		camValuesPubTopic = "/vrep/carSim/blobValues";
		carSubTopic = "/vrep/carSim/carDetectionData";
	}

	ros::Publisher blobValuesPub = n.advertise<car_msgs::DetectedSignals>(camValuesPubTopic, 1);
	camValuesPublisher = &blobValuesPub;

	ros::Subscriber carDetectSub = n.subscribe(carSubTopic, 1, carDetectionCallback);

	try {
		ros::Rate pub_rate(100.0);

		while (ros::ok()) {
			ros::spinOnce();

			cout << "car_cam spin" << endl;

			pub_rate.sleep();
		}
	} catch (exception& e) {
		ROS_FATAL("camera_cardetection_node caught exception. Aborting. %s", e.what());
		ROS_BREAK();
	}
}
