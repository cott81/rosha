#include <camera_blobdetection/CameraBlobdetection.h>

#include "ros/ros.h"
#include "car_msgs/DetectedSignals.h"
#include "std_msgs/Float64.h"
#include "vrep_common/VisionSensorData.h"
#include "error_seeder/ErrorSeederLib.h"
#include "SystemConfig.h"
//#include "v_repLib.h"
 #include <time.h>
 #include <stdio.h>

#include <exception>

using namespace std;
using namespace camera_blobdetection;

// Main Object
CameraBlobdetection* cameraBlobdetection;

// Publisher
//ros::Publisher* timePub;
ros::Publisher* camValuesPublisher;

// Global variables:
int robotId = 0;

// Constants
const string CAM_TYPE_STREET = "street";
const string CAM_TYPE_CAR = "car";

void sendBlobInformations(std::vector<float> sensorInformations, string camType) {

	if (sensorInformations.size() > 0) { // in "sensorInformations" we should have the blob information if the camera was set-up correctly
		int blobCount = sensorInformations[15];
		int dataSizePerBlob = sensorInformations[16];
		if (blobCount == 0) {
			// no blob found => can't find a car to follow
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
				signalMsg.camType = camType;
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


void streetDetectionCallback(const vrep_common::VisionSensorData::ConstPtr& msg) {
//	if (robotId == 0) {
//		std_msgs::Float64 timeMsg;
//
//		clock_t t;
//		t = clock();
//		float time = (float) t;
//
//		timeMsg.data = time;
//		timePub->publish(timeMsg);
//	}

	std_msgs::Float32MultiArray array = msg->packetData;
	std::vector<float> data = array.data;
	sendBlobInformations(data, CAM_TYPE_STREET);
}

void carDetectionCallback(const vrep_common::VisionSensorData::ConstPtr& msg) {
	std_msgs::Float32MultiArray array = msg->packetData;
	std::vector<float> data = array.data;
	sendBlobInformations(data, CAM_TYPE_CAR);
}

//void infoCallback(const vrep_common::VrepInfo::ConstPtr& info) {
//	simulationTime=info->simulationTime.data;
//	simulationRunning=(info->simulatorState.data&1)!=0;
////	cout << "infoDetect" << endl;
////	cout << simulationTime << endl;
////	cout << simulationRunning << endl;
////	simulationTime=simGetSimulationTime();
////	cout << simulationTime << endl;
//}

// Main code:
int main(int argc,char* argv[]) {
	bool robotIdByArg = false;
	bool useRobotIdInTopic = false;
	string help =
			"CameraBlobdetection\n"
			"Synobsis: camera_blobdetection_node OPTIONS\n"
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

	cout << "start camera_blobdetection" << endl;
	//bring the lib in
	cameraBlobdetection = new CameraBlobdetection();

	//ros::init(argc, argv, "camera_blobdetection_node");
	//ros::NodeHandle n;

	if (!robotIdByArg) {
		robotId = supplementary::SystemConfig::GetOwnRobotID();
	}

	string nodeName;
	stringstream sss;
	sss << "camera_blobdetection_node__" << robotId;
	sss >> nodeName;
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ROS_INFO("own robot Id: %d\n", robotId);

	error_seeder::ErrorSeederLib esl(compId);

	// build topic name
	string camValuesPubTopic;
	string streetSubTopic;
	string carSubTopic;
	stringstream ss;
	if (useRobotIdInTopic) {
//		ss << "/vrep/carSim" << robotId << "/blobValues";
		ss << "/vrep/carSim/detectedSignals";
		ss >> camValuesPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/streetDetectionData";
		ss >> streetSubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/carDetectionData";
		ss >> carSubTopic;
	} else {
		camValuesPubTopic = "/vrep/carSim/blobValues";
		streetSubTopic = "/vrep/carSim/streetDetectionData";
		carSubTopic = "/vrep/carSim/carDetectionData";
	}

	ros::Publisher blobValuesPub = n.advertise<car_msgs::DetectedSignals>(camValuesPubTopic, 1);
	camValuesPublisher = &blobValuesPub;

	// node time test
//	if (robotId == 0) {
//		ros::Publisher timePublisher = n.advertise<std_msgs::Float64>("/vrep/carSim0/startTime", 1);
//		timePub = &timePublisher;
//	}

//	time_t now = time(0);
//	long nowts = now;
//	cout << "nowTS !!!: " << nowts << endl;

	ros::Subscriber streetDetectSub = n.subscribe(streetSubTopic, 1, streetDetectionCallback);
	ros::Subscriber carDetectSub = n.subscribe(carSubTopic, 1, carDetectionCallback);

//	cout << "INIT" << endl;

	try {
		ros::Rate pub_rate(100.0);

		while (ros::ok()) {
			ros::spinOnce();

			cout << "cam spin" << endl;

			pub_rate.sleep();
		}
	} catch (exception& e) {
		ROS_FATAL("camera_blobdetection_node caught exception. Aborting. %s", e.what());
		ROS_BREAK();
	}
}
