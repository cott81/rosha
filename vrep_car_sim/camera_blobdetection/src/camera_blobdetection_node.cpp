#include <camera_blobdetection/CameraBlobdetection.h>

#include "ros/ros.h"
#include "car_msgs/DetectedSignals.h"
#include "std_msgs/Float64.h"
#include "vrep_common/VisionSensorData.h"
//#include "error_seeder/ErrorSeederLib.h"
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
ros::Publisher* timePub;
ros::Publisher* blobValuesPublisher;

// Global variables:
int robotId = 0;

int countVariable = 0;

void sendBlobInformations(std::vector<float> sensorInformations) {

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

				signalMsg.blobsize = blobSize;
				signalMsg.blobOrientation = blobOrientation;
				signalMsg.blobPosX = blobPos[0];
				signalMsg.blobPosY = blobPos[1];
				signalMsg.blobBoxDimensionX = blobBoxDimensions[0];
				signalMsg.blobBoxDimensionY = blobBoxDimensions[1];

				blobValuesPublisher->publish(signalMsg);
			}
		}
	}
} // end of sendBlobInformations function


void blobDetectionCallback(const vrep_common::VisionSensorData::ConstPtr& msg) {
	if (robotId == 0) {
		cout << "BLOBDETECT" << endl;
		std_msgs::Float64 timeMsg;

		clock_t t;
		t = clock();
		float time = (float) t;

		timeMsg.data = time;
		cout << "BLOBDETECT - PUBLISH NOW" << endl;
		timePub->publish(timeMsg);
		cout << "BLOBDETECT - PUBLISHED" << endl;
	}

	std_msgs::Float32MultiArray array = msg->packetData;
	std::vector<float> data = array.data;
	sendBlobInformations(data);
	countVariable = countVariable + 1;
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

	ros::init(argc, argv, "camera_blobdetection_node");
	ros::NodeHandle n;

	if (!robotIdByArg) {
		robotId = supplementary::SystemConfig::GetOwnRobotID();
	}
	ROS_INFO("own robot Id: %d\n", robotId);

//	error_seeder::ErrorSeederLib esl(compId);

	// build topic name
	string blobValuesPubTopic;
	string blobSubTopic;
	stringstream ss;
	if (useRobotIdInTopic) {
		ss << "/vrep/carSim" << robotId << "/blobValues";
		ss >> blobValuesPubTopic;

		ss.str("");
		ss.clear();
		ss << "/vrep/carSim" << robotId << "/blobDetectionData";
		ss >> blobSubTopic;
	} else {
		blobValuesPubTopic = "/vrep/carSim/blobValues";
		blobSubTopic = "/vrep/carSim/blobDetectionData";
	}

	ros::Publisher blobValuesPub = n.advertise<car_msgs::DetectedSignals>(blobValuesPubTopic, 1);
	blobValuesPublisher = &blobValuesPub;

	// node time test
//	if (robotId == 0) {
		cout << "ROBOT ID IS 0 => Time pub" << endl;
		ros::Publisher timePublisher = n.advertise<std_msgs::Float64>("/vrep/carSim0/startTime", 1);
		timePub = &timePublisher;
		cout << "ROBOT ID IS 0 => Time pub gesetzt" << endl;
//	}

//	time_t now = time(0);
//	long nowts = now;
//	cout << "nowTS !!!: " << nowts << endl;

	ros::Subscriber blobDetectSub = n.subscribe(blobSubTopic, 1, blobDetectionCallback);

//	cout << "INIT" << endl;

	try {
		ros::Rate pub_rate(100.0);

		while (ros::ok()) {
			ros::spinOnce();

			pub_rate.sleep();
		}
	} catch (exception& e) {
		ROS_FATAL("camera_blobdetection_node caught exception. Aborting. %s", e.what());
		ROS_BREAK();
	}
}
