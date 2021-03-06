#ifndef SYSTEMCONFIG_H_
#define SYSTEMCONFIG_H_

using namespace std;

#include <map>
#include <string>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>
#include <fstream>
#include <iostream>

#include <FileSystem.h>

#include "Configuration.h"

//changed by dki
const string DOMAIN_FOLDER = "ES_ROOT";
const string DOMAIN_CONFIG_FOLDER = "ES_CONFIG_ROOT";

namespace supplementary
{
	class SystemConfig
	{

	protected:
		static string rootPath;
		static string configPath;
		static string hostname;
		static mutex configsMapMutex;
		static map<string, shared_ptr<Configuration> > configs;
//		static const char NODE_NAME_SEPERATOR = '_';
		//static int ownRobotID;

	public:
		static SystemConfig* getInstance();
		static string robotNodeName(const string& nodeName);
		static int GetOwnRobotID();
		static string getHostname();
		static void setHostname(string newHostname);
		//static void resetHostname();
		Configuration *operator[](const string s);
		string getRootPath();
		string getConfigPath();
		void setRootPath(string rootPath);
		void setConfigPath(string configPath);
		static string getEnv(const string& var);
		//static int ownRobotID;

	private:
		SystemConfig();
		~SystemConfig(){};
	};

	//int SystemConfig::ownRobotID= 0;
	static int ownRobotID = -1;
}
#endif /* SYSTEMCONFIG_H_ */
