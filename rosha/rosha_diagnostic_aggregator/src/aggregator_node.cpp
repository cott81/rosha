/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**< \author Kevin Watts */

#include <diagnostic_aggregator/aggregator.h>
#include <exception>
#include <DiagnosticEngine/DiagnosticEngine.h>
//#include <DiagnosticEngine/KnowledgeBase/DSLFactory.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianKC.h>
#include <DiagnosticEngine/KnowledgeBase/KnowledgeChange.h>
#include <DiagnosticEngine/KnowledgeBase/BayesianKB.h>
#include <DiagnosticEngine/KnowledgeBase/KnowledgeBase.h>

#include "ros/ros.h"

using namespace std;
using namespace diagnostic_engine;

int main(int argc, char **argv)
{

  string help = "RoSHA Diagnostic Aggregator\n"
      "Synobsis: rosha_diagnostic_aggregator_node OPTIONS\n"
      "Options:\n\n"
      "ROS params: paramter for ROS, check the ROS wiki for more details.\n"
      "-help: prints this help text\n"
      "-loadFile: loads the specified yaml file for anayzer specifiactions relative to this package path.\n"
      "-loadPath: Path of the yaml file. Default is this package location.\n"
      ;

  string helpParam = "-help";
  string loadFileParam = "-loadFile";
  bool isLoadFileSet = false;
  string fileName;
  string loadPathParam = "-loadPath";
  bool isLoadPathSet = false;
  string loadPath;
  for (int i=1; i<argc; i++)
  {
    if(helpParam.compare(argv[i]) == 0)
    {
    cout << help << endl;
    exit(0);
    }
    else if (loadFileParam.compare(argv[i]) == 0)
    {
      isLoadFileSet = true;
      string s (argv[i+1]);
      fileName = s;
      ROS_INFO("yaml loadFile: %s", argv[i+1]);
    }
    else if (loadPathParam.compare(argv[i]) == 0)
    {
      isLoadPathSet = true;
      string s (argv[i+1]);
      loadPath = s;
      ROS_INFO("yaml loadPath: %s", argv[i+1]);
    }
    else
    {
      //ros arguments ...
    }
  }


  if (isLoadFileSet)
  {
    string path = "";
    if (isLoadPathSet)
    {
      path = loadPath;
    }
    else
    {
      path = ros::package::getPath("rosha_diagnostic_aggregator");
      ROS_INFO("yaml (default) loadPath: %s", path.c_str());
      cout << "path " << path << endl;
    }

    //load the yaml file
    string pathedFile = path + "/"+fileName;
    string cmd = "rosparam load " + pathedFile + " /rosha_diagnostic_aggregator";
    ROS_INFO("load analyzers command: %s", cmd.c_str());
    cout << "cmd: " << cmd << endl;
    system(cmd.c_str());
  }

  //eager instantiation (heavilly used -> thread safe, because there is only one thread here)
  DiagnosticEngine* de = DiagnosticEngine::getInstance();
  de->Init("SystemModel.xdsl");

  ros::init(argc, argv, "rosha_diagnostic_aggregator");

  try
  {
    diagnostic_aggregator::Aggregator agg;

    ros::Rate pub_rate(agg.getPubRate());
    while (agg.ok())
    {
      ros::spinOnce();
      agg.publishData();
      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("Diagnostic aggregator node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }

  exit(0);
  return 0;
}

