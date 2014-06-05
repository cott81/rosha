/*
 * repair_executor.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: dominik
 */


#include "../include/repair_executor/RepairExecutor.h"
#include "ros/ros.h"
#include <exception>

using namespace std;
using namespace repair_executor;

int main (int argc, char** argv)
{

  cout << "RepairExecuter\n" << endl;

  try
  {
    //create repair executor
    RepairExecutor re(argc, argv);
    re.Start();
  }
  catch (exception& e)
  {
    ROS_FATAL("Diagnostic aggregator node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }

  exit(0);
  return 0;

}


