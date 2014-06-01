/*
 * errorseedernode.cpp
 *
 *  Created on: May 28, 2014
 *      Author: dominik
 */

#include "../include/error_seeder/ErrorSeeder.h"
#include "ros/ros.h"
#include <exception>

using namespace std;

void call_from_thread() {
    std::cout << "Hello, World" << std::endl;
}

int main (int argc, char** argv)
{
  cout << "start error_seeder" << endl;

  //ros::init(argc, argv, "ErrorSeeder");

  try
  {
    //create ErrorSeeder
    error_seeder::ErrorSeeder es(argc, argv);

    //start ErrorSeeder
    es.Start();
  }
  catch (exception& e)
  {
    //cerr << "Diagnostic aggregator node caught exception. Aborting." << e.what() << endl;
    ROS_FATAL("Diagnostic aggregator node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }

  exit(0);
  return 0;

}
