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
  /*
  string s = "Das is\tsuoer\n";
  cout << s << endl;
  s.erase(std::remove_if( s.begin(), s.end(),
       [](char c){ return (c =='\r' || c =='\t' || c == ' ' || c == '\n');}),
          s.end() );
  cout << s << endl;
  */

  /*
  string pathedrelayConfFile = "/home/dominik/work/impera/MSLCN/MSLDDSProxy/test.conf";
  string line;
  ifstream relayMsgFileToCheck (pathedrelayConfFile);
  if (relayMsgFileToCheck.is_open())
  {
    while ( getline (relayMsgFileToCheck,line) )
    {
      cout << line << endl;
      if (line[0] == '#')
      {
        cout << "ignore COMMENT" << endl;
        continue;
      }
      stringstream iss (line);
      string ss;
      getline(iss, ss, ':'); //or better :?
      cout << "got until : : " << ss << endl;
      if (ss.compare("Topic") != 0)
      {
        cout << "not valid line" << endl;
        continue;
      }
      //string rest = iss.str();
      //cout << "rest in iss: " << rest << endl;

      string topicName;
      getline(iss, topicName, ':');
      //if (topicName[0])

       cout << "after next ws: " << topicName << endl;
       std::string topic("AlicaEngine/SyncTalk");

       //just remove whitespaces at the beginning
       for (int i=0; i<topicName.size(); i++) {
         if (topicName[i] == ' ' || topicName[i] == '\t')
         {
           topicName.erase(i, 1);
           i--;
         } else {
           //top to remove whitespaces
           //cout << "stop at position: " << i << endl;
           break;
         }
       }

       cout << "after next ws: " << topicName << endl;

       if (!topicName.compare(0, topic.size(), topic))
          {
           if (topicName.size() <= topic.size())
           {
             //no match ...
           } else {
             //no match
             if (topicName[topic.size()] == ' ' || topicName[topic.size()] == '\t')
             {
               cout << "MATCH !!!!" << endl;
               exit(0);
             } else {
               //longer string and only the first mathces ...
               //NO MATCH
             }
           }
             //foo_value = atoi(topicName.substr(topic.size()).c_str());

          }

      //erase white spaces with lampda expression
//      s.erase(std::remove_if( s.begin(), s.end(),
//           [](char c){ return (c =='\r' || c =='\t' || c == ' ' || c == '\n');}),
//              s.end() );

      cout << endl << endl << endl;

    }
    relayMsgFileToCheck.close();
  }
  else {
    printf("AddCommLink: Unable to open udp proxy conf file: %s", pathedrelayConfFile.c_str());
    return 0;
  }

  exit(0);
*/

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


