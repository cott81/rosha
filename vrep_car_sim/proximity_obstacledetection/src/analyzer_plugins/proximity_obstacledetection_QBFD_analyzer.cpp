#include <analyzer_plugins/proximity_obstacledetection_QBFD_analyzer.h>

using namespace proximity_obstacledetection_analyzer_plugins;
using namespace diagnostic_engine;
using namespace std;

//PLUGINLIB_REGISTER_CLASS(Proximity_Obstacledetection_QBFD_Analyzer, diagnostic_aggregator::Proximity_Obstacledetection_QBFD_Analyzer, diagnostic_aggregator::Analyzer)

Proximity_Obstacledetection_QBFD_Analyzer::Proximity_Obstacledetection_QBFD_Analyzer() :
    path_(""), has_initialized_(false)

{
  this->CLASSNAME = "Proximity_Obstacledetection_QBFD_Analyzer";
  cout << "\n\n\nProximity_Obstacledetection_QBFD_Analyzer: const\n\n\n" << endl;

  /*
  // specify the nodes and states to query by name
  BayesianQueryItem itemComp("RoboPkgCS", -1, "failure", -1); //ids not known yet
  this->detailedQueryRequest.push_back(itemComp);
  BayesianQueryItem itemFailureModeDL("FailureMode", -1, "deadlock", -1);
  this->detailedQueryRequest.push_back(itemFailureModeDL);
  BayesianQueryItem itemFailureModeEL("FailureMode", -1, "endlessLoop", -1);
  this->detailedQueryRequest.push_back(itemFailureModeEL);
  BayesianQueryItem itemFailureModeCr("FailureMode", -1, "crash", -1);
  this->detailedQueryRequest.push_back(itemFailureModeCr);
  BayesianQueryItem itemFailureModeNormal("FailureMode", -1, "normal", -1);
  this->detailedQueryRequest.push_back(itemFailureModeNormal);
  */

  // a better way to do this? ... a config File for the analyzer (-> additional paramter for disturbances)
  // ros parameter server ?
  this->cpuNodeStateLimits[0] = 5;    //lower threshhold [100 x % cpu]
  this->cpuNodeStateLimits[1] = 175;    //upper threshhold


  this->memNodeStateLimits[0] = 10.0;    //low bound [kB]
  this->memNodeStateLimits[1] = 12000.0;   //upper bound

  //heart beat interval: 1s (NOT INCLUDED)
  this->heartBeatIntervalLimits[0] = 900;      //low: below 900ms
  this->heartBeatIntervalLimits[1] = 1100;     //high: over 1100ms

  //thread usage limits
  this->threadNodeStateLimits[0] = 5;   //lower bound
  this->threadNodeStateLimits[1] = 6;  //upper bound

  //stream match count limits
  this->streamNodeStateLimits_nl[0] = 95.0;    //lower bound old: 8
  this->streamNodeStateLimits_nl[1] = 105.0;   //upper bound old: 10.0

  //stream rate exceptions per s
  this->streamNodeStateLimits_ex[0] = 0;        //everything above 0 is abnormal

  //msg Freq limits (NOT INCLUDED)
  this->msgFreqNodeStateLimits[0] = 0.8;  //lower bound old: .9
  this->msgFreqNodeStateLimits[1] = 1.2; //upper bound  old: 1.1

  this->reasoningResults = NULL;

  this->linkRegistered = false;
  this->matched = false;
  this->unmatched = false;

  //get the robot id
  this->robotId = supplementary::SystemConfig::GetOwnRobotID();
  stringstream ss;
  ss << this->robotId;
  ss >> this->robotIdString;
}

Proximity_Obstacledetection_QBFD_Analyzer::~Proximity_Obstacledetection_QBFD_Analyzer()
{
  delete heartBeatIntervalKC;
  delete cpuKC;
  delete memKC;
  delete threadKC;
  delete streamKC_nl;
  delete msgFreqKC;
  //delete logger;
}

bool Proximity_Obstacledetection_QBFD_Analyzer::init(const string base_name, const ros::NodeHandle &n)
{
  ROS_INFO("%s: init(): ", this->CLASSNAME);

  std::string path = ros::package::getPath("proximity_obstacledetection");
  path = path + "/model/";


  std::string  pathGroupParam = "";
  if (!n.getParam("path_group", pathGroupParam))
  {
    ROS_WARN ("No path parameter was specified in Proximity_Obstacledetection_QBFD_Analyzer! Use no parent item.");
  } else {
    path_ = pathGroupParam ;
  }

  std::string  pathCapParam = "";
  if (!n.getParam("cap", pathCapParam))
  {
    ROS_WARN ("No cap parameter was specified in Proximity_Obstacledetection_QBFD_Analyzer! Use no parent item.");
  } else {
    path_ = path_ + "/" + pathCapParam ;
  }
  std::string  pathFuncParam = "";
  if (!n.getParam("func", pathFuncParam))
  {
    ROS_WARN ("No func parameter was specified in Proximity_Obstacledetection_QBFD_Analyzer! Use no parent item.");
  } else {
    path_ = path_ + "/" + pathFuncParam ;
  }

  ROS_INFO("%s: init: path is: %s", this->CLASSNAME, path_.c_str());

  if (!n.getParam("node_to_analyze", this->nodeToAnalyze))
  {
    ROS_ERROR(
        "No power board name was specified in Proximity_Obstacledetection_QBFD_Analyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
    return false;
  }

  ROS_DEBUG("%s: init(): path_group: %s, cap: %s, func: %s, node_to_analyze: %s",
           this->CLASSNAME, pathGroupParam.c_str(), pathCapParam.c_str(), pathFuncParam.c_str(), this->nodeToAnalyze.c_str());

  //get a list from yaml
  n.getParam("systemNodesToAnalyze", systemNodesToAnalyze);
  ROS_ASSERT(systemNodesToAnalyze.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //default
  if (systemNodesToAnalyze.size() == 0) {
    //set default
    systemNodesToAnalyze[0] = "UltraSonic";
  }

  n.getParam("systemNodesStateToAnalyze", systemNodesStateToAnalyze);
  ROS_ASSERT(systemNodesStateToAnalyze.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //default
  if (systemNodesStateToAnalyze.size() == 0) {
    //set default
    systemNodesStateToAnalyze[0] = "failure";
  }

  //both lists need to have the same length
  if (systemNodesToAnalyze.size() == systemNodesStateToAnalyze.size()) {

  for (int i = 0; i < systemNodesToAnalyze.size(); ++i)
    {
      string nodeName = (string) systemNodesToAnalyze[i];
      string nodeState = (string) systemNodesStateToAnalyze[i];
      ROS_DEBUG("%s init(): register request item (name: %s, state: %s).",
               this->CLASSNAME, nodeName.c_str(), nodeState.c_str());

      // specify the nodes and states to query by name
      BayesianQueryItem itemComp(systemNodesToAnalyze[i], -1, systemNodesStateToAnalyze[i], -1); //ids not known yet
      this->detailedQueryRequest.push_back(itemComp);
    }
  } else {
    ROS_ERROR("%s init(): parameter array lengths of \"itemNamesToAnalyze\" and \"systemNodesStateToAnalyze\" do not match!. Take the default (Robot; failure) ",
              this->CLASSNAME);
  }


  boost::shared_ptr<StatusItem> item(new StatusItem("Report"));
  report_item_ = item;

  has_initialized_ = true;

  std::string my_path;
  std::string nice_name = "huhn";
  if (base_name == "/")
    my_path = nice_name;
  else
    my_path = base_name + "/" + nice_name;


//  int z;
//  z = gethostname(hostname, sizeof hostname);
  this->robotname =  supplementary::SystemConfig::getHostname();
  //char to string as stream operation
  this->fullName = "";
  stringstream ss;
//  ss << this->hostname << "_" << this->nodeToAnalyze;
  ss << this->robotname << "_" << this->nodeToAnalyze;
  ss >> this->fullName;

  ROS_INFO("%s init(): full name: %s", this->CLASSNAME, this->robotname.c_str());


  stringstream sss;
//  sss << this->hostname << "_" << "outTopic_ActCap1Func1";
  sss << this->robotname << "_" << "outTopic_ActCap1Func1";
  sss >> this->channelName;


  /*
  //logger
  string customHeader = "Time h:m:s.ms\tTics 1.1.1970 in 10ns\tP(\"RoboPkgCS\" = \"failure\")\tP(\"FailureMode\" = \"deadlock\")\t"
      "P(\"FailureMode\" = \"endlessLoop\")\t"
      "P(\"FailureMode\" = \"crash\")\t"
      "P(\"FailureMode\" = \"normal\")\n";
  string loggerName; // = this->CLASSNAME+"_"+this->fullName;
  stringstream logName;
  logName << this->CLASSNAME << "_" << this->fullName;
  logName >> loggerName;
  this->logger = new dki_helpers::DataLogger(loggerName, "SimRobotSys/EvalSimSystem/log", true, customHeader);
  */

  this->de = DiagnosticEngine::getInstance();
  std::string path2 = path+"proximity_obstacledetection_FailureModel_dbn.xdsl";
  //TODO: class parameters
  unsigned int numOfHistorySlices = 20;
  unsigned int sliceTime = 3000;
  unsigned int currentTimeSlice = 18;

  this->de->GetKnowledgeBase()->SetHistorySettings(numOfHistorySlices, sliceTime, currentTimeSlice);

  bool useModelsDefaultHistorySettings = false;         //flag is needed to use the set history parameters
  this->modelId = de->GetKnowledgeBase()->AddModel(this->fullName, path2, useModelsDefaultHistorySettings);

  ROS_INFO("%s: init: model id: %d", this->CLASSNAME, this->modelId);

  this->currentSliceIndex = 8;         //slice that represents t=0
  //get node Handles. do it here for static models. Dynamic changes require the find in the analyze()
  //TODO: get the node handles from the model
  DSLModel* m = (DSLModel*) de->GetKnowledgeBase()->GetModel(modelId);




  DSLModel* dslModel = dynamic_cast<DSLModel*>(m);
  if (dslModel == NULL) {
    ROS_ERROR("%s: init(): downward cast error of Model.", this->CLASSNAME);
  }
  this->theNet = dslModel->GetDSLNetwork();

  //get nodes for evidence updates
  this->nodeHandle_HeartBeatInterval = theNet->FindNode("HeartBeatIntervallReceiver");
  this->heartBeatIntervalKC = new BayesianKC();

  this->nodeHandle_CpuObs = theNet->FindNode("CpuObs");
  this->cpuKC = new BayesianKC();

  this->nodeHandle_MemObs = theNet->FindNode("MemObs");
  this->memKC = new BayesianKC();

  this->nodeHandle_ThreadObs = theNet->FindNode("ThreadObs");
  this->threadKC = new BayesianKC();

  this->nodeHandle_StreamObs_nl = theNet->FindNode("StreamObs_NewLine");
  this->streamKC_nl= new BayesianKC();

  this->nodeHandle_StreamObs_ex = theNet->FindNode("StreamObs_Exception");
  this->streamKC_ex= new BayesianKC();

  this->nodeHandle_MsgFreqObs = theNet->FindNode("MsgFreqObs");
  this->msgFreqKC = new BayesianKC();

  this->receivedMonUpdates = false; //initialize mon flag







  ostringstream s;
  int nodeHandle;
  int state;

  //iterator to get the nodes by name and set the ids (nodeId, StateId) for the query item
  vector<BayesianQueryItem>::iterator iter2;
  for (iter2 = this->detailedQueryRequest.begin(); iter2 != this->detailedQueryRequest.end(); iter2++) {

    nodeHandle = theNet->FindNode(iter2->GetNodeName().c_str());
    iter2->SetNodeId(nodeHandle);
    DSL_idArray *theNames;
    theNames = theNet->GetNode(nodeHandle)->Definition()->GetOutcomesNames();
    state = theNames->FindPosition(iter2->GetStateName().c_str()); // should be 1 (starts from 1?)
    iter2->SetStateId(state);

    //dont forget to set the log level to debug!
    ROS_DEBUG("%s: init(): query node: %s with nodeId %d, query state: %s with stateId %d",
              this->CLASSNAME, iter2->GetNodeName().c_str(), nodeHandle,
              iter2->GetStateName().c_str(), state);
  }

  ROS_INFO("%s: init: rootCauseNodes: %s", this->CLASSNAME, s.str().c_str());

  //set the query
  this->reasoner = (diagnostic_engine::BayesianReasoner*) de->GetReasoner();
//  this->reasoner->SetQuery(&(this->detailedQueryRequest));

  //TODO: get "normal" prop values to classify for normal warning and critical
  // -get init props (without any evidences) -> normal , or warning?
  // one time calc of the props for the root causes nodes
  // how to define WARN or ERROR



  //this->channelName = "tasmania_outTopic_ActCap1Func1";

  return true;
  //return GenericAnalyzerBase::init(my_path, nice_name, 5.0, -1, false);
}

bool Proximity_Obstacledetection_QBFD_Analyzer::match(const std::string name)
{
  //distributed failure model
  //better to create net here: because only running nodes should contribute to the overall failure model
  // ... what about if nodes were started and stopped, due to a role change
  // ... -> detach method? if no more diagnosis infos arrive (dyn role change should stop to monitor inactive nodes as well)


  cout << "\n\n MATCH \n\n" << endl;


  bool m = false;

  ROS_DEBUG("%s: match(): try to match name: %s with fullName: %s ", this->CLASSNAME, name.c_str(), this->fullName.c_str());
  //fullName is <hostname>_<nodeToAnalyze>
  if (name == this->fullName) {
    ROS_INFO("%s, match(): name matched: %s", this->CLASSNAME, this->nodeToAnalyze.c_str());
    matched = true;
    m = true;
  }

  //no match at all ... needs to match to the chanel for msgFreq as well
  //tasmania_outTopic_ActCap1Func1
  if (name == this->channelName) {
    ROS_INFO("%s, match(): name matched: %s", this->CLASSNAME, channelName.c_str());
    matched = true;
    m = true;
  }

  return m;
}

bool Proximity_Obstacledetection_QBFD_Analyzer::unmatch(const std::string name)
{
  ROS_INFO("%s, unmatch(): unmatch: %s", this->CLASSNAME, name.c_str());
  this->unmatched = true;  //set to be able reset default node values

  return true;
}

bool Proximity_Obstacledetection_QBFD_Analyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  //TODO: analyze() is only called once 1s ... not as fast as the msgs arrive! -> fixed freq, not event based!

  this->stampedTime = item->getStampedTime().toNSec() / (1000*1000); //in ms

  //TODO: replace hard coded stuff
  // -> get different Observers from config (yaml, parameter space)
  // -> get Limits from parameter space ... learned limits? -> should be stored in the node, not in care
  // => one distribuded "model" composed of
  //            FailureModels (DBN) with a priori observer models,
  //            Observer policities (observer adaptation, MIN, MAX RANGE, HIST), observer model learning (histogram)
  //            Set of usefull observers, root causes
  //            structure of the total system (robotModel.xml)

  /*
  int evidenceState;
  bool receivedMonUpdates = false;
  vector<KnowledgeChange*> evidences(0);
  */

  mutexObj.lock();        //lock this section
  int evidenceState;
  receivedMonUpdates = false;

  //erase all items in evidences
  //this->evidences.erase(this->evidences.begin(), this->evidences.end());
  vector<KnowledgeChange*> evidences(0);

  ROS_DEBUG("%s: analyze(): item->getName() : %s", this->CLASSNAME, item->getName().c_str());
  if (item->getName() == this->fullName  || item->getName() == this->channelName)
  {
    report_item_ = item;

    if (item->hasKey("CompNotifierType")) {
      /*
       * Active monitoring updates: receive notifier info
       * Type:            key: CompNotifierType value: HeartBeat,etc.
       * Measurement:     key: Measurement value: value
       */

      //
      // Heart Beat Interval
      //
      if (boost::starts_with(item->getValue("CompNotifierType"), "CompHeartBeatInterval") ) {

        int interval = 0;
        interval = boost::lexical_cast<int>(item->getValue("Interval"));

        //low heart beat interval (less 900ms)
        if (interval < this->heartBeatIntervalLimits[0])
        {
          ROS_INFO("%s: ananlyze: heart beat interval : %d -> state: low", this->CLASSNAME, interval);
          evidenceState = 0;
        } else if (interval < this->heartBeatIntervalLimits[1]) {
          //normal range
          ROS_INFO("%s: ananlyze: heart beat interval : %d -> state: normal", this->CLASSNAME, interval);
          evidenceState = 1;

        } else if (interval > this->heartBeatIntervalLimits[1]) {
          ROS_INFO("%s: ananlyze: heart beat interval : %d -> state: high", this->CLASSNAME, interval);
          evidenceState = 2;
        } else {
          ROS_WARN("%s: ananlyze: invalid value range of the heart beat interval: %d -> no state mapping possible", this->CLASSNAME, interval);
        }

        this->heartBeatIntervalKC->Init(this->modelId, this->nodeHandle_HeartBeatInterval, evidenceState, this->stampedTime);
        evidences.push_back(this->heartBeatIntervalKC); //needed at all ... simply change object?
        receivedMonUpdates = true;
      }

      receivedMonUpdates = true;

    }

    if (item->hasKey("CharacType")) {
      /*
       * Passive monitoring updates: receive observer info
       * Type:            key: CharacType value: CompCpu, CompMem, ... FlowMsgFreq CompStream CompThread
       * Measurement:     key: Measurement value: value
       */

      /*
       * Classification of continues evidence values to discrete node states
       * -> e.g. 3.2345% cpu -> state high of nade CpuObs (normal is 2%)
       */

      //
      //CPU measurement
      //
      if (boost::starts_with(item->getValue("CharacType"), "CompCpu")) {

        bool correctCheck = true;
        //error handling
        try {
          this->cpuUsage = boost::lexical_cast<int>(item->getValue("Measurement"));

          //invalid range check
          if (this->cpuUsage > 10000 || this->cpuUsage < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the cpu measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->cpuUsage);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <int> of the cpu Usage measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }

        if (correctCheck) {
          //discretization of continuous observation values

          if (this->cpuUsage < this->cpuNodeStateLimits[0]) {
            //low state: cpu < 200 (2%)

            ROS_INFO("%s: ananlyze: cpu measurement: %d -> state: low", this->CLASSNAME, this->cpuUsage);
            //TODO: more generic ...
            //0: for the first state: here: low
            evidenceState = 0;

          } else if (this->cpuUsage < this->cpuNodeStateLimits[1]) {
            //normal state: 200 < cpu < 400  //variation of the normal state

            ROS_INFO("%s: ananlyze: cpu measurement: %d -> state: normal", this->CLASSNAME, this->cpuUsage);
            //1: for the normal state: here: normal
            evidenceState = 1;

          } else {
            //high state: cpu > 400

            ROS_INFO("%s: ananlyze: cpu measurement: %d -> state: high", this->CLASSNAME, this->cpuUsage);
            //2: for the first state: here: high
            //theNet->GetNode(this->nodeHandle_CpuObs)->Value()->SetTemporalEvidence(0,2);
            evidenceState = 2;
          }

          this->cpuKC->Init(this->modelId, this->nodeHandle_CpuObs, evidenceState, this->stampedTime);
          evidences.push_back(this->cpuKC); //needed at all ... simply change object?
          receivedMonUpdates = true;
        }
      }

      //
      //MEM measurement
      //
      if (boost::starts_with(item->getValue("CharacType"), "CompMem")) {

        bool correctCheck = true;
        //error handling
        try {
          this->memUsage = boost::lexical_cast<int>(item->getValue("Measurement"));

          //invalid range check
          if (this->memUsage < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the mem measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->memUsage);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <int> of the mem Usage measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }


        if (correctCheck) {

          //set current evidence
          if (this->memUsage < this->memNodeStateLimits[0]) {
            //low state: mem < 15.5 MB

            ROS_INFO("%s: ananlyze: mem measurement: %d -> state: low", this->CLASSNAME, this->memUsage);
            //0: for the first state: here: low
            evidenceState = 0;

          } else if (this->memUsage < this->memNodeStateLimits[1]) {
            //normal state: 15.5 < mem < 17  [MB] //variation of the normal state

            ROS_INFO("%s: ananlyze: mem measurement: %d -> state: normal", this->CLASSNAME, this->memUsage);
            //1: for the normal state: here: normal
            evidenceState = 1;

          } else {
            //high state: 17 < mem [MB]

            ROS_INFO("%s: ananlyze: mem measurement: %d -> state: high", this->CLASSNAME, this->memUsage);
            //2: for the first state: here: high
            evidenceState = 2;
          }

        }

        this->memKC->Init(this->modelId, this->nodeHandle_MemObs, evidenceState, this->stampedTime);
        evidences.push_back(this->memKC);
        receivedMonUpdates = true;
      }

      //
      //THREAD measurement
      //
      if (boost::starts_with(item->getValue("CharacType"), "CompThread")) {

        bool correctCheck = true;
        //error handling
        try {
          this->threadUsage = boost::lexical_cast<int>(item->getValue("Measurement"));

          //invalid range check
          if (this->threadUsage < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the thread measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->threadUsage);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <int> of the thread Usage measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }

        if (correctCheck) {
          //set current evidence

          if (this->threadUsage < this->threadNodeStateLimits[0]) {
            //low state: thread < 11

            ROS_INFO("%s: ananlyze: thread measurement: %d -> state: abnormal", this->CLASSNAME, this->threadUsage);
            //0: for the first state: here: abnormal
            evidenceState = 1;

          } else if (this->threadUsage < this->threadNodeStateLimits[1]) {
            //normal state: 11 < thread < 13  //variation of the normal state

            ROS_INFO("%s: ananlyze: thread measurement: %d -> state: normal", this->CLASSNAME, this->threadUsage);
            //1: for the normal state: here: normal
            evidenceState = 0;

          } else {
            //abnormal state: 13 =< mem

            ROS_INFO("%s: ananlyze: thread measurement: %d -> state: abnormal", this->CLASSNAME, this->threadUsage);
            //0: for the first state: here: abnormal
            evidenceState = 1;
          }
        }

        this->threadKC->Init(this->modelId, this->nodeHandle_ThreadObs, evidenceState, this->stampedTime);
        evidences.push_back(this->threadKC);
        receivedMonUpdates = true;
      }

      //
      //STREAM MATCH COUNT/RATE measurement (LINES / s)
      //

      if (boost::starts_with(item->getValue("CharacType"), "CompStream_NewLineRate")) {

        bool correctCheck = true;
        //error handling
        try {
          this->streamRate_nl = boost::lexical_cast<double>(item->getValue("Measurement"));

          //invalid range check
          if (this->streamRate_nl < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the stream rate (lines/s) measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->streamRate_nl);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <double> of the stream rate (lines/s) measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }

        if (correctCheck) {

          //set current evidence
          if (this->streamRate_nl < this->streamNodeStateLimits_nl[0]) {
            //low state: lines/s < 8.0

            ROS_INFO("%s: ananlyze: stream measurement (new line): %d -> state: low", this->CLASSNAME, this->streamRate_nl);
            //0: for the first state: here: no
            evidenceState = 0;

          } else if (this->streamRate_nl < this->streamNodeStateLimits_nl[1]) {
            //low state: 8.0 < lines/s < 10.0  //variation of the low state

            ROS_INFO("%s: ananlyze: stream measurement (new line): %d -> state: normal", this->CLASSNAME, this->streamRate_nl);
            //1: for the normal state: here: low
            evidenceState = 1;

          } else {
            //normal state: 15 < stream matches

            ROS_INFO("%s: ananlyze: stream measurement (new line): %d -> state: high", this->CLASSNAME, this->streamRate_nl);
            //2: for the first state: here: normal
            evidenceState = 2;
          }

          this->streamKC_nl->Init(this->modelId, this->nodeHandle_StreamObs_nl, evidenceState, this->stampedTime);
          evidences.push_back(this->streamKC_nl);
          receivedMonUpdates = true;
        }
      }

      //
      //STREAM MATCH COUNT/RATE measurement (EXCEPTIONS / s)
      //

      if (boost::starts_with(item->getValue("CharacType"), "CompStream_ExceptionRate")) {

        bool correctCheck = true;
        //error handling
        try {
          this->streamRate_ex = boost::lexical_cast<double>(item->getValue("Measurement"));

          //invalid range check
          if (this->streamRate_ex < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the stream rate (exceptions/s) measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->streamRate_ex);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <double> of the stream rate (exceptions/s) measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }


        //set current evidence
        if (this->streamRate_ex == this->streamNodeStateLimits_ex[0]) {
          //normal state: exception rate == 0

          ROS_INFO("%s: ananlyze: stream measurement (exceptions): %d -> state: absent", this->CLASSNAME, this->streamRate_ex);
          //0: for the first state: here: no
          evidenceState = 0;

        } else {
          //abnormal state: exception rate > 0

          ROS_INFO("%s: ananlyze: stream measurement (exceptions): %d -> state: present", this->CLASSNAME, this->streamRate_ex);
          //1: for the normal state: here: low
          evidenceState = 1;

        }
        this->streamKC_ex->Init(this->modelId, this->nodeHandle_StreamObs_ex, evidenceState, this->stampedTime);
        evidences.push_back(this->streamKC_ex);
        receivedMonUpdates = true;
      }


      //
      //MSG_FREQ measurement
      //
      if (boost::starts_with(item->getValue("CharacType"), "FlowMsgFreq")) {

        bool correctCheck = true;
        //error handling
        try {
          this->msgFreq = boost::lexical_cast<double>(item->getValue("Measurement"));

          //invalid range check
          if (this->msgFreq < 0) {
            ROS_ERROR("%s: ananlyze: invalid value range of the msg freq measurement: %d -> no state mapping possible. Skip this observation.",
                      this->CLASSNAME, this->msgFreq);
            correctCheck = false;
          }

        } catch (boost::bad_lexical_cast &) {
          ROS_ERROR("%s: analyze: bad lexical cast <double> of the msg freq measurement: %s. Skip this observation.",
                    this->CLASSNAME, item->getValue("Measurement").c_str());
          correctCheck = false; //skip the rest
        }


        if (correctCheck) {

          //set current evidence
          if (this->msgFreq < this->msgFreqNodeStateLimits[0]) {
            //abnormal state: msgFreq < 0.9

            ROS_INFO("%s: ananlyze: msgFreq measurement: %d -> state: abnormal", this->CLASSNAME, this->msgFreq);
            //1: for the first state: here: abnormal
            evidenceState = 1;

          } else if (this->msgFreq < this->msgFreqNodeStateLimits[1]) {
            //normal state: 0.9 < msgFreq < 1.1 //variation of the normal state

            ROS_INFO("%s: ananlyze: msgFreq measurement: %d -> state: normal", this->CLASSNAME, this->msgFreq);
            //0: for the normal state: here: normal
            evidenceState = 0;

          } else {
            //abnormal state: 1.1 < msgFreq

            ROS_INFO("%s: ananlyze: msgFreq measurement: %d -> state: abnormal", this->CLASSNAME, this->msgFreq);
            //2: for the first state: here: abnormal
            evidenceState = 1;
          }
        }

        this->msgFreqKC->Init(this->modelId, this->nodeHandle_MsgFreqObs, evidenceState, this->stampedTime);
        evidences.push_back(this->msgFreqKC);
        receivedMonUpdates = true;
      }

    }// CharacType -> passive mon observation

    if(receivedMonUpdates) {

      this->de->GetKnowledgeBase()->SetCurrentModel(this->modelId);     //Thread safty !!!
      //set query (because multi model reasoner ... overwrites "old query"
      //this->reasoner->SetQuery(&(this->detailedQueryRequest));

      //need to bring and update the evidence in the net
      this->de->GetKnowledgeBase()->Update(evidences);
    }

    //return false; // Won't report this item ... using "other_analyzers"
    mutexObj.unlock();
    return true;
  }



  return false;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > Proximity_Obstacledetection_QBFD_Analyzer::report()
{
  //called each 1s independend from analyze()
  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  if (!this->matched)
  {
    return output;
  }

  // if we do not have recent continous mon updates, we assume that this analyzer is deactiveded
  if (this->unmatched)
  {
    // set the a priori probs to default values, here to 100% ok to have no influence in the reasoning any more
    BayesianKB* bkb = (BayesianKB*) this->de->GetKnowledgeBase();
    bkb->SetSystemModelNodeDefaultDef("UltraSonic", 0.0, 1.0);

    //better solution define stat. independance by resetting the definition of the parent (cap node)

    this->unmatched = false;
    this->matched = false;

    //need to send a msg that everything is ok else the old failure state stays (Failure Recovery)
    //TODO: hard coded ... be carefull ... replace with known strings ...
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ds = report_item_->toStatusMsg(path_);
    ds->level = diagnostic_msgs::DiagnosticStatus::OK;
    ds->message = "OK: Estimated failure probability < 50%. All in normal range";
    ds->name = this->fullName;
    ds->hardware_id = this->robotIdString;
    vector<diagnostic_msgs::KeyValue> kvs;
    diagnostic_msgs::KeyValue kv;
    kv.key = "CompFailure_UltraSonic_failure";
    kv.value = "0";
    kvs.push_back(kv); //copy ...
    kv.key = "RootCause_FailureMode_deadlock";
    kv.value = "0";
    kvs.push_back(kv);
    kv.key = "RootCause_FailureMode_endlessLoop";
    kv.value = "0";
    kvs.push_back(kv);
    kv.key = "RootCause_FailureMode_crash";
    kv.value = "0";
    kvs.push_back(kv);
    kv.key = "RootCause_FailureMode_normal";
    kv.value = "1";
    kvs.push_back(kv);
    ds->values = kvs;
    output.push_back(ds);

    return output;
  }

  // register link between compModel and system model
  // in report(): to make sure the system model is already placed (alternative: hard in diagnostic engine)
  if (!this->linkRegistered) {
    BayesianML modelLink;
  //  modelLink.SetCompModelNodeName("RoboPkgCS");
    int failureDetectionNode = theNet->FindNode("UltraSonic");     //node for the failure probability
    modelLink.SetCompModelNodeId(failureDetectionNode);
    modelLink.SetModelId(this->modelId);
  //  modelLink.SetSystemModelNodeId(1000);
  //  modelLink.SetSystemModelNodeName("RoboPkgCS2");
    modelLink.SetSystemModelNodeName(this->nodeToAnalyze);        //name of the node have to be the system node name here!
    //TODO: get name of the system model node by ros param!
  //  modelLink.SetSystemModelNodeId(1);

    /*
      unsigned short v[2] = {1, 0};  //state index from zero? yes
      vector<unsigned short> vv (v, v +sizeof(v) /sizeof(unsigned short) );
    modelLink.SetStateMapping(vv);
    */
    modelLink.SetDirectMapping(2);

    this->de->GetKnowledgeBase()->AddModelLink(modelLink);
   this->linkRegistered = true;
  }

  //what topic? what infos?
  //infos of interest:
  //comp identity:      name? id? already included?
  //    failure prop:   double
  //    root causes[type, prob] prob:     double[]   -> upper two levels of the model

  //TODO: Problem: right now just reports the continously LAST reasoning result ... no "no evidence" steps in between (could easily be done ...)
  // -> need function to time update the observation queue without new evidences!

  //synchronized: report is called by the main thread, analyze by the subscribe thread by ROS
  //to avoid strange effects (e.g. by setting var receiveMonUpdates) the two sections are protected
  mutexObj.lock();
  //if(receivedMonUpdates) {

    //reset flag to false
    receivedMonUpdates = false;

    this->de->GetKnowledgeBase()->SetCurrentModel(this->modelId);
    //set query (because multi model reasoner ... overwrites "old query"
    this->reasoner->SetQuery(&(this->detailedQueryRequest));


    //need to update the evidence in the net
    unsigned long currentTime = ros::Time::now().toNSec() / (1000*1000);
    this->de->GetKnowledgeBase()->UpdateTime(currentTime);

    //need a update queue method without given evidence -> empty evidence for all nodes

    //infer with the given reasoner
    de->GetKnowledgeBase()->Accept( *(this->reasoner) ); //this->reasoner

    //get the results ...
    this->reasoningResults = this->reasoner->GetResults();

  //print the DBN results
    unsigned int length = this->reasoningResults->size();
    double logData[length];
    int counter = 0;
    cout << "\n" << endl;
  for (vector<BayesianQueryItem>::iterator iter = this->reasoningResults->begin(); iter != this->reasoningResults->end(); iter++ ) {

    ROS_INFO("%s %s: report(): t=%d P(\"%s\" = \"%s\") = %f", this->CLASSNAME, this->nodeToAnalyze.c_str(),
              currentSliceIndex, iter->GetNodeName().c_str(), iter->GetStateName().c_str(), iter->GetStateProp());
    logData[counter] = iter->GetStateProp();
    counter++;
  }

  //this->logger->Write(logData, length);


  mutexObj.unlock();


  if (this ->reasoningResults == NULL) {
    ROS_WARN("%s: report(): no reasoning results available. Empty report.", this->CLASSNAME);
    //do nothing here -> send empty msg.
    //system sends aggregated report stati for all analyzers
    return output;
  }

  //boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ds = report_item_->toStatusMsg(path_);
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ds(new diagnostic_msgs::DiagnosticStatus());

  //set the name
  ds->name = this->fullName;

  //set the robotId
  ds->hardware_id = this->robotIdString;

  vector<diagnostic_msgs::KeyValue> kvs;

  std::vector<diagnostic_engine::BayesianQueryItem>::iterator iter;
  double prop = -1.0;
  double compFailureProp = -1.0;
  for (iter = this->reasoningResults->begin(); iter !=this->reasoningResults->end(); iter++) {
    prop = -1.0;
    diagnostic_msgs::KeyValue kv;
    prop = iter->GetStateProp();

    //CompFailure has to be the first item!!!
    if (iter == this->reasoningResults->begin()){
      kv.key = "CompFailure_"+iter->GetNodeName()+"_"+iter->GetStateName(); //better to ensure that query has node names
      compFailureProp = prop;
    } else {
      //rest should address root causes
      kv.key = "RootCause_"+iter->GetNodeName()+"_"+iter->GetStateName(); //better to ensure that query has node names
    }

    kv.value = boost::lexical_cast<std::string> (prop);
    kvs.push_back(kv);
  }


  ds->values = kvs;

  //Classification needed: classes OK, WARN, ERROR
  //TODO: how to do this -> in relation to importance? prio?
  // higher priorisation -> earlier repair? or more tolerance (because this is needed for task)
  // MAPE-K use ros parameter space as holistic(?) knowledge/model

  //TODO: hard coded case specific numbers -> needs more precise meaningfull classification ... HOW???
  if (compFailureProp < 0.50)
  {
    ds->level = diagnostic_msgs::DiagnosticStatus::OK;
    ds->message = "OK: Estimated failure probability < 50%. All in normal range";
  }
  else if (compFailureProp < 0.85)
  {
    ds->level = diagnostic_msgs::DiagnosticStatus::WARN;
    ds->message = "WARN: Estimated failure probability x between 50% < x < 85%. Is in unusual range, but not critical";
  }
  else if (compFailureProp <= 1.0)
  {
    ds->level = diagnostic_msgs::DiagnosticStatus::ERROR;
    ds->message = "ERROR: Estimated failure probability > 85%. In critical range.";
  }
  else
  {
    ds->level = diagnostic_msgs::DiagnosticStatus::ERROR;
    ds->message = " Failure probability greater 1! Impossible result. check it!";
    ROS_ERROR("%s report(): failure probability > 1: %f", this->CLASSNAME, compFailureProp);
  }

  output.push_back(ds);

  return output;
}

