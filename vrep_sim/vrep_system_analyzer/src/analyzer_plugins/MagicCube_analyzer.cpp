#include <analyzer_plugins/MagicCube_analyzer.h>

using namespace vrep_magic_cube_analyzer_plugins;
using namespace diagnostic_engine;
using namespace std;

//PLUGINLIB_REGISTER_CLASS(MagicCubeAnalyzer, diagnostic_aggregator::MagicCubeAnalyzer, diagnostic_aggregator::Analyzer)

MagicCubeAnalyzer::MagicCubeAnalyzer() :
    path_(""), has_initialized_(false)
{
  this->CLASSNAME = "MagicCubeAnalyzer";
  cout << "\n\n\nMagicCubeAnalyzer: const\n\n\n" << endl;

  this->de = NULL;
  this->reasoner = NULL;
  this->theNet = NULL;
  this->modelId = NULL;
  this->reasoningResults = NULL;

  //get the robot id
  this->robotId = supplementary::SystemConfig::GetOwnRobotID();
  stringstream ss;
  ss << this->robotId;
  ss >> this->robotIdString;

  int z;
  z = gethostname(hostname, sizeof hostname);
}

MagicCubeAnalyzer::~MagicCubeAnalyzer()
{
}

bool MagicCubeAnalyzer::init(const string base_name, const ros::NodeHandle &n)
{
  ROS_INFO("%s: init(): ", this->CLASSNAME);

  std::string path = ros::package::getPath("vrep_system_analyzer");
  path = path + "/model/";


  std::string  pathGroupParam = "";
  if (!n.getParam("path_group", pathGroupParam))
  {
    ROS_WARN ("No path parameter was specified in MagicCubeAnalyzer! Use no parent item.");
  } else {
    path_ = pathGroupParam ;
  }

  std::string  pathCapParam = "";
  if (!n.getParam("cap", pathCapParam))
  {
    ROS_WARN ("No cap parameter was specified in MagicCubeAnalyzer! Use no parent item.");
  } else {
    path_ = path_ + "/" + pathCapParam ;
  }
  std::string  pathFuncParam = "";
  if (!n.getParam("func", pathFuncParam))
  {
    ROS_WARN ("No func parameter was specified in MagicCubeAnalyzer! Use no parent item.");
  } else {
    path_ = path_ + "/" + pathFuncParam ;
  }

  ROS_INFO("%s: init: path is: %s", this->CLASSNAME, path_.c_str());


  if (!n.getParam("node_to_analyze", this->nodeToAnalyze))
  {
    ROS_ERROR(
        "No power board name was specified in MagicCubeAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
    return false;
  }

  //get a list from yaml
  n.getParam("systemNodesToAnalyze", systemNodesToAnalyze);
  ROS_ASSERT(systemNodesToAnalyze.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //default
  if (systemNodesToAnalyze.size() == 0) {
    //set default
    systemNodesToAnalyze[0] = "Robot";
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

  ROS_DEBUG("%s: init(): path_group: %s, cap: %s, func: %s, node_to_analyze: %s",
           this->CLASSNAME, pathGroupParam.c_str(), pathCapParam.c_str(), pathFuncParam.c_str(), this->nodeToAnalyze.c_str());


  //boost::shared_ptr<StatusItem> item(new StatusItem("Report"));
  boost::shared_ptr<StatusItem> item(new StatusItem(""));
//  boost::shared_ptr<StatusItem> item(new StatusItem(this->itemName));
  report_item_ = item;

  has_initialized_ = true;

  std::string my_path;
  std::string nice_name = "huhn";
  if (base_name == "/")
    my_path = nice_name;
  else
    my_path = base_name + "/" + nice_name;


  this->de = DiagnosticEngine::getInstance();
  std::string path2 = path+"MagicCube_scout_model.xdsl";

  bool useModelsDefaultHistorySettings = true;         //flag is needed to use the set history parameters
  bool isSystemModel = true;
  this->modelId = de->GetKnowledgeBase()->AddModel(path2, isSystemModel, useModelsDefaultHistorySettings);

  ROS_INFO("%s: init: model id: %d", this->CLASSNAME, this->modelId);

  bool useSystemModel = true;
  DSLModel* m = (DSLModel*) de->GetKnowledgeBase()->GetModel(modelId, useSystemModel);

  DSLModel* dslModel = dynamic_cast<DSLModel*>(m);
  if (dslModel == NULL) {
    ROS_ERROR("%s: init(): downward cast error of Model.", this->CLASSNAME);
  }
  this->theNet = dslModel->GetDSLNetwork();

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
  this->reasoner = (BayesianReasoner*) de->GetReasoner();

  return true;
}

bool MagicCubeAnalyzer::match(const std::string name)
{
  bool matched = false;

  /*
  ROS_DEBUG("%s: match(): try to match name: %s with fullName: %s ", this->CLASSNAME, name.c_str(), this->fullName.c_str());
  //fullName is <hostname>_<nodeToAnalyze>
  if (name == this->fullName) {
    ROS_INFO("%s, match(): name matched: %s", this->CLASSNAME, this->nodeToAnalyze.c_str());
    matched = true;
  }
  */

  return matched;
}

bool MagicCubeAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  return false;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > MagicCubeAnalyzer::report()
{
  mutexObj.lock();


  // infer model ...
  //TODO: set the system model ???
  bool useSystemModel = true;
  this->de->GetKnowledgeBase()->SetCurrentModel(this->modelId, useSystemModel);

  //set query (because multi model reasoner ... overwrites "old query"
  this->reasoner->SetQuery(&(this->detailedQueryRequest));

  //need to update the evidence in the net
  unsigned long currentTime = ros::Time::now().toNSec() / (1000*1000);
  this->de->GetKnowledgeBase()->UpdateTime(currentTime);

  //infer with the given reasoner
  ROS_INFO("%s: report(): start to infer the system model.", this->CLASSNAME);
  de->GetKnowledgeBase()->Accept( *(this->reasoner) ); //this->reasoner

  //get the results ...
  this->reasoningResults = this->reasoner->GetResults();

  //print the DBN results
    unsigned int length = this->reasoningResults->size();
    cout << "\n" << endl;
  for (vector<BayesianQueryItem>::iterator iter = this->reasoningResults->begin(); iter != this->reasoningResults->end(); iter++ ) {

    ROS_INFO("%s %s: report(): t=%d P(\"%s\" = \"%s\") = %f", this->CLASSNAME, this->nodeToAnalyze.c_str(),
              -1, iter->GetNodeName().c_str(), iter->GetStateName().c_str(), iter->GetStateProp());
  }


  mutexObj.unlock();



  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  if (this ->reasoningResults == NULL) {
    ROS_WARN("%s: report(): no reasoning results available. Empty report.", this->CLASSNAME);
    //do nothing here -> send empty msg.
    //system sends aggregated report stati for all analyzers
    return output;
  }

  std::vector<diagnostic_engine::BayesianQueryItem>::iterator iter;
  double prop = -1.0;
  double compFailureProp = -1.0;
  for (iter = this->reasoningResults->begin(); iter !=this->reasoningResults->end(); iter++) {

    //boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ds = report_item_->toStatusMsg(iter->GetNodeName());
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> ds(new diagnostic_msgs::DiagnosticStatus());
    stringstream ss;
    ss << this->hostname << "_" << iter->GetNodeName();
    ss >> ds->name;
    //ds->name = iter->GetNodeName();

    //set the robotId
    ds->hardware_id = this->robotIdString;

    vector<diagnostic_msgs::KeyValue> kvs;

    prop = -1.0;
    diagnostic_msgs::KeyValue kv;
    prop = iter->GetStateProp();

    kv.key = "CompFailure_"+iter->GetNodeName()+"_"+iter->GetStateName();
    kv.value = boost::lexical_cast<std::string> (prop);

    kvs.push_back(kv);

    ds->values = kvs;

    //TODO: hard coded case specific numbers -> needs more precise meaningfull classification ... HOW???
    if (prop < 0.50)
    {
      ds->level = diagnostic_msgs::DiagnosticStatus::OK;
      ds->message = "OK: Estimated failure probability < 50%. All in normal range";
    }
    else if (prop < 0.85)
    {
      ds->level = diagnostic_msgs::DiagnosticStatus::WARN;
      ds->message = "WARN: Estimated failure probability x between 50% < x < 85%. Is in unusual range, but not critical";
    }
    else if (prop <= 1.0)
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
  }

  return output;

}

