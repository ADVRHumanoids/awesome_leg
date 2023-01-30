#include "are_plugins_running.h"

using namespace BT;

ArePluginsRunning::ArePluginsRunning(const std::string& name, const NodeConfiguration& config) :
    ConditionNode(name, config){

};

PortsList ArePluginsRunning::providedPorts()
{

  return { OutputPort<bool>("are_plugins_running") };

}

NodeStatus ArePluginsRunning::tick()
{

    setOutput("are_plugins_running", true);

    return NodeStatus::SUCCESS;

}
