#include "are_plugins_running.h"
#include "../utils_defs.hpp"

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

    std::cout << Colors::kGreen << "ticking ArePluginsRunning" << Colors::kEndl << std::endl;

    return NodeStatus::SUCCESS;

}
