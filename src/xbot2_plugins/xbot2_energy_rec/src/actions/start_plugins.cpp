#include "start_plugins.h"
#include "../utils_defs.hpp"

using namespace BT;

NodeStatus StartPlugins::tick()
{

    std::cout << Colors::kBlue << "ticking StartPlugins action" << Colors::kEndl << std::endl;

    return NodeStatus::SUCCESS;

}
