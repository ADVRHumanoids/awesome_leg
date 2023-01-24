#include "are_plugins_running.h"

rt_BT::ArePluginsRunning::ArePluginsRunning(const std::string& name,
                                         TickFunctor tick_functor,
                                         const NodeConfiguration& config):
    ConditionNode(name, config), tick_functor_(std::move(tick_functor))
{

}
