#ifndef ARE_PLUGINS_RUNNING_H
#define ARE_PLUGINS_RUNNING_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/leaf_node.h>

using namespace  BT;

namespace rt_BT
{

    class ArePluginsRunning : public ConditionNode
    {
    public:
      typedef std::function<NodeStatus(TreeNode&)> TickFunctor;

      // You must provide the function to call when tick() is invoked
      ArePluginsRunning(const std::string& name, TickFunctor tick_functor,
                          const NodeConfiguration& config);

      ~ArePluginsRunning() override = default;

    protected:
      virtual NodeStatus tick() override;

      TickFunctor tick_functor_;
    };

}

#endif // ARE_PLUGINS_RUNNING_H
