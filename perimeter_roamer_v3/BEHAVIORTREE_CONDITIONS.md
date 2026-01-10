# BehaviorTree.CPP Custom Conditions

## Overview

BehaviorTree.CPP v3 does not support inline C++ code in XML files. The `_cppcode` attribute I used in the initial example was incorrect. Here are the proper ways to create custom conditions:

## 1. Custom Condition Node Classes (Recommended)

Create a C++ class that inherits from `BT::ConditionNode`:

```cpp
class CheckGoalInterrupted : public BT::ConditionNode
{
public:
  CheckGoalInterrupted(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("was_interrupted", "Whether a navigation goal was interrupted")
    };
  }

  BT::NodeStatus tick() override
  {
    bool was_interrupted = false;
    if (getInput("was_interrupted", was_interrupted) && was_interrupted) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};
```

Then register it with the factory:

```cpp
factory.registerNodeType<CheckGoalInterrupted>("CheckGoalInterrupted");
```

## 2. Lambda Functions (Simple Cases)

For simple conditions, you can register lambda functions:

```cpp
// Register a simple boolean check
factory.registerSimpleCondition("IsTrue", [](BT::TreeNode& self) {
    bool value = false;
    self.getInput("value", value);
    return value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
});
```

## 3. Script Nodes (BehaviorTree.CPP v4+)

In BehaviorTree.CPP v4, there's support for script nodes, but this requires Lua:

```xml
<Script code="return my_variable > 10" />
```

## 4. Built-in Condition Patterns

### Using Existing Nodes
Often you can achieve what you need with existing nodes:

```xml
<!-- Use a simple action that sets a flag -->
<SetBlackboard output_key="goal_interrupted" value="true" />

<!-- Later check the flag -->
<BlackboardCheckBool key="goal_interrupted" expected="true" />
```

### Inverter Pattern
```xml
<Inverter>
  <CheckSomething/>
</Inverter>
```

### Switch/Case Pattern
```xml
<Switch variable="{state}" case_1="IDLE" case_2="ACTIVE">
  <AlwaysSuccess/>   <!-- case_1: IDLE -->
  <AlwaysFailure/>   <!-- case_2: ACTIVE -->
  <AlwaysSuccess/>   <!-- default -->
</Switch>
```

## 5. Alternative XML Structure

Instead of inline conditions, structure your XML to use simpler logic:

```xml
<!-- Instead of complex inline condition -->
<Fallback>
  <!-- Try to resume if interrupted -->
  <Sequence>
    <CheckGoalInterrupted was_interrupted="{nav_interrupted}"/>
    <NavigateToPose target_pose="{saved_target}"/>
  </Sequence>
  
  <!-- Otherwise start new navigation -->
  <NavigateToPose target_pose="{new_target}"/>
</Fallback>
```

## Best Practices

1. **Keep conditions simple**: Complex logic belongs in C++ nodes, not XML
2. **Use descriptive names**: `CheckGoalInterrupted` vs `CheckSomeBooleanThing`
3. **Leverage blackboard**: Share state between nodes via blackboard variables
4. **Prefer composition**: Combine simple nodes rather than creating complex ones
5. **Document ports**: Always provide clear descriptions in `providedPorts()`

## Documentation References

- [BehaviorTree.CPP Official Docs](https://www.behaviortree.dev/)
- [Creating Custom Nodes](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_02_basic_nodes)
- [Using the Blackboard](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_blackboard)
- [XML Schema Reference](https://www.behaviortree.dev/docs/learn-the-basics/bt_basics)

## Common Mistakes to Avoid

- ❌ Using non-existent attributes like `_cppcode`
- ❌ Trying to embed C++ code directly in XML
- ❌ Making conditions too complex - break them down
- ❌ Not registering custom nodes with the factory
- ✅ Create simple, focused condition nodes
- ✅ Use clear, semantic names for your conditions
- ✅ Leverage the blackboard for state sharing
- ✅ Test your conditions in isolation first
