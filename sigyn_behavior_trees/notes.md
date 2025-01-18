```code

    BehaviorTreeFactory factory;
    factory.registerNodeType<Speaking>("FiboSeq");
```

`registerNodeType` takes a template argument, which is the class that you want to register. The string argument is the name of the corresponding element that will be used in the XML file.
