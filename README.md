# SciRoc 2021 - Object Detection Module

## Authors

> Marco Gabriele Fedozzi <5083365@studednti.unige.it>  
> Francesco Ganci <4143910@studenti.unige.it>  
> Laura Triglia <4494106@studenti.unige.it>  

## Interface Action Server

```
# goal
int32 mode
string[] expected_tags
---
# result
string[] found_tags
bool asExpected
---
# feedback
string step
```

Where "mode" in the goal is an expression of what the interface AS should do, eg.
- "enumerate"
- "classify"
- "compare"
To which the module will respond by executing calling different actions served by other ObjDet components.

### Considerations on the "mode"

1. Using strings is easy to understand but lowers a bit the modularity (the Logic module has to use the exact same strings the ObjDet is using, which is not enforced at any level)
2. Using numbers would be a little more robust, but less intuitive and still technically not enforced
3. Is there a way to enforce a `mode` of a constrained value? Probably not directly in ROS (since ROS messages are but structs) but we could use a Class interface/wrapper shared amongst the whole project, acting similar to
```
// definition
class ObjDetWrapper
{
  stuff stuff stuff
}

...
// From within the Logic module code //

main ()
{
 
  std::shared_ptr<ObjDetWrapper> odw = std::make_shared<ObjDetWrapper>();
  
  std::vector<string> exp_tags;
  ...
  odw->setExpectedTags(exp_tags);
  odw->setModeCompare();
  action_client.sendGoal(odw->getGoalDef());

}
...
// From within the ObjDet module's interface code //

 // Define AS class
  ...
  executeCB(...)
  {
    ...
    switch (goal->mode)
    {
      case ObjDetWrapper::ENUMERATE:
        // stuff
      case ObjDetWrapper::CLASSIFY:
        // stuff
      case ObjDetWrapper::COMPARE:
        // stuff
    }
  }
```
