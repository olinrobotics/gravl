#include <ros/ros.h>
#include <string>
#include <vector>

struct Behavior
{
  std::string name;
  int priority;
  int id;
};

std::vector<Behavior> getBehaviors(ros::NodeHandle n)
{
  std::vector<std::string> params;
  std::vector<std::string> filteredParams;
  n.getParamNames(params);
  std::copy_if(params.begin(), params.end(), filteredParams.begin(),
               [](std::string param)
               {
                 return param.substr(0, 10) == "behaviors/";
               });

  std::vector<Behavior> behaviors;
  int counter = 0;
  Behavior behavior;
  int tmp;

  for(std::string param : params)
    // This takes advantage of the assumption that parameters come in alphabetical order.
    // First is up is the id, next is the priority after which the
    // behavior is complete and added to the vector.
    {
      n.getParam(param, tmp);
      if (counter == 0)
        {
          behavior.name = param.substr(10, param.length() - 3);
          behavior.id = tmp;
          ++counter;
        }
      if (counter == 1)
        {
          behavior.priority = tmp;
          behaviors.push_back(behavior);
          counter = 0;
        }
    }

  return behaviors;
}
