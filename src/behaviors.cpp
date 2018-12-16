#include <gravl/behaviors.h>

std::vector<Behavior> getBehaviors(ros::NodeHandle n)
{
  std::vector<std::string> params;
  n.getParamNames(params);
  std::vector<std::string> filteredParams (params.size());
  auto it = std::copy_if(params.begin(), params.end(), filteredParams.begin(),
               [](std::string param)
               {
                 if (param.length() < 11) return false;
                 return param.substr(0, 11) == "/behaviors/";
               });

  filteredParams.resize(std::distance(filteredParams.begin(),it));  // shrink container to new size
  std::vector<Behavior> behaviors;
  int counter = 0;
  Behavior behavior;
  std::vector<int> tmp;

  for(std::string param : filteredParams)
    // This takes advantage of the assumption that parameters come in alphabetical order.
    // First is up is the id, next is the priority after which the
    // behavior is complete and added to the vector.
    {
      n.getParam(param, tmp);
      behavior.name = param.substr(11,param.length());
      behavior.id = tmp[0];
      behavior.priority = tmp[1];
      behaviors.push_back(behavior);
    }
  return behaviors;
}
