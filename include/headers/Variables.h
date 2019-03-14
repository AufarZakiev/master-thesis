#ifndef PROJECT_VARIABLES_H
#define PROJECT_VARIABLES_H

#include <map>

class Variables
{
public:
  static Variables& getInstance();

  bool setParam(const std::string& param_name, double value);

  bool getParam(const std::string& param_name, double& value_ref) const;

  Variables(const Variables&) = delete;
  Variables& operator=(Variables&) = delete;

protected:
  Variables();
  static Variables instance;
  std::map<std::string, std::optional<double>> storage;
};

#endif  // PROJECT_VARIABLES_H
