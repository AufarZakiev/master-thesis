#ifndef PROJECT_VARIABLES_H
#define PROJECT_VARIABLES_H

#include <map>

class Variables
{
public:
  static Variables& getInstance();

  bool getParam(std::string param_name, double& value_ref) const;

  bool setParam(std::string, double value);

protected:
  Variables();
  Variables(const Variables&) = delete;
  Variables& operator=(Variables&) = delete;
  static Variables instance;
  std::map<std::string, double> storage;
};

#endif  // PROJECT_VARIABLES_H
