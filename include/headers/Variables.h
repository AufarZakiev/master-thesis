#ifndef PROJECT_VARIABLES_H
#define PROJECT_VARIABLES_H

#include <map>
#include <algorithm>

class Variables
{
public:
  static Variables& getInstance();

  bool setParam(const std::string& param_name, double value);

  bool getParam(const std::string& param_name, double& value_ref) const;

  Variables(const Variables&) = delete;
  Variables& operator=(Variables&) = delete;

private:
  Variables();
  ~Variables() = default;
  static Variables instance;
  std::map<std::string, std::optional<double>> storage;
};

class ValidatedVariables
{
public:
  ValidatedVariables(Variables v);

  bool getParam(const std::string& param_name, double& value_ref) const;

  ~ValidatedVariables() = default;
private:
  static ValidatedVariables validatedInstance;
  std::map<std::string, double> storage;
};

#endif  // PROJECT_VARIABLES_H
