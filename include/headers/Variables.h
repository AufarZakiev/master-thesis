#ifndef PROJECT_VARIABLES_H
#define PROJECT_VARIABLES_H

#include <map>
#include <algorithm>

class Variables
{
public:
  Variables();

  bool setParam(const std::string& param_name, double value);

  bool getParam(const std::string& param_name, double& value_ref) const;

    ~Variables() = default;
private:
  std::map<std::string, std::optional<double>> storage;
};

class ValidatedVariables
{
public:
  explicit ValidatedVariables(const Variables& v);

  bool getParam(const std::string& param_name, double& value_ref) const;

  ~ValidatedVariables() = default;

  explicit operator Variables() const;
private:
  Variables validatedStorage;
};

#endif  // PROJECT_VARIABLES_H
