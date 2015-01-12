#ifndef SCRIPTFUNCTION_H
#define SCRIPTFUNCTION_H

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace argosServer {

  class Script;

  class ScriptFunction {
  public:
    ScriptFunction() { }
    virtual ~ScriptFunction() { }

    virtual void execute(Script& owner, const std::vector<std::string>& args) = 0;

  protected:
    virtual int getArgAsInt(const std::string& arg) const {
      return std::stoi(arg);
    }

    virtual float getArgAsFloat(const std::string& arg) const {
      return std::stof(arg);
    }

    virtual bool getArgAsBool(std::string arg) const {
      std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
      std::istringstream is(arg);
      bool b;
      is >> std::boolalpha >> b;

      return b;
    }
  };

}

#endif
