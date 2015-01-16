#ifndef CHECKREGIONSCRIPTFUNCTION_H
#define CHECKREGIONSCRIPTFUNCTION_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;

  class CheckRegionScriptFunction : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args) override;
  };

}

#endif
