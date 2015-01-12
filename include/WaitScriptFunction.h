#ifndef WAITSCRIPTFUNCTION_H
#define WAITSCRIPTFUNCTION_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;

  class WaitScriptFunction : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args) override;
  };

}

#endif
