#ifndef DRAWTEXTPANELSF_H
#define DRAWTEXTPANELSF_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;
  class Communicator;

  class DrawTextPanelSF : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args, Communicator& com) override;
  };

}

#endif
