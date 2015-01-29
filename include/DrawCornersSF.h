#ifndef DRAWCORNERSSF_H
#define DRAWCORNERSSF_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;
  class Communicator;

  class DrawCornersSF : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args, Communicator& com) override;
  };

}

#endif
