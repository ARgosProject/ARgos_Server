#ifndef DRAWIMAGESF_H
#define DRAWIMAGESF_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;
  class Communicator;

  class DrawImageSF : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args, Communicator& com) override;
  };

}

#endif
