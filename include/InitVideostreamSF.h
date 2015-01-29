#ifndef INITVIDEOSTREAMSF_H
#define INITVIDEOSTREAMSF_H

#include <vector>
#include <string>

#include "ScriptFunction.h"

namespace argosServer {

  class Script;
  class Communicator;

  class InitVideostreamSF : public ScriptFunction {
  public:
    void execute(Script& owner, const std::vector<std::string>& args, Communicator& com) override;
  };

}

#endif
