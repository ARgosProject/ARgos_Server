#include "CheckRegionScriptFunction.h"
#include "Script.h"

namespace argosServer {

  void CheckRegionScriptFunction::execute(Script& owner, const std::vector<std::string>& args) {
    float timeToWait = getArgAsFloat(args[0]);

    owner.setSecondsToWait(timeToWait);
  }

}
