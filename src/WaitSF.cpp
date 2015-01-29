#include "WaitSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void WaitSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    float timeToWait = getArgAsFloat(args[0]);

    owner.setSecondsToWait(timeToWait);
  }

}
