#include "PlaySoundSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void PlaySoundSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::PLAY_SOUND));
    size += com.addChars(args[0].c_str(), 32);
    size += com.addInt(getArgAsInt(args[1]));

    _properties["bytes"] = std::to_string(size);
  }

}
