#include "PlaySoundDelayedSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void PlaySoundDelayedSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::PLAY_SOUND_DELAYED));

    // Filename
    size += com.addChars(args[0].c_str(), 32);

    // Delay
    size += com.addFloat(getArgAsFloat(args[1]));

    _properties["bytes"] = std::to_string(size);
  }

}
