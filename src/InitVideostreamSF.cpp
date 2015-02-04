#include "InitVideostreamSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void InitVideostreamSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::INIT_VIDEO_STREAM));

    // Filename
    size += com.addChars(args[0].c_str(), 32);

    // Size
    for(int i = 1; i < 3; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Port
    size += com.addInt(getArgAsInt(args[3]));

    // Init video conference server
    com.initVideoConference(args[3]);

    _properties["bytes"] = std::to_string(size);
  }

}
