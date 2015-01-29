#include "DrawAxisSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawAxisSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_AXIS));

    // Length
    size += com.addFloat(getArgAsFloat(args[0]));

    // Wide
    size += com.addFloat(getArgAsFloat(args[1]));

    // Position
    for(int i = 2; i < 5; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    _properties["bytes"] = std::to_string(size);
  }

}
