#include "DrawImageSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawImageSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_IMAGE));

    // Filename
    size += com.addChars(args[0].c_str(), 32);

    // Position
    for(int i = 1; i < 4; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Size
    for(int i = 4; i < 6; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    _properties["bytes"] = std::to_string(size);
  }

}
