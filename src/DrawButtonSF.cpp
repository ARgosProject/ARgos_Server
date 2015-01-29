#include "DrawButtonSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawButtonSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_BUTTON));

    // Colour
    for(int i = 0; i < 3; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Text
    size += com.addChars(args[3].c_str(), 32);

    // Position
    for(int i = 4; i < 7; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    _properties["bytes"] = std::to_string(size);
  }

}
