#include "DrawTextPanelSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawTextPanelSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_TEXT_PANEL));

    // Colour
    for(int i = 0; i < 3; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Font size
    size += com.addInt(getArgAsInt(args[3]));

    // Text
    size += com.addChars(args[4].c_str(), 32);

    std::cout << args[4].c_str() << std::endl;

    // Position
    for(int i = 5; i < 8; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Size
    for(int i = 8; i < 10; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    _properties["bytes"] = std::to_string(size);
  }

}
