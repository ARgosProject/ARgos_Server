#include "DrawHighlightSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawHighlightSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_HIGHLIGHT));

    // Colour
    for(int i = 0; i < 3; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Position
    for(int i = 3; i < 6; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Size
    for(int i = 6; i < 8; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    _properties["bytes"] = std::to_string(size);
  }

}
