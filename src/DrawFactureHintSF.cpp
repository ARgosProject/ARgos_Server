#include "DrawFactureHintSF.h"
#include "Script.h"
#include "Communicator.h"

namespace argosServer {

  void DrawFactureHintSF::execute(Script& owner, const std::vector<std::string>& args, Communicator& com) {
    int size = 0;

    /**
     * const glm::vec3& pos, const glm::vec2& size, const glm::vec4& colour,
     * const std::wstring& title, const std::vector<std::pair<std::wstring, glm::vec3>>& textBlocks
     */

    // Function identifier
    size += com.addInt(static_cast<int>(Communicator::CallingFunctionType::DRAW_FACTURE_HINT));

    // Position
    for(int i = 0; i < 3; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Size
    for(int i = 3; i < 5; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Colour
    for(int i = 5; i < 8; ++i)
      size += com.addFloat(getArgAsFloat(args[i]));

    // Title
    size += com.addChars(args[8].c_str(), 32);

    // Text block 1
    size += com.addChars(args[9].c_str(), 32);

    // Text block 2
    size += com.addChars(args[10].c_str(), 32);

    _properties["bytes"] = std::to_string(size);
  }

}
