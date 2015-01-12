#include "ScriptSentence.h"

#include <sstream>
#include <iostream>

#include "ScriptManager.h"
#include "ScriptFunction.h"

namespace argosServer {

  ScriptSentence::ScriptSentence(const std::string& scriptSentence)
    : _command("") {
    decouple(scriptSentence);
  }

  ScriptSentence::~ScriptSentence() {
    _args.clear();
  }

  void ScriptSentence::execute(Script& owner, const ScriptManager& scriptManager) {
    scriptManager._handlers.at(_command)->execute(owner, _args);
  }

  size_t ScriptSentence::numArgs() const {
    return _args.size();
  }

  const std::string& ScriptSentence::getCommand() const {
    return _command;
  }

  const std::vector<std::string>& ScriptSentence::getArgs() const {
    return _args;
  }

  void ScriptSentence::decouple(const std::string& scriptSentence) {
    std::string token;
    std::istringstream parser(scriptSentence);
    parser.ignore(1); // Ignore '>' character

    parser >> std::ws;
    std::getline(parser, _command, ' '); // Command name

    parser >> std::ws;
    while(std::getline(parser, token, ' ')) {
      _args.push_back(token.c_str()); // Arguments
      parser >> std::ws;
    }
  }

  ScriptSentence::ScriptSentence()
    : _command("") {

  }

  ScriptSentence::ScriptSentence(const ScriptSentence& sentence)
    : _command(sentence._command), _args(sentence._args) {

  }

  ScriptSentence& ScriptSentence::operator=(const ScriptSentence& sentence) {
    if(this != &sentence) {
      _command = sentence._command;
      _args = sentence._args;
    }

    return *this;
  }

  std::ostream& operator<<(std::ostream& os, const ScriptSentence& scriptSentence) {
    os << scriptSentence._command;
    for(const std::string& arg : scriptSentence._args) {
      os << " " << arg;
    }

    return os;
  }

}
