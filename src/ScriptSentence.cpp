#include "ScriptSentence.h"

#include <sstream>
#include <iostream>

#include "ScriptManager.h"
#include "ScriptFunction.h"

namespace argosServer {

  ScriptSentence::ScriptSentence(const std::string& scriptSentence)
    : _sf(NULL), _command("") {
    decouple(scriptSentence);
  }

  ScriptSentence::~ScriptSentence() {
    _args.clear();
  }

  void ScriptSentence::execute(Script& owner, const ScriptManager& scriptManager, Communicator& com) {
    _sf = scriptManager._handlers.at(_command);
    _sf->execute(owner, _args, com);
  }

  size_t ScriptSentence::numArgs() const {
    return _args.size();
  }

  const std::string& ScriptSentence::getCommand() const {
    return _command;
  }

  const std::string& ScriptSentence::getArg(int index) const {
    return _args[index];
  }

  const std::vector<std::string>& ScriptSentence::getArgs() const {
    return _args;
  }

  ScriptFunction* ScriptSentence::getScriptFunction() {
    return _sf;
  }

  void ScriptSentence::decouple(const std::string& scriptSentence) {
    std::string token;
    std::istringstream parser(scriptSentence);

    // Ignore the '>' character
    parser.ignore(1);

    // Command name
    parser >> std::ws;
    std::getline(parser, _command, ' ');

    // Arguments
    parser >> std::ws;
    std::string ss = "";
    bool glue = false;
    while(std::getline(parser, token, ' ')) {
      if(token.find("\"") != std::string::npos) {
        ss += token + " ";
        if(glue) {
          glue = false;

          ss = ss.substr(1, ss.size()-3);
          _args.push_back(ss);
          ss = "";
        }
        else {
          glue = true;
        }
      }
      else {
        _args.push_back(token);
      }

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
