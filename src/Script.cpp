#include "Script.h"
#include "ScriptManager.h"
#include "Log.h"

#include <fstream>

namespace argosServer {

  Script::Script(ScriptManager& scriptManager)
    : _scriptManager(scriptManager), _enabled(false), _repeat(true), _secondsToWait(0.0f), _currentSentence(0) {

  }

  Script::~Script() {
    _properties.clear();
  }

  const std::string& Script::getProperty(const std::string& key) {
    return _properties[key];
  }

  void Script::setProperty(const std::string& key, const std::string& value) {
    _properties[key] = value;
  }

  std::vector<ScriptSentence>& Script::getSentences() {
    return _sentences;
  }

  int Script::getNumberOfSentences() const {
    return _sentences.size();
  }

  void Script::setSecondsToWait(float secondsToWait) {
    if(secondsToWait != 0.0f)
      _timer.start();

    _secondsToWait = secondsToWait;
  }

  int Script::load(const std::string& path) {
    int num_lines = 0;

    if(!_sentences.empty())
      _sentences.clear();

    std::ifstream file;
    try {
      file.open(path);

      if(!file)
        throw std::ios::failure("There was an error opening the script file: " + path);

      std::string line;
      while(std::getline(file, line)) {
        ++num_lines;

        switch(line[0]) {
        case '!':
          // Stop parsing
          return num_lines;
        case '#':
          // Comment - Ignore it
          break;
        case '>':
          // Command - Process it
          _sentences.push_back(ScriptSentence(line));
          break;
        }
      }
    } catch(const std::exception& e) {
      Log::error(e.what());
    }

    return num_lines;
  }

  void Script::enable(bool enabled) {
    _enabled = enabled;
  }

  /*void Script::update() {
    if(!_enabled)
      return;

    if(_secondsToWait != 0.0f) {
      if(_timer.getMilliseconds() >= (_secondsToWait * 1000)) {
        _secondsToWait = 0.0f;
        runOneSentence();
        return;
      }
    }

    runOneSentence();
    }*/

  /*void Script::runOneSentence() {
    if(_sentences.empty())
      return;

    _sentences[_currentSentence++].execute(*this, _scriptManager);

    if(_currentSentence >= _sentences.size()) {
      _currentSentence = 0;
      if(!_repeat)
        _enabled = false;
    }
    }*/

}
