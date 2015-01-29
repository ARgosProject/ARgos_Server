#ifndef SCRIPT_H
#define SCRIPT_H

#include <map>
#include <string>

#include "Timer.h"
#include "ScriptSentence.h"

namespace argosServer {

  class ScriptManager;

  class Script {
  public:
    Script(ScriptManager& scriptManager);
    ~Script();

    const std::string& getProperty(const std::string& key);
    void setProperty(const std::string& key, const std::string& value);

    std::vector<ScriptSentence>& getSentences();
    int getNumberOfSentences() const;

    void setSecondsToWait(float secondsToWait);

    int load(const std::string& path);
    void enable(bool enabled);
    //void update();

  private:
    //void runOneSentence();

  private:
    ScriptManager& _scriptManager;

    bool _enabled;
    bool _repeat;
    std::vector<ScriptSentence> _sentences;
    std::map<std::string, std::string> _properties;
    Timer _timer;
    float _secondsToWait;
    unsigned int _currentSentence;
  };

}

#endif
