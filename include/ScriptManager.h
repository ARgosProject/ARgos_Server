#ifndef SCRIPTMANAGER_H
#define SCRIPTMANAGER_H

#include <vector>
#include <map>
#include <string>
#include <functional>

#include "Singleton.h"

namespace argosServer {

  class Script;
  class ScriptFunction;

  class ScriptManager : public Singleton<ScriptManager> {
    friend class ScriptSentence;

  public:
    ScriptManager();
    ~ScriptManager();

    void loadScripts(const std::string& path);
    void runScript(int id);
    //void update();

    Script& getScript(int id);

  private:
    void fillHandlers();

  private:
    std::map<std::string, ScriptFunction*> _handlers;
    std::map<int, Script*> _scripts;
  };

}

#endif
