#include "ScriptManager.h"
#include "Script.h"
#include "ScriptFunction.h"
#include "WaitScriptFunction.h"
#include "Log.h"
#include "ConfigManager.h"

namespace argosServer {

  ScriptManager::ScriptManager() {
    fillHandlers();
  }

  ScriptManager::~ScriptManager() {
    for(auto& script : _scripts) {
      delete script.second;
    }

    for(auto& handler : _handlers) {
      delete handler.second;
    }

    _scripts.clear();
    _handlers.clear();
  }

  void ScriptManager::loadScripts(const std::string& path) {
    if(!_scripts.empty())
      _scripts.clear();

    const std::vector<std::string>& script_file_names = ConfigManager::getScriptsList();
    size_t size = script_file_names.size();

    for(int i = 0; i < size; ++i) {
      Log::info("Loading script '" + script_file_names[i] + "' from '" + path + script_file_names[i] + "'");

      Script* script = new Script(*this);
      script->setProperty("id", std::to_string(i));
      script->setProperty("filename", script_file_names[i]);
      int num_lines = script->load(path + script_file_names[i]);

      if(num_lines > 0)
        Log::success("Loaded " + std::to_string(num_lines) + " sentences from script '" + script_file_names[i] + "'");
      else
        Log::error("Loaded " + std::to_string(num_lines) + " sentences from script '" + script_file_names[i] + "'");

      _scripts[script_file_names[i]] = script;
    }
  }

  void ScriptManager::runScript(const std::string& name) {
    _scripts[name]->enable(true);
  }

  void ScriptManager::update() {
    for(auto& script : _scripts)
      script.second->update();
  }

  void ScriptManager::fillHandlers() {
    _handlers["Wait"] = new WaitScriptFunction;
  }

}
