#include "ScriptManager.h"
#include "Script.h"
#include "ScriptFunction.h"
#include "Log.h"
#include "ConfigManager.h"

#include "WaitSF.h"
#include "DrawImageSF.h"
#include "DrawVideoSF.h"
#include "DrawCornersSF.h"
#include "DrawAxisSF.h"
#include "InitVideostreamSF.h"
#include "DrawTextPanelSF.h"
#include "DrawHighlightSF.h"
#include "DrawButtonSF.h"
#include "DrawFactureHintSF.h"
#include "PlaySoundSF.h"
#include "PlaySoundDelayedSF.h"
#include "CheckRegionSF.h"

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

    const std::vector<std::pair<int, string>>& script_files = ConfigManager::getScriptsList();
    for(auto& script_file : script_files) {
      Log::info("Loading script '" + script_file.second + "' from '" + path + script_file.second + "'");

      Script* script = new Script(*this);
      script->setProperty("id", std::to_string(script_file.first));
      script->setProperty("filename", script_file.second);
      int num_lines = script->load(path + script_file.second);

      if(num_lines > 0)
        Log::success("Loaded " + std::to_string(script->getNumberOfSentences())
                     + " sentences (" + std::to_string(num_lines) + " lines read) from script '" + script_file.second
                     + "' associated with document '" + std::to_string(script_file.first) + "'");
      else
        Log::error("Loaded " + std::to_string(script->getNumberOfSentences())
                   + " sentences (" + std::to_string(num_lines) + " lines read) from script '" + script_file.second
                   + "' associated with document '" + std::to_string(script_file.first) + "'");

      _scripts[script_file.first] = script;
    }
  }

  void ScriptManager::runScript(int id) {
    _scripts[id]->enable(true);
  }

  /*void ScriptManager::update() {
    for(auto& script : _scripts)
      script.second->update();
      }*/

  Script& ScriptManager::getScript(int id) {
    return *_scripts[id];
  }

  void ScriptManager::fillHandlers() {
    _handlers["Wait"] = new WaitSF;
    _handlers["DrawImage"] = new DrawImageSF;
    _handlers["DrawVideo"] = new DrawVideoSF;
    _handlers["DrawCorners"] = new DrawCornersSF;
    _handlers["DrawAxis"] = new DrawAxisSF;
    _handlers["InitVideostream"] = new InitVideostreamSF;
    _handlers["DrawTextPanel"] = new DrawTextPanelSF;
    _handlers["DrawHighlight"] = new DrawHighlightSF;
    _handlers["DrawButton"] = new DrawButtonSF;
    _handlers["DrawFactureHint"] = new DrawFactureHintSF;
    _handlers["PlaySound"] = new PlaySoundSF;
    _handlers["PlaySoundDelayed"] = new PlaySoundDelayedSF;
    _handlers["CheckRegion"] = new CheckRegionSF;
  }

}
