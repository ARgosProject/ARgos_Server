#include "ScriptManager.h"
#include "Script.h"
#include "ScriptFunction.h"

#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"

#include "WaitScriptFunction.h"
#include "Log.h"

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

    rapidxml::file<> xml_file(path.c_str());
    rapidxml::xml_document<> xml_doc;
    rapidxml::xml_node<>* xml_root;
    rapidxml::xml_node<>* node;

    xml_doc.parse<0>(xml_file.data());
    xml_root = xml_doc.first_node("scripts");

    node = xml_root->first_node("script");
    while(node) {
      Script* script = new Script(*this);

      std::string script_name = node->first_attribute("name")->value();
      std::string script_filename = node->first_attribute("filename")->value();

      Log::info("Cargando script '" + script_name + "' desde 'data/" + script_filename + "'");

      script->setProperty("name", script_name);
      script->setProperty("filename", script_filename);
      script->setProperty("description", node->first_attribute("description")->value());
      int num_lines = script->load("data/" + script_filename);

      if(num_lines > 0)
        Log::success("Cargadas " + std::to_string(num_lines) + " sentencias del script '" + script_name + "'");
      else
        Log::error("Cargadas " + std::to_string(num_lines) + " sentencias del script '" + script_name + "'");

      _scripts[script_name] = script;

      node = node->next_sibling("script");
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
