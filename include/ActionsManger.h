/**
   @file ActionsManager.h
   @brief 
   @author Manuel Hervas
   @date 07/2014
*/
#ifndef _ARGOS_ACTIONSMANAGER_H
#define _ARGOS_ACTIONSMANAGER_H
#include <opencv2/opencv.hpp>

// Singleton
#include "Singleton.h"

using namespace std;

/**
 * ARgos server side
 */
namespace argosServer{
  
  enum Action_Type {STARTUP,INTERACTION};
  

  class Action{
  public:
    
    /**
     * Default constructor 
     */
    Action(int idx, string desc, cv::Rect rect, Action_Type actType, string script):
      id(idx),
      description(desc),
      region(rect),
      actionType(actType),
      scriptName(script){}
    
    /**
     * Default destructor
     */
    ~Action();
    

    void write(FileStorage& fs) const { //Write serialization for this class
      fs << "{" << "id" << id << "description" << description << "region" << region << "actionType" << actionType << "scriptName" << scriptName << "}";
    }
    
    void read(const FileNode& node){  //Read serialization for this class
      id = (int)node["id"];
      description = (string)node["description"];
      region = (cv::Rect)node["region"];
      actionType = (int)node["actType"];
      scriptName = (string)node["scriptName"];
    }
    
  };
  
  /**
   * Adminitration the list of actions in papers 
   */
  class ActionsManger:: public Singleton<ActionsManager>{
    
  public:
    /**
     * Default constructor 
     */
    ActionsManger();

    /**
     * Default destructor
     */
    ~ActionsManger();
    
    /**
     * Obtains the list of actions for the "id" paper
     * @param id actions list of this paper
     */
    vector<Action> getActions(int id);
    
    
  private:
    loadFile(string filename);
    saveFile(string filename);
    
    
    
    vector<vector<Action> > actionsList;
    
    
  };

}
    
#endif







  


