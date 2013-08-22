/*
 * Qiaosong Wang
 * University of Delaware
 *
 */
#ifndef PLANT_FLAG_TOOL_H
#define PLANT_FLAG_TOOL_H

#include <string>
#include <rviz/tool.h>
#include <rviz/properties/string_property.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "rviz/tool.h"

#include <QCursor>
#include <QObject>
#include <QMenu>

#include "rviz/tool_manager.h"


namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class StringProperty;
class BoolProperty;
}

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class UDCursorTool: public rviz::Tool
{
Q_OBJECT
public:
  UDCursorTool();
  ~UDCursorTool();


  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:
  void updateTopic();
  
  void menu_move_selected(); 
  void menu_delete_selected(); 
  void menu_delete_all();

 private:
  void makeFlag( const Ogre::Vector3& position );

  std::vector<Ogre::SceneNode*> flag_nodes_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz::VectorProperty* current_flag_property_;
  rviz::StringProperty* topic_property_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  bool context_menu_visible;

 protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;
  char *status_string;

  boost::shared_ptr<QMenu> tool_menu;

};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H
