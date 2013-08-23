/*
 * Qiaosong Wang
 * University of Delaware
 *
 */
#ifndef UD_CURSOR_TOOL_H
#define UD_CURSOR_TOOL_H

#include <string>
#include <rviz/tool.h>
#include <rviz/properties/string_property.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include "rviz/tool.h"

#include <QCursor>
#include <QObject>
#include <QMenu>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>

#include "rviz/tool_manager.h"


#include "ud_cursor/UDCursor.h"


#define UD_CURSOR_EDIT_MODE_ADD         0
#define UD_CURSOR_EDIT_MODE_MOVE        1
#define UD_CURSOR_EDIT_MODE_DELETE      2
#define UD_CURSOR_EDIT_MODE_DELETE_ALL  3

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

  Ogre::Plane selection_plane;

public Q_SLOTS:
  void updateTopic();
  
  void menu_edit_add();
  void menu_edit_move_selected(); 
  void menu_edit_delete_selected(); 
  void menu_edit_delete_all();

  void menu_display_indices();
  void menu_display_connections();

  void menu_plane_selection();

 private:
  void makeFlag( const Ogre::Vector3& position );

  std::vector<Ogre::SceneNode*> flag_nodes_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz::VectorProperty* current_flag_property_;
  rviz::StringProperty* topic_property_;
  ros::NodeHandle nh_;
  //  ros::Publisher pub_;
  ros::Publisher cursor_pub;
  ros::Subscriber selection_plane_sub;

  bool use_selection_plane;

  QAction *action_add;
  QAction *action_move_selected;
  QAction *action_delete_selected;
  QAction *action_delete_all;
  QAction *action_display_indices;
  QAction *action_display_connections;
  QAction *action_plane_selection;

  ud_cursor::UDCursor cursor_msg;

 protected:
  QCursor std_cursor_;
  QCursor hit_cursor_;
  QCursor plane_hit_cursor_;
  QCursor move_cursor_;
  QCursor delete_cursor_;
  char *status_string;

  boost::shared_ptr<QMenu> tool_menu;

};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // UD_CURSOR_TOOL_H
