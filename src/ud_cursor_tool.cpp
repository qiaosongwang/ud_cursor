/*
 * Qiaosong Wang
 * University of Delaware
 *
 */

/*
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreScehneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>
*/

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/default_plugin/tools/focus_tool.h>
#include <rviz/render_panel.h>

#include "ud_cursor_tool.h"
#include <geometry_msgs/PointStamped.h>
#include <shape_msgs/Plane.h>

#include "ud_imarker/UDMeasurement.h"
//#include "include/ud_imarker.hh"

#include <sstream>

float globalx,globaly,globalz;


namespace rviz_plugin_tutorials
{

UDCursorTool *ud_cursor_tool = NULL;

//----------------------------------------------------------------------------

// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we also set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.

UDCursorTool::UDCursorTool() : moving_flag_node_( NULL ), current_flag_property_( NULL )
{
  shortcut_key_ = 'u';

  status_string = (char *) malloc(256 * sizeof(char));

  use_selection_plane = false;
  selection_plane = Ogre::Plane(0, 0, 1, 0);   // default is nominal ground plane
                                                   // overridden if subscription comes in

  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_ADD;
  cursor_msg.DisplayIndices = 0;
  cursor_msg.DisplayConnections = 0;

  QAction *separator1 = new QAction(this);
  separator1->setSeparator(true);

  QAction *separator2 = new QAction(this);
  separator2->setSeparator(true);

  QAction *separator3 = new QAction(this);
  separator3->setSeparator(true);

  QAction *separator4 = new QAction(this);
  separator4->setSeparator(true);

  action_add = new QAction(tr("&Add/select point"), this);
  action_move_selected = new QAction(tr("&Move selected point"), this);
  action_delete_selected = new QAction(tr("&Delete point"), this);
  action_delete_all = new QAction(tr("Delete all points"), this);

  action_display_indices = new QAction(tr("Display indices"), this);
  action_display_indices->setCheckable(true);
  action_display_indices->setChecked(cursor_msg.DisplayIndices);

  action_display_connections = new QAction(tr("Display connections"), this);
  action_display_connections->setCheckable(true);
  action_display_connections->setChecked(cursor_msg.DisplayConnections);

  action_plane_selection = new QAction(tr("Plane selection"), this);
  action_plane_selection->setCheckable(true);
  action_plane_selection->setChecked(use_selection_plane);

  tool_menu.reset(new QMenu);
  tool_menu->addAction(action_add);
  tool_menu->addAction(separator1);
  tool_menu->addAction(action_move_selected);
  tool_menu->addAction(separator2);
  tool_menu->addAction(action_delete_selected);
  tool_menu->addAction(action_delete_all);
  tool_menu->addAction(separator3);
  tool_menu->addAction(action_display_indices);
  tool_menu->addAction(action_display_connections);
  tool_menu->addAction(separator4);
  tool_menu->addAction(action_plane_selection);

  connect(action_add, SIGNAL(triggered()), this, SLOT(menu_edit_add()));
  connect(action_move_selected, SIGNAL(triggered()), this, SLOT(menu_edit_move_selected()));
  connect(action_delete_selected, SIGNAL(triggered()), this, SLOT(menu_edit_delete_selected()));
  connect(action_delete_all, SIGNAL(triggered()), this, SLOT(menu_edit_delete_all()));

  connect(action_display_indices, SIGNAL(triggered()), this, SLOT(menu_display_indices()));
  connect(action_display_connections, SIGNAL(triggered()), this, SLOT(menu_display_connections()));

  connect(action_plane_selection, SIGNAL(triggered()), this, SLOT(menu_plane_selection()));

}

//----------------------------------------------------------------------------

// The destructor destroys the Ogre scene nodes for the flags so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.

UDCursorTool::~UDCursorTool()
{
/*
  for( unsigned i = 0; i < flag_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( flag_nodes_[ i ]);
  }
*/
}

//----------------------------------------------------------------------------
// CONTEXT MENU ACTIONS
//----------------------------------------------------------------------------

void UDCursorTool::menu_edit_add()
{
  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_ADD;
}

//----------------------------------------------------------------------------

void UDCursorTool::menu_edit_move_selected() 
{
  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_MOVE;
}

//----------------------------------------------------------------------------

void UDCursorTool::menu_edit_delete_selected() 
{
  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_DELETE;
}


//----------------------------------------------------------------------------

void UDCursorTool::menu_edit_delete_all() 
{
  // do it immediately 

  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_DELETE_ALL;
  cursor_pub.publish(cursor_msg);

  cursor_msg.EditMode = UD_CURSOR_EDIT_MODE_ADD;
}

//----------------------------------------------------------------------------

void UDCursorTool::menu_display_indices()
{
  cursor_msg.DisplayIndices = action_display_indices->isChecked();
  cursor_pub.publish(cursor_msg);
}

//----------------------------------------------------------------------------

void UDCursorTool::menu_display_connections()
{
  cursor_msg.DisplayConnections = action_display_connections->isChecked();
  cursor_pub.publish(cursor_msg);
}

//----------------------------------------------------------------------------

void UDCursorTool::menu_plane_selection()
{
  use_selection_plane = action_plane_selection->isChecked();
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

//void selectionPlaneCallback(const shape_msgs::Plane & plane_msg)
void selectionPlaneCallback(const ud_imarker::UDMeasurement & measurement_msg)
{
  if (measurement_msg.Category == MEASUREMENT_TYPE_PLANE) {

    printf("da plane!  da plane!\n"); fflush(stdout);

    ud_cursor_tool->selection_plane = Ogre::Plane(measurement_msg.Value[0],
						  measurement_msg.Value[1],
						  measurement_msg.Value[2],
						  measurement_msg.Value[3]);

    /*
  ud_cursor_tool->selection_plane = Ogre::Plane(plane_msg.coef[0],
						plane_msg.coef[1],
						plane_msg.coef[2],
						plane_msg.coef[3]);
    */
  }
}

//----------------------------------------------------------------------------

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the flag, create an Ogre::SceneNode for the moving flag, and then
// set it invisible.

void UDCursorTool::onInitialize()
{
  ud_cursor_tool = this;

  std_cursor_ = rviz::getDefaultCursor();
  hit_cursor_ = rviz::makeIconCursor( "package://rviz/icons/crosshair.svg" );
  plane_hit_cursor_ = rviz::makeIconCursor( "package://rviz/icons/visibility.svg" );
  move_cursor_ = rviz::makeIconCursor( "package://rviz/icons/move2d.svg" );
  delete_cursor_ = rviz::makeIconCursor( "package://rviz/icons/forbidden.svg" );

/*
  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "UDCursorTool: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

*/

topic_property_ = new rviz::StringProperty( "Topic", "/ud_cursor_point",
                                        "The topic on which to publish points.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );


updateTopic();

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  //Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  //moving_flag_node_->attachObject( entity );
  moving_flag_node_->setVisible( false );

  selection_plane_sub = nh_.subscribe("ud_measurement", 10, selectionPlaneCallback);
}

//----------------------------------------------------------------------------

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.

void UDCursorTool::activate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( true );

   // current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
   // current_flag_property_->setReadOnly( true );
    //getPropertyContainer()->addChild( current_flag_property_ );

   
  }
}

//----------------------------------------------------------------------------

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.

void UDCursorTool::deactivate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( false );
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }
}

//----------------------------------------------------------------------------

void UDCursorTool::updateTopic()
{
  //  pub_ = nh_.advertise<geometry_msgs::PointStamped>( topic_property_->getStdString(), 1 );
  cursor_pub = nh_.advertise<ud_cursor::UDCursor>( topic_property_->getStdString(), 1 );
}

//----------------------------------------------------------------------------

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving flag to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current flag location.  Therefore we make a new flag at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.

int UDCursorTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_flag_node_ )
  {
    return Render;
  }

  if ( event.rightDown()) {
    
    printf("pop up context menu\n"); fflush(stdout);

    event.panel->showContextMenu(tool_menu);

    //    context_menu_visible = true;

    return Render;

  }
  else if ( event.rightUp() ) {

    printf("process menu selection\n"); fflush(stdout);

    //    context_menu_visible = false;

    return Render;

  }

 Ogre::Vector3 pos;
 bool success;

 // the plane intersection way

 if (use_selection_plane) {
   //   printf("%lf %lf %lf %lf\n", selection_plane.normal.x, selection_plane.normal.y, selection_plane.normal.z, selection_plane.d);
   success = rviz::getPointOnPlaneFromWindowXY( event.viewport,
						selection_plane,
						event.x, event.y, pos);
 }

 // the point selection way

 else
   success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );

  char status_char;

  //  setCursor( success ? hit_cursor_ : std_cursor_ );
  if (success) {
    if (cursor_msg.EditMode == UD_CURSOR_EDIT_MODE_ADD) {

      if (use_selection_plane)
	setCursor(plane_hit_cursor_);
      else
	setCursor(hit_cursor_);
      status_char = 'A';
    }
    else if (cursor_msg.EditMode == UD_CURSOR_EDIT_MODE_MOVE) {
      setCursor(move_cursor_);
      status_char = 'M';
    }
    else if (cursor_msg.EditMode == UD_CURSOR_EDIT_MODE_DELETE) {
      setCursor(delete_cursor_);
      status_char = 'D';
    }
  }
  else {
    setCursor(std_cursor_);
    status_char = ' ';
  }

  if ( success ) {
    
    sprintf(status_string, "<b>UD Cursor</b> %c [%.3lf, %.3lf, %.3lf]", status_char, pos.x, pos.y, pos.z);
    setStatus(status_string);

    /*
    std::ostringstream s;
    s << "<b>UD Cursor:</b> ";
    s.precision(3);
    s << " [" << pos.x << ", " << pos.y << ", " << pos.z << "]";
    
    setStatus( s.str().c_str() );
    */

    /*
    if (( pos.x!= globalx)&&( pos.y!= globaly)&&( pos.z!= globalz)) {
      s <<"<b>READY FOR A CLICK!<b>";
      setStatus( s.str().c_str() );
    }
    */

    if ( event.leftUp() ) {

      //      s << "<b>  Detect Click At:</b> ";
      //      s.precision(5);
      //      s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
      //      setStatus( s.str().c_str() );

      sprintf(status_string, "<b>UD Cursor</b> Sending %c [%.3lf, %.3lf, %.3lf]", status_char, pos.x, pos.y, pos.z);
      setStatus(status_string);

      /*
      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      ps.header.seq = edit_mode;
      printf("publishing with seq %i\n", ps.header.seq);
      pub_.publish( ps );
      */

      //      ud_cursor::UDCursor cursor_msg;
      cursor_msg.x = pos.x;
      cursor_msg.y = pos.y;
      cursor_msg.z = pos.z;
      cursor_msg.header.frame_id = context_->getFixedFrame().toStdString();
      cursor_msg.header.stamp = ros::Time::now();
      //      cursor_msg.EditMode = edit_mode;
      cursor_pub.publish( cursor_msg );
      
      globalx=pos.x;
      globaly=pos.y;
      globalz=pos.z;
      
      updateTopic();
    }

    /*
      if( event.leftUp() )
      {
      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      ps.header.seq = edit_mode;
      pub_.publish( ps );
      
      if ( auto_deactivate_property_->getBool() )
      {
      flags |= Finished;
      }
      }
    */
  }  // success
  else {
    sprintf(status_string, "<b>UD Cursor</b>");

    setStatus(status_string);
    //    setStatus("");
  }
  
/*

  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_flag_node_->setVisible( true );
    moving_flag_node_->setPosition( intersection );
    current_flag_property_->setVector( intersection );

    if( event.leftDown() )
    {
      makeFlag( intersection );
      current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
      return Render | Finished;
    }
  }
  else
  {
    moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
  }

*/

  return Render;
}

//----------------------------------------------------------------------------

// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.

void UDCursorTool::makeFlag( const Ogre::Vector3& position )
{

  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  flag_nodes_.push_back( node );
}

//----------------------------------------------------------------------------

// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.

void UDCursorTool::save( rviz::Config config ) const
{

/*
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( flag_config );
  }
*/
}

//----------------------------------------------------------------------------

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void UDCursorTool::load( const rviz::Config& config )
{
  // Here we get the "Flags" sub-config from the tool config and loop over its entries:
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
/*
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Flag " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( flag_config );
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
  }
*/
}

//----------------------------------------------------------------------------

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::UDCursorTool,rviz::Tool )
// END_TUTORIAL
