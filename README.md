ud_cursor
=========
This is a tool plugin for RVIZ for the user to continously click and publish 3D points.  It is based on the rviz_plugin_tutorials PlantFlagTool sample code.

To install: git clone in your catkin workspace src directory, then catkin_make to generate the RVIZ plugin.  In rviz, click the plus symbol ("Add a new tool") on the top right and choose the "ud_cursor" plugin.  The shortcut to activate the tool is "u" (click another tool to exit cursor click mode or simply hit 'm' to return to the default move camera tool).

To use:
Left-click on a point in the scene.  The cursor will change when a point is selected; clicking then will publish the point's 3-D coordinates on the "ud_clicked_point" topic.  If a point is not selected, a click will do nothing.

Contact Info:
Qiaosong Wang
qiaosong@udel.edu
