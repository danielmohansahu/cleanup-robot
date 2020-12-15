/**
*  @file user_panel.h
 * @brief Implementation of custom RVIZ user panel for control
 *
 * @copyright [2020] <Daniel Sahu, Spencer Elyard, Santosh Kesani>
 */

/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#ifndef CONTROLLER_SRC_USER_PANEL_H
#define CONTROLLER_SRC_USER_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <actionlib/client/simple_action_client.h>

# include <rviz/panel.h>

# include <controller/SetModeAction.h>
#endif

#include <string>

class QPushButton;

/**
* @brief Namespace for Cleanup Implementation
*/
namespace cleanup {

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// TeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.

/**
* @brief Implementation for a custom user panel in RVIZ
*/
class UserPanel: public rviz::Panel {
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.

  /**
  * @brief Constructor
  */
  explicit UserPanel(QWidget* parent = 0);

public Q_SLOTS:
  /**
  * @brief Call Explore function
  */
  void explore();

  /**
  * @brief Call Clean function
  */
  void clean();

  /**
  * @brief Call Stop function
  */
  void stop();

protected:
  void sendGoal(const std::string& mode);

  QPushButton* explore_button_;
  QPushButton* clean_button_;
  QPushButton* stop_button_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // service clients for various behaviors
  actionlib::SimpleActionClient<controller::SetModeAction> client_;
};

} // end namespace cleanup

#endif // CONTROLLER_SRC_USER_PANEL_H
