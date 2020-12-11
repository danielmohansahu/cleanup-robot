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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>

#include <geometry_msgs/Twist.h>

#include "user_panel.h"

namespace cleanup
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
UserPanel::UserPanel( QWidget* parent )
  : rviz::Panel( parent ),
    client_("controller/set_mode")
{
  // add buttons to send specific goals
  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( new QLabel( "Behaviors:" ));

  // explore behavior
  explore_button_ = new QPushButton("Explore");
  button_layout->addWidget( explore_button_ );

  // clean behavior
  clean_button_ = new QPushButton("Clean");
  button_layout->addWidget( clean_button_ );

  // stop behavior
  stop_button_ = new QPushButton("Stop");
  button_layout->addWidget( stop_button_ );

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( button_layout );
  setLayout( layout );

  // Next we make signal/slot connections.
  connect(explore_button_, SIGNAL(clicked()), this, SLOT(explore()));
  connect(clean_button_, SIGNAL(clicked()), this, SLOT(clean()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
}

void UserPanel::explore()
{
  sendGoal("explore");
}

void UserPanel::clean()
{
  sendGoal("clean");
}

void UserPanel::stop()
{
  client_.cancelAllGoals();
}

// Send a navigation goal.
void UserPanel::sendGoal(const std::string& mode)
{
  // construct goal object
  controller::SetModeGoal goal;
  goal.mode = mode;
  client_.sendGoal(goal);
}

} // end namespace cleanup

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cleanup::UserPanel, rviz::Panel)
