/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVpractica_3implED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCpractica_3implENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "practica_3impl.h"

namespace bica
{
practica_3impl::practica_3impl() : state_(TOBALL), myBaseId_("practica_3impl")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

practica_3impl::~practica_3impl()
{
}

void practica_3impl::activateCode()
{
  	deactivateAllDeps();

	state_ = TOBALL;
	state_ts_ = ros::Time::now();

	ToBall_activateDeps();
	ToBall_code_once();

}

bool practica_3impl::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case TURN:

	Turn_code_iterative();

	msg.data = "Turn";
	if(Turn_2_ToBall())
	{

	deactivateAllDeps();

	state_ = TOBALL;
	state_ts_ = ros::Time::now();

	ToBall_activateDeps();
	ToBall_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TOBLUEGOAL:

	ToBlueGoal_code_iterative();

	msg.data = "ToBlueGoal";
	if(ToBlueGoal_2_ToYellGoal())
	{

	deactivateAllDeps();

	state_ = TOYELLGOAL;
	state_ts_ = ros::Time::now();

	ToYellGoal_activateDeps();
	ToYellGoal_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TOBALL:

	ToBall_code_iterative();

	msg.data = "ToBall";
	if(ToBall_2_ToBlueGoal())
	{

	deactivateAllDeps();

	state_ = TOBLUEGOAL;
	state_ts_ = ros::Time::now();

	ToBlueGoal_activateDeps();
	ToBlueGoal_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TOYELLGOAL:

	ToYellGoal_code_iterative();

	msg.data = "ToYellGoal";
	if(ToYellGoal_2_Turn())
	{

	deactivateAllDeps();

	state_ = TURN;
	state_ts_ = ros::Time::now();

	Turn_activateDeps();
	Turn_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
practica_3impl::deactivateAllDeps()
{
	removeDependency("Turn");
	removeDependency("Forward");
	removeDependency("Perception");
};

void
practica_3impl::Turn_activateDeps()
{
	addDependency("Turn");
}

void
practica_3impl::ToBlueGoal_activateDeps()
{
	addDependency("Perception");
	addDependency("Forward");
}

void
practica_3impl::ToBall_activateDeps()
{
	addDependency("Forward");
	addDependency("Perception");
}

void
practica_3impl::ToYellGoal_activateDeps()
{
	addDependency("Perception");
	addDependency("Forward");
}



} /* namespace bica */
