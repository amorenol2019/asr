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

*   THIS SOFTWARE IS PROVPractica3ImplED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCPractica3ImplENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "Practica3Impl.h"

namespace bica
{
Practica3Impl::Practica3Impl() : state_(FORWARD_BALL), myBaseId_("Practica3Impl")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

Practica3Impl::~Practica3Impl()
{
}

void Practica3Impl::activateCode()
{
  	deactivateAllDeps();

	state_ = FORWARD_BALL;
	state_ts_ = ros::Time::now();

	forward_ball_activateDeps();
	forward_ball_code_once();

}

bool Practica3Impl::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case FORWARD_YELLOW:

	forward_yellow_code_iterative();

	msg.data = "forward_yellow";
	if(forward_yellow_2_turn())
	{

	deactivateAllDeps();

	state_ = TURN;
	state_ts_ = ros::Time::now();

	turn_activateDeps();
	turn_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FORWARD_BALL:

	forward_ball_code_iterative();

	msg.data = "forward_ball";
	if(forward_ball_2_forward_blue())
	{

	deactivateAllDeps();

	state_ = FORWARD_BLUE;
	state_ts_ = ros::Time::now();

	forward_blue_activateDeps();
	forward_blue_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FORWARD_BLUE:

	forward_blue_code_iterative();

	msg.data = "forward_blue";
	if(forward_blue_2_forward_yellow())
	{

	deactivateAllDeps();

	state_ = FORWARD_YELLOW;
	state_ts_ = ros::Time::now();

	forward_yellow_activateDeps();
	forward_yellow_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TURN:

	turn_code_iterative();

	msg.data = "turn";
	if(turn_2_forward_ball())
	{

	deactivateAllDeps();

	state_ = FORWARD_BALL;
	state_ts_ = ros::Time::now();

	forward_ball_activateDeps();
	forward_ball_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
Practica3Impl::deactivateAllDeps()
{
	removeDependency("Turn");
	removeDependency("Forward");
};

void
Practica3Impl::forward_yellow_activateDeps()
{
	addDependency("Forward");
}

void
Practica3Impl::forward_ball_activateDeps()
{
	addDependency("Forward");
}

void
Practica3Impl::forward_blue_activateDeps()
{
	addDependency("Forward");
}

void
Practica3Impl::turn_activateDeps()
{
	addDependency("Turn");
}



} /* namespace bica */
