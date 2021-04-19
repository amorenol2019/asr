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

*   THIS SOFTWARE IS PROVpractica4ED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCpractica4ENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "practica4.h"

namespace bica
{
practica4::practica4() : state_(FIRST), myBaseId_("practica4")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

practica4::~practica4()
{
}

void practica4::activateCode()
{
  	deactivateAllDeps();

	state_ = FIRST;
	state_ts_ = ros::Time::now();

	First_activateDeps();
	First_code_once();

}

bool practica4::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case SECOND:

	Second_code_iterative();

	msg.data = "Second";
	if(Second_2_Third())
	{

	deactivateAllDeps();

	state_ = THIRD;
	state_ts_ = ros::Time::now();

	Third_activateDeps();
	Third_code_once();
	}
	state_pub_.publish(msg);
	break;

	case LAST:

	Last_code_iterative();

	msg.data = "Last";
	if(Last_2_First())
	{

	deactivateAllDeps();

	state_ = FIRST;
	state_ts_ = ros::Time::now();

	First_activateDeps();
	First_code_once();
	}
	state_pub_.publish(msg);
	break;

	case THIRD:

	Third_code_iterative();

	msg.data = "Third";
	if(Third_2_Last())
	{

	deactivateAllDeps();

	state_ = LAST;
	state_ts_ = ros::Time::now();

	Last_activateDeps();
	Last_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FIRST:

	First_code_iterative();

	msg.data = "First";
	if(First_2_Second())
	{

	deactivateAllDeps();

	state_ = SECOND;
	state_ts_ = ros::Time::now();

	Second_activateDeps();
	Second_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
practica4::deactivateAllDeps()
{
	removeDependency("Navigate");
};

void
practica4::Second_activateDeps()
{
	addDependency("Navigate");
}

void
practica4::Last_activateDeps()
{
	addDependency("Navigate");
}

void
practica4::Third_activateDeps()
{
	addDependency("Navigate");
}

void
practica4::First_activateDeps()
{
	addDependency("Navigate");
}



} /* namespace bica */
