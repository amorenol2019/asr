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

*   THIS SOFTWARE IS PROVsequenceED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCsequenceENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "sequence.h"

namespace bica
{
sequence::sequence() : state_(CARRETA), myBaseId_("sequence")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

sequence::~sequence()
{
}

void sequence::activateCode()
{
  	deactivateAllDeps();

	state_ = CARRETA;
	state_ts_ = ros::Time::now();

	carreta_activateDeps();
	carreta_code_once();

}

bool sequence::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case CONTENEDOR:

	contenedor_code_iterative();

	msg.data = "contenedor";
	if(contenedor_2_esquina())
	{

	deactivateAllDeps();

	state_ = ESQUINA;
	state_ts_ = ros::Time::now();

	esquina_activateDeps();
	esquina_code_once();
	}
	state_pub_.publish(msg);
	break;

	case CAJAS:

	cajas_code_iterative();

	msg.data = "cajas";
	if(cajas_2_contenedor())
	{

	deactivateAllDeps();

	state_ = CONTENEDOR;
	state_ts_ = ros::Time::now();

	contenedor_activateDeps();
	contenedor_code_once();
	}
	state_pub_.publish(msg);
	break;

	case ESQUINA:

	esquina_code_iterative();

	msg.data = "esquina";
	if(esquina_2_carreta())
	{

	deactivateAllDeps();

	state_ = CARRETA;
	state_ts_ = ros::Time::now();

	carreta_activateDeps();
	carreta_code_once();
	}
	state_pub_.publish(msg);
	break;

	case CARRETA:

	carreta_code_iterative();

	msg.data = "carreta";
	if(carreta_2_cajas())
	{

	deactivateAllDeps();

	state_ = CAJAS;
	state_ts_ = ros::Time::now();

	cajas_activateDeps();
	cajas_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
sequence::deactivateAllDeps()
{
};

void
sequence::contenedor_activateDeps()
{
}

void
sequence::cajas_activateDeps()
{
}

void
sequence::esquina_activateDeps()
{
}

void
sequence::carreta_activateDeps()
{
}



} /* namespace bica */
