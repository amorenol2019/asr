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

*   THIS SOFTWARE IS PROVsequenced_destinationED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCsequenced_destinationENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
#ifndef SEQUENCED_DESTINATION_H_
#define SEQUENCED_DESTINATION_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class sequenced_destination : public bica::Component
{
public:
  sequenced_destination();
  virtual ~sequenced_destination();

  void activateCode();

  	virtual void carreta_code_iterative() {};
	virtual void carreta_code_once() {};
	virtual void esquina_code_iterative() {};
	virtual void esquina_code_once() {};
	virtual void caja_code_iterative() {};
	virtual void caja_code_once() {};
	virtual void contenedor_code_iterative() {};
	virtual void contenedor_code_once() {};

  	virtual bool esquina_2_contenedor() {return false;};
	virtual bool contenedor_2_caja() {return false;};
	virtual bool caja_2_carreta() {return false;};
	virtual bool carreta_2_esquina() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void carreta_activateDeps();
	void esquina_activateDeps();
	void caja_activateDeps();
	void contenedor_activateDeps();


  	static const int CARRETA = 0;
	static const int ESQUINA = 1;
	static const int CAJA = 2;
	static const int CONTENEDOR = 3;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* SEQUENCED_DESTINATION_H_ */
