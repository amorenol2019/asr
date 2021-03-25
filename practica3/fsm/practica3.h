
#ifndef PRACTICA3_H_
#define PRACTICA3_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class practica3: public bica::Component
{
public:
  practica3();
  virtual ~practica3();

  void activateCode();

  	virtual void ToYellGoal_code_iterative() {};
	virtual void ToYellGoal_code_once() {};
	virtual void ToBlueGoal_code_iterative() {};
	virtual void ToBlueGoal_code_once() {};
	virtual void Turn_code_iterative() {};
	virtual void Turn_code_once() {};
	virtual void ToBall_code_iterative() {};
	virtual void ToBall_code_once() {};

  	virtual bool Turn_2_ToBall() {return false;};
	virtual bool ToBlueGoal_2_ToYellGoal() {return false;};
	virtual bool ToBall_2_ToBlueGoal() {return false;};
	virtual bool ToYellGoal_2_Turn() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void ToYellGoal_activateDeps();
	void ToBlueGoal_activateDeps();
	void Turn_activateDeps();
	void ToBall_activateDeps();


  	static const int TOYELLGOAL = 0;
	static const int TOBLUEGOAL = 1;
	static const int TURN = 2;
	static const int TOBALL = 3;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;

};

} /* namespace bica */

#endif /* PRACTICA3_H_ */
