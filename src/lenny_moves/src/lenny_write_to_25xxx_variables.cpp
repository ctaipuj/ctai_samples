#include <ros/ros.h> //ros
#include <motoman_variables/motoman_variables.h>//required to write to the network inputs in lenny's controller

//You can only write or read a network input use ladder to activate external outputs 30xxx

int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "writing_to_lenny_i_o");
	ros::NodeHandle n;
	motoman_variables my_variable;
	
	my_variable.turnOn(25010);
	ros::Duration(3).sleep();
	my_variable.turnOff(0);
	ros::Duration(3).sleep();
	my_variable.turnOn(25011);
	ros::Duration(3).sleep();
	my_variable.setAddressValue(25010,1);
	my_variable.setAddressValue(25020,1);
	ros::Duration(3).sleep();
	my_variable.setAddressValue(30010,1);
	ros::Duration(3).sleep();
	my_variable.turnOn(25012);
	ros::Duration(3).sleep();
	my_variable.getAddressValue(0);
	ros::Duration(3).sleep();
	my_variable.getAddressValue(25010);
	my_variable.turnOff(0);
	ros::Duration(3).sleep();
	my_variable.setAddressValue(25020,0);
	ros::Duration(3).sleep();
	my_variable.turnOff(2);
	return 0;
}
