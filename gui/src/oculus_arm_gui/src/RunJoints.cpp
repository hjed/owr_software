#include "oculus_arm_gui/JointManager.hpp"

int main(int argc, char ** argv) {
	ros::init(argc, argv, "arm_gui");
	ros::NodeHandle nh;
	JointManager jm(nh);
	jm.logicLoop();

}
