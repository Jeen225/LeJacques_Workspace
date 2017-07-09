#include <iostream>
#include "pi.h"

using namespace std;

int main()
{
	PI yaw_controller(1900,1100,1);
	while(1)
	{
		yaw_controller.setPID(true,320,640,4);
		cout <<yaw_controller.getError() <<" "<<yaw_controller.kp <<" " <<yaw_controller.getCommand() <<" " <<yaw_controller.yaw_command() <<endl;
	}
}
