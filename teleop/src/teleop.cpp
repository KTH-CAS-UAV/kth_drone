#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <termios.h>


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "teleop");

  ros::NodeHandle n;

  
  ros::Publisher pub_teleop = n.advertise<std_msgs::String>("/teleop/keyinput", 1000);

  ros::Rate loop_rate(10);

  std_msgs::String msg;

  std::stringstream ss;
  
  while (ros::ok())
  {


        int c = getch();   // call your non-blocking input function
        
        if (c == 'n'){
          msg.data="n";
          ROS_INFO("sending Next View point command");
        }
        else if (c == 'b'){
          msg.data="b";
          ROS_INFO("sending previous View point command");
        }
        else{
          msg.data="f";
          ROS_INFO("sending useless command");
        }


    pub_teleop.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}