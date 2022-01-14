// ROS
//https://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
#ifdef ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

nav_msgs::String str_odo;
ros::Publisher Odometrie("Odometrie", &str_odo);

std_msgs::String str_imu;
ros::Publisher imu("imu", &str_imu);

std_msgs::String str_gps;
ros::Publisher gps("gps", &str_gps);

void messageCb( const std_msgs::str_moteurs msg)
{
 
}

std_msgs::String str_moteurs;
ros::Subscriber<std_msgs::msg> moteurs("motors", &str_moteurs, &messageCb);
#endif


#ifdef ROS
void PublishOdo()
{
  String Message = String(CompteurOdoG) + " " + String(CompteurOdoD);
  str_odo.data = Message;
  Odometrie.publish( &str_odo );
  nh.spinOnce();
  // delay(1000);
}

void PublishIMU()
{
  String Message = "X : " + String(mpu.getAngleX()) + " Y : " + String(mpu.getAngleY()) + " Z : " + String(mpu.getAngleZ());
  str_imu.data = Message;
  Iimu.publish( &str_imu );
  nh.spinOnce();
  // delay(1000);
}
#endif

#ifdef ROS
ROS_CALLBACK(messageCb, std_msgs::char, msg)
{
  switch (msg.data) {
    case 'A':
      Avance();
      break;
    case 'R':
      Avance();
      break;
    case 'G':
      Gauche();
      break;
    case 'D':
      Droite();
      break;
    case 'P':
      AvanceProg();
      break;
    case 'S':
      Stop();
      break;
    default:
      // Serial1.println("choix : A R S G D P");
      break;
  } // fin switch

}
#endif
