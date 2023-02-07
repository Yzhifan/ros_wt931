#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "uart_931.h"

ros::Publisher imu_pub;
uint32_t seq = 0;
sensor_msgs::Imu imu_msg;

void imu_data_pub(double *a, double *w, double *q)
{
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.seq = seq;
  imu_msg.linear_acceleration.x = a[0];
  imu_msg.linear_acceleration.y = a[1];
  imu_msg.linear_acceleration.z = a[2];
  imu_msg.angular_velocity.x = w[0];
  imu_msg.angular_velocity.y = w[1];
  imu_msg.angular_velocity.z = w[2];
  imu_msg.orientation.w = q[0];
  imu_msg.orientation.x = q[1];
  imu_msg.orientation.y = q[2];
  imu_msg.orientation.z = q[3];
  imu_pub.publish(imu_msg);

  seq++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;
  imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1);
  imu_msg.header.frame_id = "base_link";

  const char *port_name = "/dev/ttyUSB0";
  double baud = 921600;
  int fd;
  fd = uart_open(fd, port_name);
  if (fd == -1)
  {
    printf("Failed to uart open, port %s\n", port_name);
    return -1;
  }
  if (uart_set(fd, baud, 8, 'N', 1) == -1)
  {
    printf("Failed to uart set\n");
    return -1;
  }

  int ret = 0;
  char r_buf[1024];
  bzero(r_buf, 1024);

  int num_pack = 0;
  double a[3], w[3], q[4];

  while (ros::ok())
  {
    ret = recv_data(fd, r_buf, 44);
    if (ret == -1)
    {
      printf("Failed to uart read\n");
      uart_close(fd);
      return -2;
    }
    for (int i = 0; i < ret; i++)
    {
      if (parse_data(r_buf[i]) == 9)
      {
        get_data(a, w, q);
        imu_data_pub(a, w, q);
      }
    }
    usleep(100);
  }

  ret = uart_close(fd);
  if (ret == -1)
  {
    printf("\nFailed to uart close\n");
    return -1;
  }
  else
  {
    printf("\nClose port\n");
  }

  return 0;
}
