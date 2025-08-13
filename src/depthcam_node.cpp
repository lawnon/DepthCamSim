/**
** Datei: depthcam_node.cpp
** Beschreibung: Navigations-Programm zu Erfassung der Tiefenwerte
**               und steuerung des Roboters.
** Autor: Chukwunonso Bob-Anyeji
** Date: 18.07.2025
 */

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

int MsgCount = 1;
long AngleCounter = 0;
long RotCount = 30;
float SafeDist = 150;
int RobState = 0;
bool CheckRight = false;
bool CheckLeft = false;

std::string topic_pub = "/cmd_vel";
gz::transport::Node node;
gz::transport::Node::Publisher Publisher = node.Advertise<gz::msgs::Twist>(topic_pub);

/** Subscriber-Instance zur Erfassung und Auswertung der Daten */
class DepthCamSubscriber : public rclcpp::Node
{
  public:
    DepthCamSubscriber()
      : Node("depthCam_Abonement")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/depthcam/points", 10, std::bind(&DepthCamSubscriber::PointCloudHandler, this, _1));
    }

  private:

    void PointCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) const
    {
      int logHz = 10;
      MsgCount++;

      if((MsgCount % logHz) == 0)
      {
        std::cout << std::endl
                  << "MSG[" << MsgCount << "]"
                  << "=============================================================="
                  << std::endl
                  << "* PunktDaten mit 4 Werten/Punkt d.h X, Y, z, rgb *"
                  << std::endl
                  << "Width: "<< msg->width
                  << " | Height: " << msg->height
                  << " | Field.Count: " << msg->fields.size()
                  << " | Point.Count: " << msg->data.size()
                  << std::endl
                  << "RowStep:" << msg->row_step
                  << " | PointStep: " << msg->point_step
                  << " | BigEndian: " << msg->is_bigendian
                  << " | Dense: " << msg->is_dense << std::endl;

        for(size_t i = 0;i < msg->fields.size(); i++)
        {
          std::cout << "Field.Name: " << msg->fields[i].name
                    << " | Field.Offset: " << msg->fields[i].offset
                    << " | Field.Type: " << std::to_string(msg->fields[i].datatype)
                    << " | Field.Count: " << msg->fields[i].count << std::endl;
        }
      }

      if (msg->is_dense != 1)
        return;

      long index = 0;
      double max = 0;
      double min = 0;
      for (sensor_msgs::PointCloud2ConstIterator<double> it(*msg, "x");
           it != it.end();++it)
      {
        if(max < *it){
          max = *it;
        }
        if(min > *it){
          min = *it;
        }
        index++;
      }
      double diff = std::abs(min);

      /** Navigations Auswertung */
      gz::msgs::Twist data;
      if(diff > 0)
      {
        switch (RobState)
        {
          case 0:
              AngleCounter++;
              data.mutable_linear()->set_x(0.0);
              data.mutable_angular()->set_z(0.0);
              if(AngleCounter > 5)
              {
                if(diff > SafeDist)
                {
                  RobState = 1;
                }
                else
                {
                  if(CheckRight)
                    RobState = 2;
                  else
                    RobState =3;
                }
              }
            break;
          case 1:
            if (diff > SafeDist)
            {
              data.mutable_linear()->set_x(0.5);
              data.mutable_angular()->set_z(0.0);

              AngleCounter = 0;
              CheckRight = true;
            }
            else
            {
              RobState = 2;
            }

            break;
          case 2:
            if(AngleCounter < RotCount)
            {
              AngleCounter++;
              data.mutable_linear()->set_x(0.0);
              data.mutable_angular()->set_z(0.5);
              std::cout << AngleCounter << std::endl;

            }
            if(AngleCounter >= RotCount)
            {
              AngleCounter = 0;
              RobState = 0;
              CheckRight = false;
            }
            break;
          case 3:
            if (AngleCounter < (RotCount*2))
            {
              AngleCounter++;
              data.mutable_linear()->set_x(0.0);
              data.mutable_angular()->set_z(-0.5);
            }
            if(AngleCounter >= (RotCount*2))
            {
              AngleCounter = 0;
              RobState = 0;
              CheckRight = true;
            }
            break;
        }

        if (0 == (MsgCount % logHz))
        {
          std::cout << "RobState: " << RobState
                    << " | Diff: " << diff
                    << " | MinVal: "<< min
                    << " | MaxVal: " << max << std::endl;
        }

        Publisher.Publish(data);
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr subscription_;
};

/** Programm Eingangs Punkt */
int main(int argc, char * argv[])
{
  std::string topic_sub = "/depthcam/points";

  std::cout << "/* Navigationsprogramm gestartet */"
            << std::endl
            << "Topic: [" << topic_sub
            << "] wird Abonniert..." << std::endl;
  rclcpp::init(argc, argv);

  std::cout << "Topic [" << topic_sub
            << "] Abonnement Erfolgreich."
            << std::endl
            <<"Warten auf Sensordaten eingang..."<< std::endl;
  rclcpp::spin(std::make_shared<DepthCamSubscriber>());

  /*Programm Beenden*/
  rclcpp::shutdown();
  gz::transport::waitForShutdown();

  return 0;
}
