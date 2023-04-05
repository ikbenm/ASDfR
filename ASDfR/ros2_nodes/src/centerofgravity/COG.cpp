// Copyright 2016 Open Source Robotics Foundation, Inc.

//

// Licensed under the Apache License, Version 2.0 (the "License");

// you may not use this file except in compliance with the License.

// You may obtain a copy of the License at

//

//     http://www.apache.org/licenses/LICENSE-2.0

//

// Unless required by applicable law or agreed to in writing, software

// distributed under the License is distributed on an "AS IS" BASIS,

// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

// See the License for the specific language governing permissions and

// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)
float center[2];

using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{

public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    this->declare_parameter("treshhold", 240); //this makes the treshhold parametere 240 so only my phones light would work in a very light up room
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "webcam_input", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)); //here it subscribes to webcam_input 

    publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10); //here it makes a publisher called setpoint with the kind Point2
  }



private:

  void topic_callback(const sensor_msgs::msg::Image & image) const
  {

int treshhold=
this->get_parameter("treshhold").get_parameter_value().get<int>(); //gets the parameter treshhold 


 auto message = asdfr_interfaces::msg::Point2(); //mesage is of kind point2 aka x and y float

int avg_bright;
int n=image.data.size();
int grayscale[image.width][image.height]; //grayscale array 
int bright = 0;
int sumx = 0;
int sumy = 0;

  for(int i=0; i < n/3; ++i) //this for loop is used to walk through the data of the image and make a grayscale array
  {
    grayscale[i%image.width][i/image.width]= (image.data[i*3] + image.data[i*3+1] + image.data[i*3+2])/3;
  }

 for(int i=0; i < image.width; ++i) 
  {
  for(int j=0; j < image.height; ++j)
  {
    if(grayscale[i][j]>treshhold) //this checks if the grayscale value of that pixel is higher than the treshhold and if it is the xvalue is added to a memmory value of x and the same is done by y (this way the COG can be calulated)
    {
      bright += 1;
      sumx += i;
      sumy += j;
    }
 }
  }

  if (bright == 0) //this is used because you cant devide by zero 
  {
    message.x = image.width/2; //then we just set it to the center 
    message.y = image.height/2;
  }
  else{ //if got a bright spot
    message.x = (sumx/bright); //here the COG for the x value is calculated
    message.y = (sumy/bright); //and here for the y value
  }

  message.x = message.x / image.width * 2 -1; //here the scale is used like described in the asignment from -1 to 1 to control the camera
  message.y = 1 - 2*message.y / image.height;

    std::string msg = std::to_string(message.x) + " " + std::to_string(message.y); //makes the message
    RCLCPP_INFO(this->get_logger(),msg.c_str()); //shows the message
    publisher_->publish(message); //publishes the data named message aka x and y
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
