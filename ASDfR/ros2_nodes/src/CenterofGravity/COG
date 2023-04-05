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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
   this->declare_parameter("treshhold", 100);
  
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image & image) const
  {

int treshhold=
this->get_parameter("treshhold").get_parameter_value().get<int>();
//int treshhold=50;

int avg_bright;
int sum=0;
int n=image.data.size();
int grayscale[image.width][image.height];
int bright = 1;
int sumx = 0;
int sumy = 0;
int center[2];


std::string msg="";

  for(int i=0; i < n/3; ++i) 
  {
  grayscale[i%image.width][i/image.width]= (image.data[i*3] + image.data[i*3+1] + image.data[i*3+2])/3;

  }

 for(int i=0; i < image.width; ++i) 
  {
    for(int j=0; j<image.height; ++j)
 {
if(grayscale[i][j]>treshhold)
{
  bright += 1;
  sumx += i;
  sumy += j;
}

 }
    
  }

  if (bright == 0)
  {
    center[0] = image.width/2;
center[1] = image.height/2;
msg+=("no bright spots");
  }
  else{ //if got a bright spot
center[0] = sumx/bright;
center[1] = sumy/bright;
  }


msg +=(std::to_string(center[0]));
msg +=(" ");
msg +=(std::to_string(center[1]));

  //std::string msg = std::to_string(image.data.size());
  
    RCLCPP_INFO(this->get_logger(), msg.c_str());
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
