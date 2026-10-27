#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace rclcpp;

Node::SharedPtr node;

Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
TimerBase::SharedPtr timet;

int mTime = 0;

void callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    mTime = 0;
    geometry_msgs::msg::Twist out = *msg;
    if (out.linear.x >= 0.2){
        out.linear.x = 0.2;
    } else if (out.linear.x <= -0.2){
        out.linear.x = -0.2;
    }
    if (out.angular.z >= 0.65){
        out.angular.z = 0.65;
    } else if (out.angular.z <= -0.65){
        out.angular.z = -0.65;
    }
    pub_->publish(out);
}

void timer(){
    mTime++;
    if (mTime >= 5){
        geometry_msgs::msg::Twist out;
        out.linear.x = 0.0;
        out.angular.z = 0.0;
        pub_->publish(out);
    }
}

int main(int argc,char **argv) {

    init(argc, argv);
    node = Node::make_shared("safedrive");
    sub_ = node->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10,&callback);
    pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    timet = node->create_timer(std::chrono::milliseconds(100),&timer);

    rclcpp::spin(node);  
    rclcpp::shutdown();
    return 0;
  
}