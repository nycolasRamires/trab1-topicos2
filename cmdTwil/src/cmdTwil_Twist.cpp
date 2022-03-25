#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class setVelTwil: public rclcpp::Node{
    public:
        setVelTwil(void);
        void commandCB(void) const;

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

setVelTwil::setVelTwil(void): Node("twilVel_commander"){
    
    velPublisher_ = create_publisher<geometry_msgs::msg::Twist>("/dynamics_linearizing_controller/command",10);

    using namespace std::chrono_literals;
    timer_=rclcpp::create_timer(this,this->get_clock(),1000ms,std::bind(&setVelTwil::commandCB,this));
}

void setVelTwil::commandCB(void) const{
    
    geometry_msgs::msg::Twist comando;
    
    comando.linear.x=0.0;
	comando.linear.y=0.0;
	comando.linear.z=0.0;
    
    comando.angular.x=0.0;
	comando.angular.y=0.0;
	comando.angular.z=10.0;

	velPublisher_->publish(comando);
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<setVelTwil>());
	
	rclcpp::shutdown();
	return 0;
}