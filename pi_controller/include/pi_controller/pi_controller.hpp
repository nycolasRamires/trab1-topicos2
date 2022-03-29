#ifndef PI_CONTROLLER__PI_CONTROLLER_HPP_
#define PI_CONTROLLER__PI_CONTROLLER_HPP_

#include "pi_controller/visibility_control.h"
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/msg/Twist.hpp>
#include <geometry_msgs/msg/Accel.hpp>

namespace pi_controller //tem q ver se é velocity controller !!
{
  using CallbackReturn=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class PiController: public controller_interface::ControllerInterface
  {
  public: PI_CONTROLLER
    PI_CONTROLLER_PUBLIC
		PiController(void);

		PI_CONTROLLER_PUBLIC
		controller_interface::InterfaceConfiguration command_interface_configuration(void) const override;

		PI_CONTROLLER_PUBLIC
		controller_interface::InterfaceConfiguration state_interface_configuration(void) const override;

		PI_CONTROLLER_PUBLIC
		CallbackReturn on_init(void) override;

		PI_CONTROLLER_PUBLIC
		CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

		PI_CONTROLLER_PUBLIC
		CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

		PI_CONTROLLER_PUBLIC
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

		PI_CONTROLLER_PUBLIC
		controller_interface::return_type update(void) override;

    private:
    std::vector<std::string> joints_;
    //publicador e publicador de tempo real
    std::shared_ptr<rclcpp::Publisher<geometry_msg::msg::Accel>> acel_publisher;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msg::msg::Accel>> rt_acel_publisher;

    //Assinatura da ref e de u(em odom)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_s;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr u_s; // !!
    
    double time_step_;
		rclcpp::Time lastSamplingTime_;

  	int priority_;
    //Assinatura em tempo real da ref e de u(em odom)
		realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_ref_s;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_u_s; // !!

    //callback para quando for publicado no tópico
    void commandCB_ref(const geometry_msgs::msg::Twist::SharedPtr ref_CB);
    void commandCB_u(const geometry_msgs::msg::Twist::SharedPtr u_CB); // !!

  };

}  // namespace pi_controller

#endif  // PI_CONTROLLER__PI_CONTROLLER_HPP_
