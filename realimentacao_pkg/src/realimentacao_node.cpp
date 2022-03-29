#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <rclcpp/rclcpp.hpp>


class Realimentacao: public rclcpp::Node{
  public:
  Realimentacao(void);
  void receptor_ref(const geometry_msgs::msg::Pose::SharedPtr ref_r);
  void receptor_pose_atual(const geometry_msgs::msg::Pose::SharedPtr pose_atual_r);
  void Pub_N1_N2(void);
  private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_s;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_atual_s;

  geometry_msgs::msg::Pose ref;
  geometry_msgs::msg::Pose pose_atual; //ODOM

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr n1_n2_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist ns; //tem angular e linear
};

void Realimentacao::receptor_ref(const geometry_msgs::msg::Pose::SharedPtr ref_r)
{
  ref = *ref_r;
}

void Realimentacao::receptor_pose_atual(const geometry_msgs::msg::Pose::SharedPtr pose_atual_r)
{
  pose_atual = *pose_atual_r.pose.pose;
}

Realimentacao::Realimentacao(void): Node("realimentacao_node"){
  using std::placeholders::_1;
  //Serve apenas para fazer o link?
  ref_s = create_subscription<geometry_msgs::msg::Pose>("/ref",10,std::bind(&Realimentacao::receptor_ref,this,_1));
  pose_atual_s = create_subscription<geometry_msgs::msg::Pose>("/dynamics_linearizing_controller/odom",10,std::bind(&Realimentacao::receptor_pose_atual,this,_1)); // !!

  n1_n2_publisher = create_publisher<geometry_msgs::msg::Twist>("/vel_ref",10);

  using namespace std::chrono_literals;
  timer_=rclcpp::create_timer(this,this->get_clock(),100ms,std::bind(&Realimentacao::Pub_N1_N2,this));

}

void Realimentacao::Pub_N1_N2(void){
  //  calcular o e, alpha , phi
  //  calculcar o n1 e n2
  //  n1 = u1ref
  //  n2 = u2ref
  //publicar n1 e n2 em um tópico que o PID vai ASSINAR 
  //e é isso por aqui
  n1_n2_publisher->publish(ns);
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<Realimentacao>());

  rclcpp::shutdown();

  return 0;
}
