#include <sys/mman.h>
#include <pi_controller/pi_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid_ros.hpp>


namespace pi_controller //velocity ???
{

    PiController::PiController(void){}

    CallbackReturn PiController::on_init(void)
	{
		try
		{
			auto_declare<std::vector<std::string>>("joints",joints_);
			auto_declare<double>("time_step",0.01);
			auto_declare<int>("priority",sched_get_priority_max(SCHED_FIFO));   
		}
		catch(const std::exception &e)
		{
			RCLCPP_ERROR_STREAM(node_->get_logger(),"Exception thrown in on_init() with message: " << e.what());
			return CallbackReturn::ERROR;
		}
		return CallbackReturn::SUCCESS;
	}

    CallbackReturn PiController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
	{
        control_toolbox::PidROS pid_a_ang;
        control_toolbox::PidRos pid_a_lin;

		try
		{
			joints_=node_->get_parameter("joints").as_string_array();
			if(joints_.empty())
				throw std::runtime_error("'joints' parameter was empty,");

			acel_publisher=node_->create_publisher<geometry_msg::msg::Accel>("~/acel",rclcpp::SystemDefaultsQoS());
			rt_acel_publisher=std::make_shared<realtime_tools::RealtimePublisher<geometry_msg::msg::Accel>>(acel_publisher);
            rt_acel_publisher->msg_.accel.accel.linear.y=0;
            rt_acel_publisher->msg_.accel.accel.linear.z=0;
            rt_acel_publisher->msg_.accel.accel.angular.x=0;
            rt_acel_publisher->msg_.accel.accel.angular.y=0;

			using std::placeholders::_1;
			ref_s=node_->create_subscription<geometry_msgs::msg::Twist>("/vel_ref",1000,
				std::bind(&PiController::commandCB_ref,this,_1));

            u_s=node_->create_subscription<geometry_msgs::msg::Twist>("/dynamics_linearizing_controller/odom",1000,
				std::bind(&PiController::commandCB_u,this,_1));


			time_step_=node_->get_parameter("time_step").as_double();
		}
		catch(const std::exception &e)
		{
			RCLCPP_ERROR_STREAM(node_->get_logger(),"Exception thrown in on_confiture(): " << e.what());
			return CallbackReturn::ERROR;
		}

		if(!node_->get_parameter("priority",priority_))
			RCLCPP_WARN(node_->get_logger(),"No 'priority' configured for controller. Using highest possible priority.");

		return CallbackReturn::SUCCESS;
	}

    controller_interface::InterfaceConfiguration PiController::command_interface_configuration(void) const
	{
		controller_interface::InterfaceConfiguration config;
		config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

		for(const auto &joint : joints_)
			config.names.push_back(joint + "/" + hardware_interface::HW_IF_ACCELERATION);

		return config;
	}

    controller_interface::InterfaceConfiguration PiController::state_interface_configuration(void) const
	{
		controller_interface::InterfaceConfiguration config;
		config.type=controller_interface::interface_configuration_type::INDIVIDUAL;

		for(const auto &joint : joints_)
			config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);

		return config;
	}

    CallbackReturn PiController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
	{
		pid_a_ang.init(); //INIT NOS PIDs
        pid_a_lin.init();

		lastSamplingTime_=node_->get_clock()->now();

		struct sched_param param;
		param.sched_priority=priority_;
		if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
			RCLCPP_WARN(node_->get_logger(),"Failed to set real-time scheduler.");
		if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
			RCLCPP_WARN(node_->get_logger(),"Failed to lock memory.");

		return CallbackReturn::SUCCESS;
	}    

    CallbackReturn PiController::on_deactivate(const rclcpp_lifecycle::State &/*previous_state*/)
	{
		for(unsigned int i=0;i < joints_.size();i++)
			command_interfaces_[i].set_value(0);

		return CallbackReturn::SUCCESS;
	}

controller_interface::return_type PiController::update(void)
	{
        //pega tempo agora e calculo o dt
		auto time=node_->get_clock()->now();
		auto dt=time-lastSamplingTime_;
        
        //verifica se dt foi menor q time_step_
		if(dt.seconds() < time_step_) return controller_interface::return_type::OK;
		lastSamplingTime_=time;
        
        auto vel_ref = rt_ref_s.readFromRT();
        auto vel_u = rt_u_s.readFromRT();

        auto e_ang = vel_ref.angular.z - vel_u.angular.z;
        auto e_lin = vel_ref.linear.x - vel_u.linear.x;

        auto a_ang = pid_a_ang.updatePid(e, dt);
        auto a_lin = pid_a_lin.updatePid(e, dt);

        if(rt_acel_publisher && rt_acel_publisher->trylock()){
            rt_acel_publisher->msg_.angular.z = a_ang;
            rt_acel_publisher->msg_.linear.x = a_lin;
            
            rt_acel_publisher->unlockAndPublish();
        }

		return controller_interface::return_type::OK;
	}

    void PiController::commandCB_ref(const geometry_msgs::msg::Twist::SharedPtr ref_CB)
	{
		rt_ref_s.writeFromNonRT(ref_CB);
	}

    void PiController::commandCB_u(const geometry_msgs::msg::Twist::SharedPtr u_CB)
	{
		rt_u_s.writeFromNonRT(u_CB);
	}

}  // namespace pi_controller
