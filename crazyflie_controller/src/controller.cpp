#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:

    Controller(
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw",
            true)
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_kp(get(n, "PIDs/Body/kp"))
        , m_kd(get(n, "PIDs/Body/kd"))
        , m_ki(get(n, "PIDs/Body/ki"))
        , m_oldPosition(0,0,0)
        , m_current_r_error_integration(0,0,0)
        , m_massThrust(get(n, "MassThrust"))
        , m_maxAngle(get(n, "MaxAngle"))
    {
        ros::NodeHandle nh;
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_current_r_error_integration = tf::Vector3(0,0,0);
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform("/world", m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform("/world", m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform("/world", m_frame, ros::Time(0), transform);

                tf::Vector3 position = transform.getOrigin();
                tf::Vector3 current_velocity = (position - m_oldPosition) / dt;
                m_oldPosition = position;

                tf::Vector3 current_z_axis = transform.getRotation().getAxis();
                current_z_axis /= transform.getRotation().length();

                tf::Vector3 target_position(
                    m_goal.pose.position.x,
                    m_goal.pose.position.y,
                    m_goal.pose.position.z);

                tf::Quaternion target_quaternion(
                    m_goal.pose.orientation.x,
                    m_goal.pose.orientation.y,
                    m_goal.pose.orientation.z,
                    m_goal.pose.orientation.w);

                tfScalar target_euler_roll, target_euler_pitch, target_euler_yaw;
                tf::Matrix3x3(target_quaternion).getRPY(
                    target_euler_roll,
                    target_euler_pitch,
                    target_euler_yaw);

                tfScalar current_euler_roll, current_euler_pitch, current_euler_yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY(
                    current_euler_roll,
                    current_euler_pitch,
                    current_euler_yaw);

                tf::Vector3 current_r_error = target_position - position;
                tf::Vector3 current_r_error_unit = current_r_error.normalized();

                m_current_r_error_integration += current_r_error * dt;
                if (m_current_r_error_integration.length() >= 6) {
                    m_current_r_error_integration = 6.0 * m_current_r_error_integration.normalized();
                }

                double r_error_norm = current_r_error.length();
                tf::Vector3 target_velocity = 1.3 * (r_error_norm/5.0)*current_r_error_unit;
                if (r_error_norm >= 5.0) {
                    target_velocity = 1.3 * current_r_error_unit; // The velocity in the target position direction is 1 m/s
                }

                // compute z-axis-desired
                tf::Vector3 z_axis_desired = m_massThrust * tf::Vector3(0,0,1) + m_kp*current_r_error + m_kd*(target_velocity - current_velocity) + m_ki*m_current_r_error_integration;
                double angle = acos(z_axis_desired.normalized().dot(tf::Vector3(0,0,1)));
                double kp = m_kp;
                double kd = m_kd;
                double ki = m_ki;

                while (angle >= m_maxAngle) {
                    kp *= 0.9;
                    kd *= 0.9;
                    ki *= 0.9;
                    z_axis_desired = m_massThrust * tf::Vector3(0,0,1) + kp*current_r_error + kd*(target_velocity - current_velocity) + ki*m_current_r_error_integration;
                    angle = acos(z_axis_desired.normalized().dot(tf::Vector3(0,0,1)));
                }
                tf::Vector3 z_axis_desired_unit = z_axis_desired.normalized();

                // control
                double thrust = z_axis_desired.dot(current_z_axis);
                if (thrust < 0) {
                    thrust = 0;
                }
                if (thrust > 65536) {
                    thrust = 65536;
                }

                tf::Vector3 x_axis_desired = z_axis_desired_unit.cross(tf::Vector3(sin(target_euler_yaw), cos(target_euler_yaw), 0));
                x_axis_desired.normalize();
                tf::Vector3 y_axis_desired = z_axis_desired_unit.cross(x_axis_desired);

                double pitch_angle = asin(-1.0*x_axis_desired.getZ()) * 180.0 / M_PI;
                double yaw_angle = atan2(x_axis_desired.getY(), x_axis_desired.getX());
                double roll_angle = atan2(y_axis_desired.getZ(), z_axis_desired_unit.getZ()) * 180.0 / M_PI;

                geometry_msgs::Twist msg;
                msg.linear.x = pitch_angle;
                msg.linear.y = roll_angle;
                msg.linear.z = thrust;
                msg.angular.z = m_pidYaw.update(current_euler_yaw, yaw_angle);
                m_pubNav.publish(msg);

            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;

    double m_kp;
    double m_kd;
    double m_ki;
    tf::Vector3 m_oldPosition;
    tf::Vector3 m_current_r_error_integration;
    double m_massThrust;
    double m_maxAngle;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n("~");
  std::string frame;
  n.getParam("frame", frame);
  std::cout << frame << std::endl;

  Controller controller(frame, n);

  double frequency;
  n.param("frequency", frequency, 50.0);
  controller.run(frequency);

  // std::thread t(&Controlle::run, &controller);
  // t.detach();

  // ros::spin();

  return 0;
}
