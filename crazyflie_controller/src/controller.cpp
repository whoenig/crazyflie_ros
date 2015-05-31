#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>


#include "pid.hpp"

class Controller
{
public:

    Controller(const std::string& frame)
        : m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidX(40, 20, 2.0, -10, 10, -0.1, 0.1, "x")
        , m_pidY(-40, -20, -2.0, -10, 10, -0.1, 0.1, "y")
        , m_pidZ(5000.0, 6000.0, 3500.0, 10000, 60000, -1000, 1000, "z")
        , m_pidYaw(-200.0, -20.0, 0.0, -200.0, 200.0, 0, 0, "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
    {
        ros::NodeHandle n;
        m_pubNav = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = n.subscribe("goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = n.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = n.advertiseService("land", &Controller::land, this);
    }

    void run()
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/50.0), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::Pose::ConstPtr& msg)
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
        m_pidX.reset();
        m_pidZ.reset();
        m_pidZ.reset();
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
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
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
                m_goal.position.z = 0.05;
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

                //tf::Stamped< tf::Pose > targetWorld(m_goal, transform.stamp_, "world");
                geometry_msgs::PoseStamped targetWorld;
                //(m_goal, transform.stamp_, "world");
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = "world";
                targetWorld.pose = m_goal;

                //tf::Stamped< tf::Pose > targetDrone;
                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
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
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::Pose m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n("~");
  std::string frame;
  n.getParam("frame", frame);
  std::cout << frame << std::endl;

  Controller controller(frame);
  controller.run();

  // std::thread t(&Controlle::run, &controller);
  // t.detach();

  // ros::spin();

  return 0;
}
