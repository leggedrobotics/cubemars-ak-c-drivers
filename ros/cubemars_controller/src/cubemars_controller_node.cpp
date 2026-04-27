#include <rclcpp/rclcpp.hpp>
#include <cubemars_msgs/msg/mit_command.hpp>
#include <cubemars_msgs/msg/mit_feedback.hpp>
#include <cubemars_msgs/srv/enter_mit.hpp>
#include <cubemars_msgs/srv/exit_mit.hpp>
#include <cubemars_msgs/srv/set_zero.hpp>

#include "ak_mit_adapter.h"

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>

class CubemarsControllerNode : public rclcpp::Node
{
public:
    CubemarsControllerNode() : Node("cubemars_controller_node"), sock_(-1)
    {
        declare_parameter("can_interface", "can0");
        declare_parameter("motor_model", "AK40-10");

        auto iface = get_parameter("can_interface").as_string();
        auto model = get_parameter("motor_model").as_string();

        ak_set_model_by_name(model.c_str());
        RCLCPP_INFO(get_logger(), "Motor model: %s", model.c_str());

        sock_ = open_can_socket(iface.c_str());
        if (sock_ < 0) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open CAN socket on %s — ensure the interface is up "
                "(e.g. 'sudo ip link set %s type can bitrate 1000000 && sudo ip link set %s up')",
                iface.c_str(), iface.c_str(), iface.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "CAN socket open on %s", iface.c_str());
        }

        using MITCommand = cubemars_msgs::msg::MITCommand;
        mit_cmd_sub_ = create_subscription<MITCommand>(
            "~/mit_command", 10,
            [this](MITCommand::ConstSharedPtr msg) {
                MITFrame f = ak_command_frame(
                    msg->motor_id, msg->p_des, msg->v_des,
                    msg->kp, msg->kd, msg->t_ff);
                send_frame(f);
            });

        using EnterMIT = cubemars_msgs::srv::EnterMIT;
        enter_srv_ = create_service<EnterMIT>(
            "~/enter_mit",
            [this](const std::shared_ptr<EnterMIT::Request> req,
                   std::shared_ptr<EnterMIT::Response> res) {
                res->success = send_frame(ak_enter_frame(req->motor_id));
            });

        using ExitMIT = cubemars_msgs::srv::ExitMIT;
        exit_srv_ = create_service<ExitMIT>(
            "~/exit_mit",
            [this](const std::shared_ptr<ExitMIT::Request> req,
                   std::shared_ptr<ExitMIT::Response> res) {
                res->success = send_frame(ak_exit_frame(req->motor_id));
            });

        using SetZero = cubemars_msgs::srv::SetZero;
        zero_srv_ = create_service<SetZero>(
            "~/set_zero",
            [this](const std::shared_ptr<SetZero::Request> req,
                   std::shared_ptr<SetZero::Response> res) {
                res->success = send_frame(ak_set_zero_frame(req->motor_id));
            });

        feedback_pub_ = create_publisher<cubemars_msgs::msg::MITFeedback>(
            "~/mit_feedback", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { poll_feedback(); });
    }

    ~CubemarsControllerNode()
    {
        if (sock_ >= 0) close(sock_);
    }

private:
    int sock_;

    rclcpp::Subscription<cubemars_msgs::msg::MITCommand>::SharedPtr mit_cmd_sub_;
    rclcpp::Service<cubemars_msgs::srv::EnterMIT>::SharedPtr enter_srv_;
    rclcpp::Service<cubemars_msgs::srv::ExitMIT>::SharedPtr exit_srv_;
    rclcpp::Service<cubemars_msgs::srv::SetZero>::SharedPtr zero_srv_;
    rclcpp::Publisher<cubemars_msgs::msg::MITFeedback>::SharedPtr feedback_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int open_can_socket(const char *iface)
    {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0) return -1;

        struct ifreq ifr{};
        strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            close(s);
            return -1;
        }

        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            close(s);
            return -1;
        }

        return s;
    }

    bool send_frame(const MITFrame &f)
    {
        if (sock_ < 0) return false;
        struct can_frame frame{};
        frame.can_id  = f.id;
        frame.can_dlc = f.len;
        memcpy(frame.data, f.data, f.len);
        ssize_t n = write(sock_, &frame, sizeof(frame));
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            RCLCPP_ERROR(get_logger(), "CAN write failed: %s", strerror(errno));
            return false;
        }
        return true;
    }

    void poll_feedback()
    {
        if (sock_ < 0) return;
        struct can_frame frame{};
        ssize_t n = recv(sock_, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n != static_cast<ssize_t>(sizeof(frame))) return;
        if (frame.can_id & CAN_EFF_FLAG) return;
        if (frame.can_id & CAN_RTR_FLAG) return;

        float pos, vel, torque;
        ak_decode_feedback(frame.data, &pos, &vel, &torque);

        auto msg = cubemars_msgs::msg::MITFeedback();
        msg.motor_id = static_cast<uint8_t>(frame.can_id & CAN_SFF_MASK);
        msg.position  = pos;
        msg.velocity  = vel;
        msg.torque    = torque;
        feedback_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CubemarsControllerNode>());
    rclcpp::shutdown();
    return 0;
}
