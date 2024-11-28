#include <common_lib.hpp>
#include "agent/agents.h"
#include "utils/utils.h"
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <variant>
#include <curl/curl.h>
#include "swarm_agents/swarm.hpp"

class SwarmROS
{
public:

/*
    * @Description: 
    * @Author: Yao Jian
    * @Date: 2023-11-10
    * @LastEditTime: 2023-11-10
    */
   /**
    * @func 
    * @desc 
    * @param {}  
    * @return {} 
    */
    SwarmROS(std::shared_ptr<rclcpp::Node> &node);
    ~SwarmROS(){};


    int agent_id_;
    int max_nb_agents_;
    Agent english_agent;
    Agent comm_agent;
    
    Swarm swarm;
    
    



    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::TimerBase::SharedPtr conversation_timer_;
        

        std::string api_key_;
        std::string base_url_;
        std::string user_input;
        std::vector<nlohmann::json> messages;
        std::map<std::string, std::string> context;
        
        

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr conversation_received_subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr conversation_received_single_subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr conversation_received_all_subscription;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    conversation_send_publisher_;
        std::map<int, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> conversation_update_single_publishers_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    conversation_update_all_publisher_;
        void conversation_callback(const std_msgs::msg::String::SharedPtr msg);
        void send_conversation_all(int i);
        void conversation_received_single_callback(const std_msgs::msg::String::SharedPtr msg);
        
        
    
};