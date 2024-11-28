#include "swarm_agents/swarm_ros.hpp"


// Function to transfer to Chinese agent
AgentFunction transfer_to_Chinese_agent = [](const std::map<std::string, std::string>& params) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    Agent chinese_agent;
    chinese_agent.set_name("Chinese Agent");
    chinese_agent.set_model("deepseek-chat");
    
    chinese_agent.set_instructions(u8"您好，我是一位中文助手，很高兴为您服务。");
    return chinese_agent;
};

// Additional weather-related functions
std::variant<std::string, Agent, std::map<std::string, std::string>> 
get_forecast(const std::map<std::string, std::string>& params) {
    if (params.find("location") == params.end()) {
        return "Error: Location not provided";
    }
    if (params.find("days") == params.end()) {
        return "Error: Number of days not provided";
    }
    
    return "The " + params.at("days") + "-day forecast for " + params.at("location") + 
           " shows mild temperatures with partly cloudy skies.";
}

std::variant<std::string, Agent, std::map<std::string, std::string>> 
get_weather_alerts(const std::map<std::string, std::string>& params) {
    if (params.find("location") == params.end()) {
        return "Error: Location not provided";
    }
    
    return "No current weather alerts for " + params.at("location");
}

// Callback function to write response data
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

SwarmROS::SwarmROS(std::shared_ptr<rclcpp::Node> &node)
    : node_(node)
{
    cout << "start agent..." << endl;
    node_->declare_parameter<int>("max_nb_agents", 3);
    node_->declare_parameter<int>("agent_id", 0);
    node_->declare_parameter<std::string>("api_key", "sk-face3f6903e24f778bbe44b21d82dc6e");
    node_->declare_parameter<std::string>("base_url", "https://api.deepseek.com/chat/completions");
    node_->get_parameter("agent_id", agent_id_);
    node_->get_parameter("max_nb_agents", max_nb_agents_);
    node_->get_parameter("api_key", api_key_);
    node_->get_parameter("base_url", base_url_);
    cout << " agent_id = " << agent_id_ << " max_nb_agents = " << max_nb_agents_ << endl;

    // Create English agent
    english_agent.set_name("English Agent");
    english_agent.set_model("deepseek-chat");

    english_agent.set_instructions(u8"You are a language translation assistant that only speaks English.\n"
                                   u8"If a user speaks Chinese, immediately transfer them to the Chinese Agent. Note You should not speak Chinese.\n");

    // Add the transfer function with metadata
    auto transfer_metadata = create_function_signature<decltype(transfer_to_Chinese_agent)>(
        "transfer_to_Chinese_Agent",
        "Transfer Chinese speaking users to Chinese agent",
        transfer_to_Chinese_agent,
        {},
        {} // no required parameters
    );

    // Add function to English agent
    english_agent.functions = {transfer_metadata};

    // Initialize the Swarm with API credentials


   // std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
    //std::string base_url = "https://api.deepseek.com/chat/completions";



    swarm.set_api_key(api_key_);
    swarm.set_base_url(base_url_);

    // Interactive loop for weather queries

    std::cout << "WeatherBot initialized. Type 'quit' to exit.\n";
    conversation_send_publisher_ = node_->create_publisher<std_msgs::msg::String>("/order", 10);

    // 创建发布者
    for (int i = 0; i < max_nb_agents_; ++i)
    {
        std::string topic_name = "/r" + std::to_string(i) + "/conversation/single_message_shared";
        conversation_update_single_publishers_[i] = node_->create_publisher<std_msgs::msg::String>(topic_name, 10);
    }
    conversation_received_single_subscription = node_->create_subscription<std_msgs::msg::String>(
        "/r" + std::to_string(agent_id_) + "/conversation/single_message_shared", 10,
        std::bind(&SwarmROS::conversation_received_single_callback, this, std::placeholders::_1));

    conversation_received_subscription = node_->create_subscription<std_msgs::msg::String>(
        "/order", 10,
        std::bind(&SwarmROS::conversation_callback, this, std::placeholders::_1));

    /* conversation_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(5000),
            //std::chrono::seconds(1),
            std::bind(&SwarmROS::send_conversation_callback, this)); */

    comm_agent.set_name("Comm Agent");
    comm_agent.set_model("deepseek-chat");

    comm_agent.set_instructions(u8"我是一位通信助手， 负责在不同agent之间通过ros话题传递历史消息。");

    send_conversation_all(0);

    // rclcpp::WallRate rate(100);
}

    void SwarmROS::conversation_received_single_callback(const std_msgs::msg::String::SharedPtr msg){
        nlohmann::json json_data = nlohmann::json::parse(msg->data);
        std::vector<nlohmann::json> messages;

        // 检查 json_data 是否是一个数组
        if (json_data.is_array())
        {
            // 将 nlohmann::json 对象转换为 std::vector<nlohmann::json>
            messages = json_data.get<std::vector<nlohmann::json>>();

            // 现在你可以使用 messages 这个 vector
            for (const auto &message : messages)
            {
                // 处理每个 JSON 对象
                std::cout << message.dump() << std::endl;
            }
        }
        else
        {
            std::cerr << "Error: JSON data is not an array." << std::endl;
        }

        std::cout << "\nEnter your weather-related question: ";

        /* if (user_input == "quit") {
            break;
        } */

        // Add user message to history
        // Create initial message in Chinese

        std::map<std::string, std::string> context;

        try
        {
            // Run the conversation
            Response response = swarm.run(
                english_agent,
                messages,
                context,
                "",    // no model override
                false, // no streaming
                true   // debug mode
            );
            

            // Print the final response
            for (const auto &message : response.get_messages())
            {
                if (message.find("role") != message.end() &&
                    message.find("content") != message.end())
                {

                    if (message.at("role") == "assistant")
                    {
                        std::cout << "\n"
                                  << message.at("content") << std::endl;
                    }
                }
            }

            

            // Check if agent was changed
            if (response.get_agent() && response.get_agent()->get_name() == "Chinese Agent")
            {
                std::cout << "\nSuccessfully transferred to Chinese Agent!" << std::endl;
            }
            else
            {
                std::cout << "\nAgent not changed." << std::endl;
                std::cout << "\nThe current agent is " << response.get_agent()->get_name() << std::endl;
            }
            
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }


    }

    void SwarmROS::conversation_callback(const std_msgs::msg::String::SharedPtr msg)
    {

        nlohmann::json json_data = nlohmann::json::parse(msg->data);
        std::vector<nlohmann::json> messages;

        // 检查 json_data 是否是一个数组
        if (json_data.is_array())
        {
            // 将 nlohmann::json 对象转换为 std::vector<nlohmann::json>
            messages = json_data.get<std::vector<nlohmann::json>>();

            // 现在你可以使用 messages 这个 vector
            for (const auto &message : messages)
            {
                // 处理每个 JSON 对象
                std::cout << message.dump() << std::endl;
            }
        }
        else
        {
            std::cerr << "Error: JSON data is not an array." << std::endl;
        }

        std::cout << "\nEnter your weather-related question: ";

        /* if (user_input == "quit") {
            break;
        } */

        // Add user message to history
        // Create initial message in Chinese

        std::map<std::string, std::string> context;

        try
        {
            // Run the conversation
            Response response = swarm.run(
                english_agent,
                messages,
                context,
                "",    // no model override
                false, // no streaming
                true   // debug mode
            );
            

            // Print the final response
            for (const auto &message : response.get_messages())
            {
                if (message.find("role") != message.end() &&
                    message.find("content") != message.end())
                {

                    if (message.at("role") == "assistant")
                    {
                        std::cout << "\n"
                                  << message.at("content") << std::endl;
                    }
                }
            }

            

            // Check if agent was changed
            if (response.get_agent() && response.get_agent()->get_name() == "Chinese Agent")
            {
                std::cout << "\nSuccessfully transferred to Chinese Agent!" << std::endl;
            }
            else
            {
                std::cout << "\nAgent not changed." << std::endl;
                std::cout << "\nThe current agent is " << response.get_agent()->get_name() << std::endl;
            }
            
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }

        
    }

    void SwarmROS::send_conversation_all(int i)
    {

        std::vector<nlohmann::json> messages = {
            {{"role", "user"},
             {"content", u8"今天星期几?"}}};
        // 将 std::vector 转换为 nlohmann::json 对象
        nlohmann::json json_messages = messages;

        // 将 JSON 对象转换为字符串
        std::string json_string = json_messages.dump(); // 转换为 JSON 字符串

        auto message = std_msgs::msg::String();
        message.data = json_string; // 将 JSON 字符串赋值给消息
        RCLCPP_INFO(node_->get_logger(), "Publishing: '%s'", message.data.c_str());
        //conversation_send_publisher_->publish(message);
         conversation_update_single_publishers_[i]->publish(message);
    }
