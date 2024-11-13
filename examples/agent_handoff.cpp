#include "agents.h"
#include "swarm.hpp"
#include <iostream>

// Function to transfer to Chinese agent
AgentFunction transfer_to_Chinese_agent = [](const std::map<std::string, std::string>& params) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    Agent chinese_agent;
    chinese_agent.set_name("Chinese Agent");
    chinese_agent.set_model("deepseek-chat");
    
    chinese_agent.set_instructions(u8"您好，我是一位中文助手，很高兴为您服务。");
    return chinese_agent;
};

int main() {
    // Create English agent
    Agent english_agent("English Agent", "deepseek-chat", 
        u8"You are a language translation assistant that only speaks English.\n"
        u8"If a user speaks Chinese, immediately transfer them to the Chinese Agent. Note You should not speak Chinese.\n"
    );
    
    // Add the transfer function with metadata
    auto transfer_metadata = create_function_signature<decltype(transfer_to_Chinese_agent)>(
        "transfer_to_Chinese_Agent",
        "Transfer Chinese speaking users to Chinese agent",
        transfer_to_Chinese_agent,
        {},
        {}   // no required parameters
    );
    
    // Add function to English agent
    english_agent.functions = {transfer_metadata};
    
    // Initialize the Swarm with API credentials
    // std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
    // std::string base_url = "https://api.deepseek.com/beta/completions";

    std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
    std::string base_url = "https://api.deepseek.com/chat/completions";
    
    // std::string api_key = "sk-hxjweswehodmsljfxlwbigqfhkfcxwyttwxpkxahajcvjmgr";
    // std::string base_url="https://api.siliconflow.cn/v1";

    Swarm swarm(api_key, base_url);
    
    // Create initial message in Chinese
    std::vector<nlohmann::json> messages = {
        {
            {"role", "user"},
            {"content", u8"今天星期几?"}
        }
    };
    
    std::map<std::string, std::string> context;
    
    try {
        // Run the conversation
        Response response = swarm.run(
            english_agent,
            messages,
            context,
            "",     // no model override
            false,  // no streaming
            true    // debug mode
        );
        
        // Print the final response
        for (const auto& message : response.get_messages()) {
            if (message.find("role") != message.end() && 
                message.find("content") != message.end()) {
                
                if (message.at("role") == "assistant") {
                    std::cout << "\n" << message.at("content") << std::endl;
                }
            }
        }
        
        // Check if agent was changed
        if (response.get_agent() && response.get_agent()->get_name() == "Chinese Agent") {
            std::cout << "\nSuccessfully transferred to Chinese Agent!" << std::endl;
        }
        else {
            std::cout << "\nAgent not changed." << std::endl;
            std::cout << "\nThe current agent is " << response.get_agent()->get_name() << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    return 0;
} 