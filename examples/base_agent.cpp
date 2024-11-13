#include "agents.h"
#include <iostream>
#include "swarm.hpp"
#include <nlohmann/json.hpp>


std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
std::string base_url = "https://api.deepseek.com/chat/completions";

int main() {
    // Create an agent with default values: name, model, and instructions
    Agent agent("Base Agent", "deepseek-chat", "You are a helpful assistant.");

    Swarm swarm(api_key, base_url);

    std::vector<nlohmann::json> messages = {
        {
            {"role", "user"},
            {"content", "Hello, who are you?"}
        }
    };

    std::map<std::string, std::string> context;

    auto response = swarm.run(agent, messages, context);

    // Print the final response
    for (const auto& message : response.get_messages()) {
        if (message.find("role") != message.end() && 
            message.find("content") != message.end()) {
            
            if (message.at("role") == "assistant") {
                std::cout << "\n" << message.at("content") << std::endl;
            }
        }
    }

    return 0;
}