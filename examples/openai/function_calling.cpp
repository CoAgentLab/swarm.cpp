#include "agents.h"
#include "swarm.hpp"
#include <iostream>
#include <nlohmann/json.hpp>


std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
std::string base_url = "https://api.deepseek.com/chat/completions";

AgentFunction add_two_numbers = [](const std::map<std::string, std::string>& params) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    std::cout << "Adding " << params.at("a") << " and " << params.at("b") << std::endl;
    
    int a = std::stoi(params.at("a"));
    int b = std::stoi(params.at("b"));
    return std::to_string(a + b);
};  


int main() {

    Agent agent("Function Calling Agent", "deepseek-chat", "You are a function calling agent.");
    agent.functions = {create_function_signature<decltype(add_two_numbers)>(
        "add_two_numbers",
        "Add two numbers",
        add_two_numbers,
        {
            {"a", "The first number to add"},
            {"b", "The second number to add"}
        },
        {"a", "b"}  // required parameters
    )}; 

    Swarm swarm(api_key, base_url);

    std::vector<nlohmann::json> messages = {
        {
            {"role", "user"},
            {"content", "What is 10 plus 20?"}
        }
    };  

    std::map<std::string, std::string> context;

    auto response = swarm.run(agent, messages, context, "", false, true);

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