#include "agents.h"
#include <iostream>
#include "swarm.hpp"
#include <nlohmann/json.hpp>


std::string api_key = "your-api-key";
std::string base_url = "https://api.deepseek.com/chat/completions";

int main() {
    // Create an agent with default values: name, model, and instructions
    Agent agent("Base Agent", "deepseek-chat", "You are a helpful assistant.");

    Swarm swarm(api_key, base_url);

    std::vector<nlohmann::json> messages;
    std::map<std::string, std::string> context;

    std::cout << "Chat with the Base Agent (type 'exit' to quit):" << std::endl;

    std::string user_input;
    while (true) {
        std::cout << "\nYou: ";
        std::getline(std::cin, user_input);

        if (user_input == "exit") {
            break;
        }

        // Add user message to the conversation
        messages.push_back({
            {"role", "user"},
            {"content", user_input}
        });

        auto response = swarm.run(agent, messages, context);

        // Print the assistant's response
        for (const auto& message : response.get_messages()) {
            if (message.find("role") != message.end() && 
                message.find("content") != message.end()) {
                
                if (message.at("role") == "assistant") {
                    std::cout << "\nAssistant: " << message.at("content") << std::endl;
                }
            }
        }

        
        for (const auto& msg : response.get_messages()) {
            messages.push_back(nlohmann::json(msg));
        }
    }

    return 0;
}