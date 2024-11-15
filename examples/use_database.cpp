#include <swarm.hpp>
#include <iostream>
#include "agents.h"


std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
std::string base_url = "https://api.deepseek.com/chat/completions";

std::map<std::string, std::string> user_database = {
    {"name", "John Doe"},
    {"age", "30"},
    {"city", "New York"},
    {"country", "USA"},
    {"user_id", "12345"}
};

AgentFunction get_user_info = [](const std::map<std::string, std::string>& params) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    std::cout << "Getting user info" << std::endl;
    
    std::string name = user_database.at("name");
    std::string age = user_database.at("age");
    std::string city = user_database.at("city");
    std::string country = user_database.at("country");
    
    return name + " is " + age + " years old from " + city + ", " + country;
};

InstructionFunction instructions = [](const std::map<std::string, std::string>& context) -> std::string {
    return "You are a helpful agent with context: " + context.at("name");
};


int main() {
    Agent user("Agent", "deepseek-chat", instructions);
    user.functions = {create_function_signature<decltype(get_user_info)>(
        "get_user_info",
        "Get user information",
        get_user_info,
        {
            {"name", "The name of the user"},
            {"age", "The age of the user"},
            {"city", "The city of the user"},
            {"country", "The country of the user"}
        },
        {"name"}  // required parameters
    )};
    
    Swarm swarm(api_key, base_url);

    std::vector<nlohmann::json> messages = {
        {
            {"role", "user"},
            {"content", "What is my user id?"}
        }
    };  

    auto response = swarm.run(user, messages, user_database);

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