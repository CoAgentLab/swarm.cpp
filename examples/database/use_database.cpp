#include <swarm.hpp>
#include <iostream>
#include "agents.h"


std::string api_key = "your_api_key";
std::string base_url = "https://api.deepseek.com/chat/completions";

std::map<std::string, std::string> user_database = {
    {"name", "John Doe"},
    {"age", "30"},
    {"city", "New York"},
    {"country", "USA"},
    {"user_id", "123456"}
};

AgentFunction get_user_info = [](const std::map<std::string, std::string>& params) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    std::cout << "Getting user info for key: " << params.at("key") << std::endl;
    
    try {
        return user_database.at(params.at("key"));
    } catch (const std::out_of_range& e) {
        return "Information not found for key: " + params.at("key");
    }
};

InstructionFunction instructions = [](const std::map<std::string, std::string>& context) -> std::string {
    return "You are a helpful agent with context: " + context.at("name");
};


int main() {
    Agent user("Agent", "deepseek-chat", instructions);
    user.functions = {create_function_signature<decltype(get_user_info)>(
        "get_user_info", // function name
        "Get specific user information by key", // function description
        get_user_info, // function implementation
        {
            {"key", "The key to look up in the user database"} // parameter description
        },
        {"key"}  // required parameters
    )};
    
    Swarm swarm(api_key, base_url);

    // Get user input for the key they want to check
    std::string user_query;
    std::cout << "What information would you like to check? (name/age/city/country/user_id): ";
    std::getline(std::cin, user_query);

    std::vector<nlohmann::json> messages = {
        {
            {"role", "user"},
            {"content", "What is my " + user_query + "?"}
        }
    };  

    std::cout << "Running swarm" << std::endl;
    auto response = swarm.run(user, messages, user_database, "", false, true);

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