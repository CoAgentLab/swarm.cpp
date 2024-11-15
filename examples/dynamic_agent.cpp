#include "agents.h"
#include <iostream>


int main() {

    // Static instructions
    Agent static_agent("Static", "gpt-4", "You are a helpful agent.");

    // Dynamic instructions
    Agent dynamic_agent("Dynamic", "gpt-4", 
        [](const std::map<std::string, std::string>& context) {
            std::string result = "You are a helpful agent with context: ";
            return result + context.at("key");
        }
    );

    // Using the instructions
    std::map<std::string, std::string> context = {{"key", "value"}};
    std::string static_instructions = static_agent.get_instructions(context);  // Returns static string
    std::string dynamic_instructions = dynamic_agent.get_instructions(context);  // Returns dynamic string with context
    std::cout << static_instructions << std::endl;
    std::cout << dynamic_instructions << std::endl;
    return 0;
}
