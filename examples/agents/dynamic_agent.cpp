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
    dynamic_agent.set_instructions(context); // Set the context first
    
    std::string static_instructions = static_agent.get_instructions();  // Now call without parameters
    std::string dynamic_instructions = dynamic_agent.get_instructions(); // Now call without parameters
    std::cout << static_instructions << std::endl;
    std::cout << dynamic_instructions << std::endl;
    return 0;
}
