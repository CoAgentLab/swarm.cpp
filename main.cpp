#include "agents.h"
#include "swarm.hpp"
#include <iostream>


// Function to check weather (this is a mock implementation)
std::variant<std::string, Agent, std::map<std::string, std::string>> 
check_weather(const std::map<std::string, std::string>& params) {
    // In a real implementation, you would make an API call to a weather service
    // For this example, we'll return a mock response
    if (params.find("location") == params.end()) {
        return "Error: Location not provided";
    }
    
    return "The weather in " + params.at("location") + " is sunny with a temperature of 72°F";
}

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


int main() {
    // Create a weather agent
    Agent weather_agent;
    weather_agent.set_name("WeatherBot");
    weather_agent.set_model("deepseek-ai/DeepSeek-V2.5");
    weather_agent.set_instructions(
        "You are a helpful weather assistant. You can:\n"
        "1. Check current weather conditions\n"
        "2. Provide weather forecasts\n"
        "3. Report weather alerts\n"
        "Always ask for the location if not provided. For forecasts, ask for the number of days."
        "Use appropriate functions based on the user's request."
    );
    
    // Add the weather-related functions with proper metadata
    auto check_weather_metadata = create_function_metadata(
        "check_weather",
        "Get current weather conditions for a location",
        {{"location", "The city or location to check weather for"}},
        {"location"}
    );
    
    auto get_forecast_metadata = create_function_metadata(
        "get_forecast",
        "Get weather forecast for a specified number of days",
        {
            {"location", "The city or location to get forecast for"},
            {"days", "Number of days for the forecast (1-7)"}
        },
        {"location", "days"}
    );
    
    auto get_alerts_metadata = create_function_metadata(
        "get_weather_alerts",
        "Check for any active weather alerts or warnings",
        {{"location", "The city or location to check alerts for"}},
        {"location"}
    );
    
    // Add functions to the agent
    weather_agent.functions = {check_weather, get_forecast, get_weather_alerts};
    
    // Initialize the Swarm with your API credentials
    Swarm swarm("sk-hxjweswehodmsljfxlwbigqfhkfcxwyttwxpkxahajcvjmgr", 
                  "https://api.siliconflow.cn/v1");
    
    // Interactive loop for weather queries
    std::string user_input;
    std::vector<nlohmann::json> messages;
    std::map<std::string, std::string> context;
    
    std::cout << "WeatherBot initialized. Type 'quit' to exit.\n";
    
    while (true) {
        std::cout << "\nEnter your weather-related question: ";
        std::getline(std::cin, user_input);
        
        if (user_input == "quit") {
            break;
        }
        
        // Add user message to history
        messages = {
            {
                {"role", "user"},
                {"content", user_input}
            }
        };
        
        try {
            // Run the conversation
            Response response = swarm.run(
                weather_agent,
                messages,
                context,
                "",     // no model override
                false,  // no streaming
                true    // debug mode
            );
            
            // Print the assistant's response
            for (const auto& message : response.get_messages()) {
                if (message.find("role") != message.end() && 
                    message.find("content") != message.end()) {
                    
                    // Skip function messages in the output
                    if (message.at("role") == "function") {
                        continue;
                    }
                    
                    if (message.at("role") == "assistant") {
                        std::cout << "\nWeatherBot: " << message.at("content") << std::endl;
                    }
                }
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }
    
    return 0;
}