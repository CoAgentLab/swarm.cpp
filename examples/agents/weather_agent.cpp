#include "agents.h"
#include "swarm.hpp"
#include <iostream>

// Direct AgentFunction implementations
AgentFunction check_weather = [](const std::map<std::string, std::string>& parameters) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    if (parameters.find("location") == parameters.end()) {
        std::cout << "Debug: Parameters received:" << std::endl;
        for (const auto& [key, value] : parameters) {
            std::cout << "  " << key << ": " << value << std::endl;
        }
        return "Error: Location not provided";
    }
    
    return "The weather in " + parameters.at("location") + " is sunny with a temperature of 72°F";
};

AgentFunction get_forecast = [](const std::map<std::string, std::string>& parameters) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    if (parameters.find("location") == parameters.end()) {
        return "Error: Location not provided";
    }
    if (parameters.find("days") == parameters.end()) {
        return "Error: Number of days not provided";
    }

    return "The " + parameters.at("days") + "-day forecast for " + parameters.at("location") + 
           " shows mild temperatures with partly cloudy skies.";
};

AgentFunction get_weather_alerts = [](const std::map<std::string, std::string>& parameters) -> std::variant<std::string, Agent, std::map<std::string, std::string>> {
    if (parameters.find("location") == parameters.end()) {
        return "Error: Location not provided";
    }
    
    return "No current weather alerts for " + parameters.at("location");
};

int main() {
    // Create a weather agent
    Agent weather_agent("WeatherBot", "deepseek-chat", 
        "You are a helpful weather assistant. You can:\n"
        "1. Check current weather conditions\n"
        "2. Provide weather forecasts\n"
        "3. Report weather alerts\n"
        "Always ask for the location if not provided. For forecasts, ask for the number of days."
        "Use appropriate functions based on the user's request."
    );
    
    // Add the weather-related functions with proper metadata
    auto check_weather_metadata = create_function_signature<decltype(check_weather)>(
        "check_weather",
        "Get current weather conditions for a location",
        check_weather,
        std::vector<Parameter>{
            // not initialized correctly, to be fixed
            {"location", "The city or location to check weather for"}
        },
        std::vector<std::string>{"location"}
    );
    
    auto get_forecast_metadata = create_function_signature<decltype(get_forecast)>(
        "get_forecast",
        "Get weather forecast for a specified number of days",
        get_forecast,
        {{"location", "The city or location to get forecast for"},
            {"days", "Number of days for the forecast (1-7)"}
        },
        {"location", "days"}
    );
    
    auto get_alerts_metadata = create_function_signature<decltype(get_weather_alerts)>(
        "get_weather_alerts",
        "Check for any active weather alerts or warnings",
        get_weather_alerts,
        {{"location", "The city or location to check alerts for"}},
        {"location"}
    );
    
    // Add functions to the agent
    weather_agent.functions = {check_weather_metadata, get_forecast_metadata, get_alerts_metadata};
    
    // Initialize the Swarm with your API credentials
    std::string api_key = "your_api_key";
    std::string base_url = "https://api.deepseek.com/chat/completions";
    Swarm swarm(api_key, base_url);
                
    
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
