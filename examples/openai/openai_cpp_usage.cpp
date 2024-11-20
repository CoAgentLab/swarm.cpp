#include "openai.hpp"
#include <iostream>

std::string api_key = "sk-face3f6903e24f778bbe44b21d82dc6e";
std::string base_url = "https://api.deepseek.com/chat/completions";

int main() {
    OpenAI client(api_key, base_url);
    std::vector<ChatCompletion::Message> messages = {
        ChatCompletion::Message::system("You are a helpful assistant")
    };
    
    std::string user_input;
    while (true) {
        // Get user input
        std::cout << "\nYou: ";
        std::getline(std::cin, user_input);
        
        // Check for exit condition
        if (user_input == "exit" || user_input == "quit") {
            break;
        }
        
        // Add user message to history
        messages.push_back(ChatCompletion::Message::user(user_input));
        
        // Get AI response
        auto response = client.chat.create(
            ChatCompletion::CreateParams()
                .model("deepseek-chat")
                .messages(messages)
                .max_tokens(1024)
                .temperature(0.7)
        );
        
        // Print AI response
        std::string ai_response = response.choices[0].message.content;
        std::cout << "\nAssistant: " << ai_response << std::endl;
        
        // Add AI response to history
        messages.push_back(ChatCompletion::Message::assistant(ai_response));
    }

    return 0;
}