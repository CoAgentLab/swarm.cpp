#pragma once
#include <string>
#include <memory>
#include <vector>
#include <optional>
#include "http_client.h"

class ChatCompletion {
public:
    struct Message {
        std::string role;
        std::string content;
        
        // Builder pattern for fluent interface
        static Message system(const std::string& content) {
            return {"system", content};
        }
        static Message user(const std::string& content) {
            return {"user", content};
        }
        static Message assistant(const std::string& content) {
            return {"assistant", content};
        }
    };

    struct CompletionChoice {
        Message message;
        int index;
        std::string finish_reason;
    };

    struct Response {
        std::vector<CompletionChoice> choices;
        std::string id;
        std::string model;
        int64_t created;
    };

    class CreateParams {
    public:
        CreateParams& model(const std::string& m) { model_ = m; return *this; }
        CreateParams& messages(const std::vector<Message>& msgs) { messages_ = msgs; return *this; }
        CreateParams& max_tokens(int tokens) { max_tokens_ = tokens; return *this; }
        CreateParams& temperature(float temp) { temperature_ = temp; return *this; }
        CreateParams& stream(bool s) { stream_ = s; return *this; }
        
        CreateParams& frequency_penalty(float penalty) { frequency_penalty_ = penalty; return *this; }
        CreateParams& presence_penalty(float penalty) { presence_penalty_ = penalty; return *this; }
        CreateParams& top_p(float p) { top_p_ = p; return *this; }
        CreateParams& stop(const std::vector<std::string>& stop_sequences) { stop_ = stop_sequences; return *this; }
        CreateParams& logprobs(bool enable) { logprobs_ = enable; return *this; }
        CreateParams& top_logprobs(std::optional<int> n) { top_logprobs_ = n; return *this; }
        
        CreateParams& response_format(const std::string& type) { 
            response_format_ = {{"type", type}}; 
            return *this; 
        }
        
        nlohmann::json to_json() const {
            nlohmann::json payload;
            payload["model"] = model_;
            payload["max_tokens"] = max_tokens_;
            payload["temperature"] = temperature_;
            payload["stream"] = stream_;
            payload["frequency_penalty"] = frequency_penalty_;
            payload["presence_penalty"] = presence_penalty_;
            payload["top_p"] = top_p_;
            
            if (!stop_.empty()) {
                payload["stop"] = stop_;
            }
            
            if (logprobs_) {
                payload["logprobs"] = true;
                if (top_logprobs_) {
                    payload["top_logprobs"] = *top_logprobs_;
                }
            }
            
            if (!response_format_.empty()) {
                payload["response_format"] = response_format_;
            }
            
            nlohmann::json messages = nlohmann::json::array();
            for (const auto& msg : messages_) {
                messages.push_back({
                    {"role", msg.role},
                    {"content", msg.content}
                });
            }
            payload["messages"] = messages;
            return payload;
        }

    private:
        std::string model_;
        std::vector<Message> messages_;
        int max_tokens_ = 1024;
        float temperature_ = 0.7;
        bool stream_ = false;
        float frequency_penalty_ = 0.0;
        float presence_penalty_ = 0.0;
        float top_p_ = 1.0;
        std::vector<std::string> stop_;
        bool logprobs_ = false;
        std::optional<int> top_logprobs_;
        nlohmann::json response_format_;
    };

    static Response create(const CreateParams& params);
};

class OpenAI {
public:
    OpenAI(const std::string& api_key, const std::string& base_url = "https://api.openai.com/v1")
        : api_key_(api_key), base_url_(base_url) {}

    struct Chat {
        Chat(OpenAI& client) : client_(client) {}
        
        ChatCompletion::Response create(const ChatCompletion::CreateParams& params) {
            auto response = HttpClient::post_json(
                client_.base_url_,
                client_.api_key_,
                params.to_json()
            );
            
            return parse_response(response.json);
        }

    private:
        OpenAI& client_;
        
        ChatCompletion::Response parse_response(const nlohmann::json& json);
    };

    Chat chat{*this};

private:
    std::string api_key_;
    std::string base_url_;
};

ChatCompletion::Response OpenAI::Chat::parse_response(const nlohmann::json& json) {
    ChatCompletion::Response response;
    response.id = json["id"].get<std::string>();
    response.model = json["model"].get<std::string>();
    response.created = json["created"].get<int64_t>();
    
    for (const auto& choice : json["choices"]) {
        ChatCompletion::CompletionChoice completion_choice;
        completion_choice.index = choice["index"].get<int>();
        completion_choice.message = {
            choice["message"]["role"].get<std::string>(),
            choice["message"]["content"].get<std::string>()
        };
        completion_choice.finish_reason = choice["finish_reason"].get<std::string>();
        response.choices.push_back(completion_choice);
    }
    
    return response;
} 