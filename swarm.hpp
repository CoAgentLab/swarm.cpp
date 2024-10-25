#pragma once

#include "agents.h"
#include "utils.h"
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <variant>
#include <curl/curl.h>


// Callback function to write response data
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

class Swarm {
public:
    Swarm(const std::string& api_key, const std::string& base_url) 
        : api_key_(api_key), base_url_(base_url) {}

    nlohmann::json get_chat_completion(
        const Agent& agent,
        const std::vector<nlohmann::json>& history,
        const std::map<std::string, std::string>& context_variables,
        const std::string& model_override,
        bool stream,
        bool debug
    ) {
        nlohmann::json payload;
        payload["model"] = model_override.empty() ? agent.get_model() : model_override;
        payload["messages"] = history;
        payload["stream"] = stream;

        // Convert functions to JSON using the improved util functions
        nlohmann::json tools_json = nlohmann::json::array();
        for (const auto& func : agent.functions) {
            nlohmann::json func_json;
            // Create metadata for each function
            auto metadata = create_function_metadata(
                "function_name",  // need to store function names somewhere
                "function description"  // And descriptions
            );
            function_to_json(func, metadata, func_json);
            tools_json.push_back(func_json);
        }
        
        if (!tools_json.empty()) {
            payload["functions"] = tools_json;
            payload["parallel_tool_calls"] = agent.parallel_tool_calls;
        }

        // Initialize CURL
        CURL* curl = curl_easy_init();
        std::string response_string;
        nlohmann::json response_json;

        if (curl) {
            // Set headers
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            std::string auth_header = "Authorization: Bearer " + api_key_;
            headers = curl_slist_append(headers, auth_header.c_str());

            // Convert payload to string
            std::string payload_str = payload.dump();

            // Set CURL options
            curl_easy_setopt(curl, CURLOPT_URL, (base_url_ + "/v1/chat/completions").c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload_str.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

            if (debug) {
                curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
            }

            // Perform the request
            CURLcode res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                // Clean up
                curl_slist_free_all(headers);
                curl_easy_cleanup(curl);
                throw std::runtime_error("CURL request failed: " + std::string(curl_easy_strerror(res)));
            }

            // Get HTTP response code
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

            // Clean up
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);

            // Parse response
            try {
                response_json = nlohmann::json::parse(response_string);
            } catch (const nlohmann::json::parse_error& e) {
                throw std::runtime_error("Failed to parse API response: " + std::string(e.what()));
            }

            // Check for API errors
            if (http_code != 200) {
                std::string error_message = response_json.contains("error") ? 
                    response_json["error"]["message"].get<std::string>() : 
                    "Unknown API error";
                throw std::runtime_error("API request failed with code " + 
                    std::to_string(http_code) + ": " + error_message);
            }
        } else {
            throw std::runtime_error("Failed to initialize CURL");
        }

        return response_json;
    }


    Result handle_function_result(
        const std::variant<std::string, Agent, std::map<std::string, std::string>>& result_var, 
        bool debug
    ) {
        Result result;
        
        if (std::holds_alternative<std::string>(result_var)) {
            result.set_value(std::get<std::string>(result_var));
        } 
        else if (std::holds_alternative<Agent>(result_var)) {
            result.set_agent(std::make_shared<Agent>(std::get<Agent>(result_var)));
        } 
        else if (std::holds_alternative<std::map<std::string, std::string>>(result_var)) {
            result.set_context(std::get<std::map<std::string, std::string>>(result_var));
        }

        debug_print(debug, "Function result processed: " + result.get_value());

        return result;
    }


    Response handle_tool_calls(
        const nlohmann::json& tool_calls,
        const std::vector<AgentFunction>& functions,
        std::map<std::string, std::string>& context_variables,
        bool debug
    ) {
        Response response;
        
        for (const auto& tool_call : tool_calls) {
            std::string name = tool_call["function"]["name"];
            auto args = tool_call["function"]["arguments"];
            
            // Find matching function
            for (const auto& func : functions) {
                auto raw_result = func(context_variables);
                Result result = handle_function_result(raw_result, debug);
                
                if (result.has_value()) {
                    std::map<std::string, std::string> message{
                        {"role", "function"},
                        {"name", name},
                        {"content", result.get_value()}
                    };
                    response.add_message(message);
                }
                
                if (result.has_agent()) {
                    response.set_agent(result.get_agent());
                }
                
                if (!result.get_context().empty()) {
                    merge_fields(context_variables, result.get_context());
                }
            }
        }
        
        return response;
    }


    Response run(
        Agent& agent,
        std::vector<nlohmann::json>& messages,
        std::map<std::string, std::string> context_variables,
        const std::string& model_override = "",
        bool stream = false,
        bool debug = false,
        int max_turns = 10,
        bool execute_tools = true
    ) {
        Agent* active_agent = &agent;
        std::vector<nlohmann::json> history = messages;
        int init_len = messages.size();
        Response final_response;

        while (history.size() - init_len < max_turns && active_agent != nullptr) {
            nlohmann::json completion = get_chat_completion(
                *active_agent, history, context_variables, model_override, stream, debug
            );
            
            nlohmann::json message = completion["choices"][0]["message"];
            debug_print(debug, "Received completion: " + message.dump());
            message["sender"] = active_agent->get_name();
            history.push_back(message);

            if (!message.contains("tool_calls") || !execute_tools) {
                debug_print(debug, "Ending turn - no tool calls or tool execution disabled");
                break;
            }

            Response partial_response = handle_tool_calls(
                message["tool_calls"], active_agent->functions, context_variables, debug
            );
            
            // Update history with function responses
            for (const auto& msg : partial_response.get_messages()) {
                history.push_back(nlohmann::json(msg));
            }
            
            // Update active agent if changed
            if (partial_response.get_agent()) {
                active_agent = partial_response.get_agent().get();
            }
            
            // Update context variables
            merge_fields(context_variables, partial_response.get_context());
        }

        // Prepare final response
        for (auto it = history.begin() + init_len; it != history.end(); ++it) {
            std::map<std::string, std::string> message;
            for (auto& [key, value] : it->items()) {
                message[key] = value.is_string() ? value.get<std::string>() : value.dump();
            }
            final_response.add_message(message);
        }
        
        final_response.set_agent(std::make_shared<Agent>(*active_agent));
        final_response.set_context(context_variables);

        return final_response;
    }

private:
    std::string api_key_;
    std::string base_url_;
};
