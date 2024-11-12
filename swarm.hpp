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
#include <iostream>


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
        
        debug_print(debug, "Agent instructions: " + agent.get_instructions());
        // Create messages array starting with system instructions
        std::vector<nlohmann::json> messages = {
            {{"role", "system"}, {"content", agent.get_instructions()}}
        };
        
        // Add all history messages
        messages.insert(messages.end(), history.begin(), history.end());
        
        // Update payload with messages
        payload["messages"] = messages;
        
        // payload["echo"] = false;
        // payload["frequency_penalty"] = 0;
        // payload["logprobs"] = 0;
        // payload["max_tokens"] = 1024;
        // payload["presence_penalty"] = 0;
        // payload["stop"] = nullptr;
        // payload["stream_options"] = nullptr;
        // payload["suffix"] = nullptr;
        // payload["temperature"] = 1;
        // payload["top_p"] = 1;
        payload["stream"] = stream;

        // Add tool calls
        if (!agent.functions.empty()) {
            nlohmann::json tools = nlohmann::json::array();
            
            // Convert each function to a tool definition
            for (size_t i = 0; i < agent.functions.size(); i++) {
                nlohmann::json tool;
                // Note: You'll need to store function metadata alongside the functions
                // This is a simplified example - you may want to enhance this  
                function_to_json(agent.functions[i], tool);
                tools.push_back(tool);
            }
            
            payload["tools"] = tools;
            
            // Set tool_choice if specified
            if (!agent.tool_choice.empty()) {
                payload["tool_choice"] = agent.tool_choice;
            }
        }

        // Initialize CURL
        CURL* curl = curl_easy_init();
        std::string response_string;
        nlohmann::json response_json;

        if (curl) {
            // Update headers
            struct curl_slist* headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");
            headers = curl_slist_append(headers, "Accept: application/json");
            std::string auth_header = "Authorization: Bearer " + api_key_;
            headers = curl_slist_append(headers, auth_header.c_str());

            // Convert payload to string
            std::string payload_str = payload.dump();
            
            // Debug print the payload
            if (debug) {
                debug_print(debug, "Request payload: " + payload_str);
            }

            std::string request_url = base_url_ + ""; // "" for future use
            curl_easy_setopt(curl, CURLOPT_URL, request_url.c_str());
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

            std::cout << "Response from LLM: \n" << response_string << std::endl;
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

        // Convert DeepSeek response format to match expected format
        // nlohmann::json converted_response;
        // if (!response_json["choices"].empty()) {
        //     auto& choice = response_json["choices"][0];
        //     nlohmann::json message = {
        //         {"role", "assistant"}
        //     };

        //     // Handle content if present
        //     if (choice.contains("text")) {
        //         message["content"] = choice["text"];
        //     }

        //     // Handle tool calls if present
        //     if (choice.contains("tool_calls")) {
        //         message["tool_calls"] = choice["tool_calls"];
        //     }

        //     converted_response["choices"] = {{
        //         {"message", message}
        //     }};
        // }

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
        const std::vector<FunctionSignature>& functions,
        std::map<std::string, std::string>& context_variables,
        bool debug
    ) {
        Response response;
        
        for (const auto& tool_call : tool_calls) {
            std::string name = tool_call["function"]["name"];
            auto args = tool_call["function"]["arguments"];
            debug_print(debug, "Tool calling: " + name);
            debug_print(debug, "Tool calling arguments: " + args.dump());
            // Find matching function
            for (const auto& func : functions) {
                if (func.name == name) {
                    // Parse JSON arguments into a map
                    std::map<std::string, std::string> parsed_args = {};
                    if (args.is_string()) {
                        // Parse the JSON string into an object
                        auto args_obj = nlohmann::json::parse(args.get<std::string>());
                        for (auto& [key, value] : args_obj.items()) {
                            parsed_args[key] = value.get<std::string>();
                        }
                    } else if (args.is_object()) {
                        // Direct object conversion
                        for (auto& [key, value] : args.items()) {
                            parsed_args[key] = value.get<std::string>();
                        }
                    }

                    // Call the function with parsed arguments
                    std::cout << "executing function...: " << name << std::endl;
                    std::cout << "parsed_args: {";
                    for (const auto& [key, value] : parsed_args) {
                        std::cout << "\"" << key << "\": \"" << value << "\", ";
                    }
                    std::cout << "}" << std::endl;
                    auto raw_result = func.func(parsed_args);
                    if (debug) {
                        if (std::holds_alternative<std::string>(raw_result)) {
                            debug_print(debug, "Function raw result: " + std::get<std::string>(raw_result));
                        } else if (std::holds_alternative<Agent>(raw_result)) {
                            debug_print(debug, "Function returned an Agent");
                        }
                    }
                    Result result = handle_function_result(raw_result, debug);

                    // Add the function result to the response
                    if (result.has_value()) {
                        std::map<std::string, std::string> message{
                            {"role", "tool"},
                            {"name", name},
                            {"content", result.get_value()},
                            {"tool_call_id", tool_call["id"]},
                        };
                        response.add_message(message);
                    }

                    // Update: Create a deep copy of the agent before storing
                    if (result.has_agent()) {
                        auto agent_ptr = std::make_shared<Agent>(*result.get_agent());
                        response.set_agent(agent_ptr);
                    }
                    
                    if (!result.get_context().empty()) {
                        merge_fields(context_variables, result.get_context());
                    }
                    
                    break; // Exit the loop after finding and processing the matching function
                }
                else {
                    throw std::runtime_error("No matching function found for tool call: " + name);  
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
        std::shared_ptr<Agent> active_agent = std::make_shared<Agent>(agent);
        std::vector<nlohmann::json> history = messages;
        int init_len = messages.size();
        Response final_response;

        while (history.size() - init_len < max_turns && active_agent != nullptr) {
            print_agent_state(debug, *active_agent);
            nlohmann::json completion = get_chat_completion(
                *active_agent, history, context_variables, model_override, stream, debug
            );
            nlohmann::json message = completion["choices"][0]["message"];
            debug_print(debug, "Received message: " + message.dump());
            message["sender"] = active_agent->get_name();
            history.push_back(message);

            if (!execute_tools || !message.contains("tool_calls") || message["tool_calls"].empty()) {
                debug_print(debug, "Ending turn - no tool calls or tool execution disabled");
                break;
            }

            Response partial_response = handle_tool_calls(
                message["tool_calls"], active_agent->functions, context_variables, debug
            );
            
            nlohmann::json messages_json(partial_response.get_messages());
            debug_print(debug, "Partial response: " + messages_json.dump());
            
            // Update history with function responses
            for (const auto& msg : partial_response.get_messages()) {
                history.push_back(nlohmann::json(msg));
            }
            
            // Update active agent if changed
            if (partial_response.get_agent()) {
                active_agent = partial_response.get_agent();
            }

            // Update context variables
            merge_fields(context_variables, partial_response.get_context());
        }

        // Prepare final response
        std::cout << "Final response history: " << history.size() << std::endl;
        for (auto it = history.begin() + init_len; it != history.end(); ++it) {
            std::map<std::string, std::string> message;
            for (auto& [key, value] : it->items()) {
                message[key] = value.is_string() ? value.get<std::string>() : value.dump();
            }
            final_response.add_message(message);
        }
        // Debug output for final response messages
        if (debug) {
            nlohmann::json debug_messages = nlohmann::json::array();
            for (const auto& msg : final_response.get_messages()) {
                debug_messages.push_back(msg);
            }
            debug_print(debug, "Final response messages: " + debug_messages.dump(2));
        }
        final_response.set_agent(std::make_shared<Agent>(*active_agent));
        final_response.set_context(context_variables);

        return final_response;
    }

private:
    std::string api_key_;
    std::string base_url_;
};
