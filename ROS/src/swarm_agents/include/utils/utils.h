#pragma once

#include <string>
#include <iostream>
#include <map>
#include <functional>
#include <ctime>
#include <nlohmann/json.hpp>
#include "agent/agents.h"
#include <typeindex>
#include <any>


inline void debug_print(bool debug, const std::string& message) {
    if (!debug) return;
    
    std::time_t now = std::time(nullptr);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    
    // Check if message is JSON-formatted
    try {
        auto j = nlohmann::json::parse(message);
        
        // Add newlines before and after the timestamp
        std::cout << "\n[" << buf << "]\n" 
                 << "Debug Output:\n"
                 << "----------------------------------------\n"
                 << j.dump(2) << "\n"  // indent with 2 spaces
                 << "----------------------------------------\n" 
                 << std::endl;
    } catch (nlohmann::json::parse_error&) {
        // Not JSON, print normal message
        std::cout << "\n[" << buf << "] " << message << "\n" << std::endl;
    }
}

inline void print_agent_state(bool debug, const Agent& agent) {
    if (!debug) return;

    std::cout << "\nActive Agent State:"    
                << "\nName: " << agent.get_name()
                << "\nModel: " << agent.get_model() 
                << "\nInstructions: " << agent.get_instructions()
                << "\nNumber of functions: " << agent.functions.size()
                << "\nTool choice: " << agent.tool_choice
                << "\nParallel tool calls: " << (agent.parallel_tool_calls ? "true" : "false")
                << std::endl;
}

inline void merge_fields(std::map<std::string, std::string>& target, 
                         const std::map<std::string, std::string>& source) {
    for (const auto& [key, value] : source) {
        target[key] += value;
    }
}

// // General function to register any function
// inline FunctionSignature registerFunction(const std::string &name, const std::string &description, AgentFunction func, const std::vector<Parameter> &params, const std::vector<std::string> &required = {}) {
//     auto funcSig = create_function_signature(name, description, func, params, required);
//     return funcSig;
// }

// Define typeMap for JSON type conversion
inline const std::map<std::type_index, std::string> typeMap = {
    {typeid(std::string), "string"},
    {typeid(int), "integer"},
    {typeid(double), "number"},
    {typeid(float), "number"},
    {typeid(bool), "boolean"},
    {typeid(std::vector<std::any>), "array"},
    {typeid(std::map<std::string, std::any>), "object"},
    {typeid(std::nullptr_t), "null"}
};

inline void function_to_json(const FunctionSignature &funcSig,
                             nlohmann::json& json) {
    nlohmann::json properties = nlohmann::json::object();
    std::vector<std::string> required = {};

    for (const auto &param : funcSig.parameters) {
        std::string jsonType = typeMap.count(typeid(param.type)) ? 
            typeMap.at(typeid(param.type)) : "string";
            
        properties[param.name] = {
            {"type", jsonType},
            {"description", param.description}
        };
        if (std::find(funcSig.required_parameters.begin(), 
            funcSig.required_parameters.end(), param.name) != funcSig.required_parameters.end()) {
            required.push_back(param.name);
        }
    }

    json = {
        {"type", "function"},
        {"function", {
            {"name", funcSig.name},
            {"description", funcSig.description},
            {"parameters", {
                {"type", "object"},
                {"properties", properties},
                {"required", required},
                {"additionalProperties", false}
            }}
        }}
    };
}
