#pragma once

#include <string>
#include <iostream>
#include <map>
#include <functional>
#include <ctime>
#include <nlohmann/json.hpp>
#include "agents.h"


inline void debug_print(bool debug, const std::string& message) {
    if (!debug) return;
    std::time_t now = std::time(nullptr);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    std::cout << "[" << buf << "] " << message << std::endl;
}


inline void merge_fields(std::map<std::string, std::string>& target, 
                         const std::map<std::string, std::string>& source) {
    for (const auto& [key, value] : source) {
        target[key] += value;
    }
}

// a parameter of a function
struct Parameter {
    std::string name;
    std::string type;
    std::string description;
};

struct FunctionSignature {
    std::string name;
    std::string description;
    std::vector<Parameter> parameters;
    std::vector<std::string> required_parameters;
};

// Helper to automatically create a FunctionSignature
template <typename Func>
inline FunctionSignature create_function_signature(
    const std::string &name, 
    const std::string &description, 
    Func func, 
    const std::vector<Parameter> &params,
    const std::vector<std::string> &required = {}) 
{
    return FunctionSignature{
        .name = name,
        .description = description,
        .parameters = params,
        .required_parameters = required
    };
}


// General function to register any function
template <typename Func>
inline FunctionSignature registerFunction(const std::string &name, const std::string &description, Func func, const std::vector<Parameter> &params, const std::vector<std::string> &required = {}) {
    auto funcSig = create_function_signature(name, description, func, params, required);
    return funcSig;
}


inline void function_to_json(const AgentFunction& func, 
                             const FunctionMetadata& metadata,
                             nlohmann::json& json_output) {
    nlohmann::json properties;
    for (const auto& [param_name, param_desc] : metadata.parameters) {
        properties[param_name] = {
            {"type", "string"},
            {"description", param_desc}
        };
    }

    json_output = {
        {"type", "function"},
        {"function", {
            {"name", metadata.name},
            {"description", metadata.description},
            {"parameters", {
                {"type", "object"},
                {"properties", properties},
                {"required", metadata.required_parameters}
            }}
        }}
    };
}


// Convert FunctionSignature to JSON following the OpenAI function calling format
// https://platform.openai.com/docs/guides/function-calling#step-3-pass-your-function-definitions-as-available-tools-to-the-model-along-with-the-messages
inline void function_to_json(const AgentFunction& func, 
                             const FunctionSignature &funcSig,
                             nlohmann::json& json) {
    nlohmann::json properties;
    std::vector<std::string> required;

    for (const auto &param : funcSig.parameters) {
        std::string jsonType = typeMap[param.type];
        if (jsonType.empty()) {
            jsonType = "string"; // Default to string if type not found
        }
        properties[param_name] = {
            {"type", jsonType},
            {"description", param.description}
        };
        if (std::find(funcSig.required_parameters.begin(), funcSig.required_parameters.end(), param.name) != funcSig.required_parameters.end()) {
            required.push_back(param.name);
        }
    }
    // json["type"] = "function";
    // json["function"]["name"] = funcSig.name;
    // json["function"]["description"] = funcSig.description;
    // json["function"]["parameters"]["type"] = "object";
    // json["function"]["parameters"]["properties"] = properties;
    // json["function"]["parameters"]["required"] = required;
    // json["function"]["parameters"]["additionalProperties"] = "False";

    json = {
        {"type", "function"},
        {"function", {
            {"name", funcSig.name},
            {"description", funcSig.description},
            {"parameters", {
                {"type", "object"},
                {"properties", properties},
                {"required", required},
                {"additionalProperties", "False"}
            }}
        }}
    };
}
