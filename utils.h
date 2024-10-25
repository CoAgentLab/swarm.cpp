#pragma once

#include <string>
#include <iostream>
#include <map>
#include <functional>
#include <ctime>
#include <nlohmann/json.hpp>
#include "types.h"


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


// function_to_json with structured metadata
struct FunctionMetadata {
    std::string name;
    std::string description;
    std::map<std::string, std::string> parameters;  // parameter_name -> parameter_description
    std::vector<std::string> required_parameters;
};


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


// Helper function to create function metadata
inline FunctionMetadata create_function_metadata(
    const std::string& name,
    const std::string& description,
    const std::map<std::string, std::string>& parameters = {},
    const std::vector<std::string>& required = {}) {
    return FunctionMetadata{
        .name = name,
        .description = description,
        .parameters = parameters,
        .required_parameters = required
    };
}
