#pragma once

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <memory>
#include <variant>

// Forward declaration
class Agent;

/// Function type that processes agent requests and returns either a string response,
/// a new Agent instance, or a context map
using AgentFunction = std::function<std::variant<std::string, Agent, std::map<std::string, std::string>>(const std::map<std::string, std::string>&)>;


class Agent {
public:
    std::string name = "Agent";
    std::string model = "gpt-4o";
    std::string instructions = "You are a helpful agent.";
    std::vector<AgentFunction> functions;
    std::string tool_choice;
    bool parallel_tool_calls = true;

    // Default constructor
    Agent() = default;

    // Virtual destructor for safe inheritance
    virtual ~Agent() = default;

    // Copy constructor and assignment
    Agent(const Agent&) = default;
    Agent& operator=(const Agent&) = default;

    // Move constructor and assignment
    Agent(Agent&&) noexcept = default;
    Agent& operator=(Agent&&) noexcept = default;

    // Virtual methods that derived classes might want to override
    virtual std::string get_name() const { return name; }
    virtual void set_name(const std::string& new_name) { name = new_name; }

    virtual std::string get_model() const { return model; }
    virtual void set_model(const std::string& new_model) { model = new_model; }

    virtual std::string get_instructions() const { return instructions; }
    virtual void set_instructions(const std::string& new_instructions) { instructions = new_instructions; }

protected:
    // Protected methods that derived classes can use
    virtual void initialize() {}  // Hook for derived classes to initialize
};


class Response {
public:
    // Data members
    std::vector<std::map<std::string, std::string>> messages;
    std::shared_ptr<Agent> agent;
    std::map<std::string, std::string> context_variables;

    // Default constructor
    Response() = default;

    // Virtual destructor for inheritance safety
    virtual ~Response() = default;

    // Copy constructor and assignment
    Response(const Response&) = default;
    Response& operator=(const Response&) = default;

    // Move constructor and assignment
    Response(Response&&) noexcept = default;
    Response& operator=(Response&&) noexcept = default;

    // Virtual methods for accessing/modifying data
    virtual const std::vector<std::map<std::string, std::string>>& get_messages() const { return messages; }
    virtual void add_message(const std::map<std::string, std::string>& message) { messages.push_back(message); }

    virtual std::shared_ptr<Agent> get_agent() const { return agent; }
    virtual void set_agent(std::shared_ptr<Agent> new_agent) { agent = std::move(new_agent); }

    virtual const std::map<std::string, std::string>& get_context() const { return context_variables; }
    virtual void set_context(const std::map<std::string, std::string>& context) { context_variables = context; }

protected:
    virtual void initialize() {}  // Hook for derived classes
};


class Result {
public:
    // Data members
    std::string value;
    std::shared_ptr<Agent> agent;
    std::map<std::string, std::string> context_variables;

    // Default constructor
    Result() = default;

    // Virtual destructor for inheritance safety
    virtual ~Result() = default;

    // Copy constructor and assignment
    Result(const Result&) = default;
    Result& operator=(const Result&) = default;

    // Move constructor and assignment
    Result(Result&&) noexcept = default;
    Result& operator=(Result&&) noexcept = default;

    // Virtual methods for accessing/modifying data
    virtual std::string get_value() const { return value; }
    virtual void set_value(const std::string& new_value) { value = new_value; }

    virtual std::shared_ptr<Agent> get_agent() const { return agent; }
    virtual void set_agent(std::shared_ptr<Agent> new_agent) { agent = std::move(new_agent); }

    virtual const std::map<std::string, std::string>& get_context() const { return context_variables; }
    virtual void set_context(const std::map<std::string, std::string>& context) { context_variables = context; }

    // Utility methods
    [[nodiscard]] virtual bool has_value() const { return !value.empty(); }
    [[nodiscard]] virtual bool has_agent() const { return agent != nullptr; }

protected:
    virtual void initialize() {}  // Hook for derived classes
};
