#include <gtest/gtest.h>
#include "../util.h"
#include <sstream>

// Test debug_print function
TEST(UtilTest, DebugPrint) {
    std::stringstream buffer;
    std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

    debug_print(true, "Test message");
    std::string output = buffer.str();
    EXPECT_TRUE(output.find("Test message") != std::string::npos);

    buffer.str("");
    debug_print(false, "Should not print");
    EXPECT_TRUE(buffer.str().empty());

    std::cout.rdbuf(old);
}

// Test merge_fields function
TEST(UtilTest, MergeFields) {
    std::map<std::string, std::string> target = {{"key1", "value1"}, {"key2", "value2"}};
    std::map<std::string, std::string> source = {{"key2", " appended"}, {"key3", "value3"}};

    merge_fields(target, source);

    EXPECT_EQ(target["key1"], "value1");
    EXPECT_EQ(target["key2"], "value2 appended");
    EXPECT_EQ(target["key3"], "value3");
}

// Test create_function_metadata and function_to_json
TEST(UtilTest, FunctionToJson) {
    auto metadata = create_function_metadata(
        "test_function",
        "A test function",
        {{"param1", "First parameter"}, {"param2", "Second parameter"}},
        {"param1"}
    );

    nlohmann::json json_output;
    function_to_json([](const std::map<std::string, std::string>&) { return ""; }, metadata, json_output);

    EXPECT_EQ(json_output["function"]["name"], "test_function");
    EXPECT_EQ(json_output["function"]["description"], "A test function");
    EXPECT_EQ(json_output["function"]["parameters"]["properties"]["param1"]["description"], "First parameter");
    EXPECT_EQ(json_output["function"]["parameters"]["properties"]["param2"]["description"], "Second parameter");
    EXPECT_EQ(json_output["function"]["parameters"]["required"], nlohmann::json::array({"param1"}));
}


//Test class Agent in types.h
TEST(AgentTest, AgentConstructor) {
    Agent agent;
    EXPECT_EQ(agent.name, "Agent");
    EXPECT_EQ(agent.model, "gpt-4o");
    EXPECT_EQ(agent.instructions, "You are a helpful agent.");
    EXPECT_EQ(agent.functions.size(), 0);
    EXPECT_EQ(agent.tool_choice, "");
    EXPECT_TRUE(agent.parallel_tool_calls);
}

TEST(AgentTest, AgentCopyConstructor) {
    Agent agent;
    Agent copy_agent(agent);
    EXPECT_EQ(copy_agent.name, "Agent");
    EXPECT_EQ(copy_agent.model, "gpt-4o");
    EXPECT_EQ(copy_agent.instructions, "You are a helpful agent.");
    EXPECT_EQ(copy_agent.functions.size(), 0);
    EXPECT_EQ(copy_agent.tool_choice, "");
    EXPECT_TRUE(copy_agent.parallel_tool_calls);
}

TEST(AgentTest, AgentMoveConstructor) {
    Agent agent;
    Agent copy_agent(std::move(agent));
    EXPECT_EQ(copy_agent.name, "Agent");
    EXPECT_EQ(copy_agent.model, "gpt-4o");
    EXPECT_EQ(copy_agent.instructions, "You are a helpful agent.");
    EXPECT_EQ(copy_agent.functions.size(), 0);
    EXPECT_EQ(copy_agent.tool_choice, "");
    EXPECT_TRUE(copy_agent.parallel_tool_calls);
}

TEST(AgentTest, AgentAssignment) {
    Agent agent;
    Agent copy_agent;
    copy_agent = agent;
    EXPECT_EQ(copy_agent.name, "Agent");
    EXPECT_EQ(copy_agent.model, "gpt-4o");
    EXPECT_EQ(copy_agent.instructions, "You are a helpful agent.");
    EXPECT_EQ(copy_agent.functions.size(), 0);
    EXPECT_EQ(copy_agent.tool_choice, "");
    EXPECT_TRUE(copy_agent.parallel_tool_calls);
}

TEST(AgentTest, AgentMoveAssignment) {
    Agent agent;
    Agent copy_agent;
    copy_agent = std::move(agent);
    EXPECT_EQ(copy_agent.name, "Agent");
    EXPECT_EQ(copy_agent.model, "gpt-4o");
    EXPECT_EQ(copy_agent.instructions, "You are a helpful agent.");
    EXPECT_EQ(copy_agent.functions.size(), 0);
    EXPECT_EQ(copy_agent.tool_choice, "");
    EXPECT_TRUE(copy_agent.parallel_tool_calls);
}

TEST(AgentTest, Agentassignment) {
    Agent agent;
    agent.set_name("Test Agent");
    EXPECT_EQ(agent.get_name(), "Test Agent");

    agent.set_model("chatgpt o1");
    EXPECT_EQ(agent.get_model(), "chatgpt o1");

    agent.set_instructions("You are a test agent.");
    EXPECT_EQ(agent.get_instructions(), "You are a test agent.");
}


// Test Response class in types.h
TEST(ResponseTest, ResponseConstructor) {
    Response response;
    EXPECT_EQ(response.messages.size(), 0);
    EXPECT_EQ(response.agent, nullptr);
    EXPECT_EQ(response.context_variables.size(), 0);
}

TEST(ResponseTest, ResponseCopyConstructor) {
    Response response;
    Response copy_response(response);
    EXPECT_EQ(copy_response.messages.size(), 0);
    EXPECT_EQ(copy_response.agent, nullptr);
    EXPECT_EQ(copy_response.context_variables.size(), 0);
}

TEST(ResponseTest, ResponseMoveConstructor) {
    Response response;
    Response copy_response(std::move(response));
    EXPECT_EQ(copy_response.messages.size(), 0);
    EXPECT_EQ(copy_response.agent, nullptr);
    EXPECT_EQ(copy_response.context_variables.size(), 0);
}

TEST(ResponseTest, ResponseAssignment) {
    Response response;
    Response copy_response;
    copy_response = response;
    EXPECT_EQ(copy_response.messages.size(), 0);
    EXPECT_EQ(copy_response.agent, nullptr);
    EXPECT_EQ(copy_response.context_variables.size(), 0);
}

TEST(ResponseTest, ResponseMoveAssignment) {
    Response response;
    Response copy_response;
    copy_response = std::move(response);
    EXPECT_EQ(copy_response.messages.size(), 0);
    EXPECT_EQ(copy_response.agent, nullptr);
    EXPECT_EQ(copy_response.context_variables.size(), 0);
}


TEST(ResponseTest, ResponseModify) {
    Response response;
    response.add_message({{"Messagekey", "Messagevalue"}});
    EXPECT_EQ(response.messages.size(), 1);
    EXPECT_EQ(response.messages[0]["Messagekey"], "Messagevalue");

    response.set_agent(std::make_shared<Agent>());
    EXPECT_EQ(response.agent->name, "Agent");
    EXPECT_EQ(response.agent->model, "gpt-4o");
    EXPECT_EQ(response.agent->instructions, "You are a helpful agent.");

    response.set_context({{"Contextkey", "Contextvalue"}});
    EXPECT_EQ(response.context_variables["Contextkey"], "Contextvalue");
}


// Test Result class in types.h
TEST(ResultTest, ResultConstructor) {
    Result result;
    EXPECT_EQ(result.value, "");
    EXPECT_EQ(result.agent, nullptr);
    EXPECT_EQ(result.context_variables.size(), 0);
}

TEST(ResultTest, ResultCopyConstructor) {
    Result result;
    Result copy_result(result);
    EXPECT_EQ(copy_result.value, "");
    EXPECT_EQ(copy_result.agent, nullptr);
    EXPECT_EQ(copy_result.context_variables.size(), 0);
}

TEST(ResultTest, ResultMoveConstructor) {
    Result result;
    Result copy_result(std::move(result));
    EXPECT_EQ(copy_result.value, "");
    EXPECT_EQ(copy_result.agent, nullptr);
    EXPECT_EQ(copy_result.context_variables.size(), 0);
}

TEST(ResultTest, ResultCopyAssignment) {
    Result result;
    Result copy_result;
    copy_result = result;
    EXPECT_EQ(copy_result.value, "");
    EXPECT_EQ(copy_result.agent, nullptr);
    EXPECT_EQ(copy_result.context_variables.size(), 0);
}

TEST(ResultTest, ResultMoveAssignment) {
    Result result;
    Result copy_result;
    copy_result = std::move(result);
    EXPECT_EQ(copy_result.value, "");
    EXPECT_EQ(copy_result.agent, nullptr);
    EXPECT_EQ(copy_result.context_variables.size(), 0);
}

TEST(ResultTest, ResultModify) {
    Result result;

    result.set_value("Test value");
    EXPECT_EQ(result.value, "Test value");

    result.set_agent(std::make_shared<Agent>());
    EXPECT_EQ(result.agent->name, "Agent");
    EXPECT_EQ(result.agent->model, "gpt-4o");
    EXPECT_EQ(result.agent->instructions, "You are a helpful agent.");

    result.set_context({{"Contextkey", "Contextvalue"}});
    EXPECT_EQ(result.context_variables["Contextkey"], "Contextvalue");
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}