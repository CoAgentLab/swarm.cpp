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

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
