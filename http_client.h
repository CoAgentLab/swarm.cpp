#pragma once
#include <string>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

class HttpClient {
public:
    struct Response {
        int status_code;
        std::string body;
    };

    static Response post_json(
        const std::string& url,
        const std::string& api_key,
        const nlohmann::json& payload,
        bool debug = false
    );

private:
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp);
}; 

