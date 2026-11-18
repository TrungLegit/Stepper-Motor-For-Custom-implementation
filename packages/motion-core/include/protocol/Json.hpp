#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <utility>
#include <string_view>
#include <optional>
#include <initializer_list>

namespace motion_core::protocol {

enum class JsonType {
    Null,
    Boolean,
    Number,
    String,
    Array,
    Object
};

/**
 * @brief Lightweight, zero-dependency C++17 JSON DOM value representation.
 */
class JsonValue {
public:
    JsonValue();
    JsonValue(std::nullptr_t);
    JsonValue(bool b);
    JsonValue(int v);
    JsonValue(int64_t v);
    JsonValue(uint32_t v);
    JsonValue(uint64_t v);
    JsonValue(float v);
    JsonValue(double v);
    JsonValue(const char* s);
    JsonValue(std::string s);
    JsonValue(std::string_view s);
    JsonValue(std::vector<JsonValue> arr);
    JsonValue(std::initializer_list<std::pair<std::string, JsonValue>> obj);

    // Factory methods
    static JsonValue array();
    static JsonValue object();

    // Type queries
    JsonType type() const noexcept { return m_type; }
    bool is_null() const noexcept { return m_type == JsonType::Null; }
    bool is_bool() const noexcept { return m_type == JsonType::Boolean; }
    bool is_number() const noexcept { return m_type == JsonType::Number; }
    bool is_string() const noexcept { return m_type == JsonType::String; }
    bool is_array() const noexcept { return m_type == JsonType::Array; }
    bool is_object() const noexcept { return m_type == JsonType::Object; }

    // Value extractors with defaults
    bool as_bool(bool def = false) const noexcept;
    double as_double(double def = 0.0) const noexcept;
    float as_float(float def = 0.0f) const noexcept;
    int32_t as_int32(int32_t def = 0) const noexcept;
    int64_t as_int(int64_t def = 0) const noexcept;
    const std::string& as_string() const;
    std::string as_string(const std::string& def) const;

    // Array operations
    const std::vector<JsonValue>& as_array() const;
    std::vector<JsonValue>& as_array();
    void push_back(JsonValue val);
    const JsonValue& operator[](size_t index) const;
    JsonValue& operator[](size_t index);

    // Object operations
    const std::vector<std::pair<std::string, JsonValue>>& as_object() const;
    std::vector<std::pair<std::string, JsonValue>>& as_object();
    bool contains(const std::string& key) const;
    const JsonValue* find(const std::string& key) const;
    JsonValue* find(const std::string& key);
    const JsonValue& get(const std::string& key) const;
    void set(const std::string& key, JsonValue val);
    JsonValue& operator[](const std::string& key);

    size_t size() const noexcept;
    bool empty() const noexcept;

    // Serialization
    std::string serialize(bool pretty = false, int indent = 0) const;

    // Parser
    static JsonValue parse(std::string_view json, std::string* error_msg = nullptr);
    static bool tryParse(std::string_view json, JsonValue& out_val, std::string* error_msg = nullptr);

    // Comparison for testing
    bool operator==(const JsonValue& other) const;
    bool operator!=(const JsonValue& other) const { return !(*this == other); }

private:
    JsonType m_type{JsonType::Null};
    bool m_bool_val{false};
    double m_num_val{0.0};
    bool m_is_integer{false};
    int64_t m_int_val{0};
    std::string m_str_val;
    std::vector<JsonValue> m_arr_val;
    std::vector<std::pair<std::string, JsonValue>> m_obj_val;

    static const JsonValue s_null_val;
};

} // namespace motion_core::protocol
