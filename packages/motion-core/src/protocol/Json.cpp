#include "protocol/Json.hpp"
#include <sstream>
#include <iomanip>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <charconv>

namespace motion_core::protocol {

const JsonValue JsonValue::s_null_val{};

JsonValue::JsonValue() : m_type(JsonType::Null) {}
JsonValue::JsonValue(std::nullptr_t) : m_type(JsonType::Null) {}
JsonValue::JsonValue(bool b) : m_type(JsonType::Boolean), m_bool_val(b) {}
JsonValue::JsonValue(int v) : m_type(JsonType::Number), m_num_val(static_cast<double>(v)), m_is_integer(true), m_int_val(v) {}
JsonValue::JsonValue(int64_t v) : m_type(JsonType::Number), m_num_val(static_cast<double>(v)), m_is_integer(true), m_int_val(v) {}
JsonValue::JsonValue(uint32_t v) : m_type(JsonType::Number), m_num_val(static_cast<double>(v)), m_is_integer(true), m_int_val(static_cast<int64_t>(v)) {}
JsonValue::JsonValue(uint64_t v) : m_type(JsonType::Number), m_num_val(static_cast<double>(v)), m_is_integer(true), m_int_val(static_cast<int64_t>(v)) {}
JsonValue::JsonValue(float v) : m_type(JsonType::Number), m_num_val(static_cast<double>(v)), m_is_integer(false) {}
JsonValue::JsonValue(double v) : m_type(JsonType::Number), m_num_val(v), m_is_integer(false) {}
JsonValue::JsonValue(const char* s) : m_type(JsonType::String), m_str_val(s ? s : "") {}
JsonValue::JsonValue(std::string s) : m_type(JsonType::String), m_str_val(std::move(s)) {}
JsonValue::JsonValue(std::string_view s) : m_type(JsonType::String), m_str_val(s) {}
JsonValue::JsonValue(std::vector<JsonValue> arr) : m_type(JsonType::Array), m_arr_val(std::move(arr)) {}
JsonValue::JsonValue(std::initializer_list<std::pair<std::string, JsonValue>> obj)
    : m_type(JsonType::Object), m_obj_val(obj) {}

JsonValue JsonValue::array() {
    JsonValue v;
    v.m_type = JsonType::Array;
    return v;
}

JsonValue JsonValue::object() {
    JsonValue v;
    v.m_type = JsonType::Object;
    return v;
}

bool JsonValue::as_bool(bool def) const noexcept {
    return (m_type == JsonType::Boolean) ? m_bool_val : def;
}

double JsonValue::as_double(double def) const noexcept {
    if (m_type == JsonType::Number) {
        return m_num_val;
    }
    return def;
}

float JsonValue::as_float(float def) const noexcept {
    if (m_type == JsonType::Number) {
        return static_cast<float>(m_num_val);
    }
    return def;
}

int32_t JsonValue::as_int32(int32_t def) const noexcept {
    if (m_type == JsonType::Number) {
        return m_is_integer ? static_cast<int32_t>(m_int_val) : static_cast<int32_t>(m_num_val);
    }
    return def;
}

int64_t JsonValue::as_int(int64_t def) const noexcept {
    if (m_type == JsonType::Number) {
        return m_is_integer ? m_int_val : static_cast<int64_t>(m_num_val);
    }
    return def;
}

const std::string& JsonValue::as_string() const {
    if (m_type == JsonType::String) {
        return m_str_val;
    }
    static const std::string s_empty;
    return s_empty;
}

std::string JsonValue::as_string(const std::string& def) const {
    if (m_type == JsonType::String) {
        return m_str_val;
    }
    return def;
}

const std::vector<JsonValue>& JsonValue::as_array() const {
    return m_arr_val;
}

std::vector<JsonValue>& JsonValue::as_array() {
    if (m_type != JsonType::Array) {
        m_type = JsonType::Array;
        m_arr_val.clear();
    }
    return m_arr_val;
}

void JsonValue::push_back(JsonValue val) {
    if (m_type != JsonType::Array) {
        m_type = JsonType::Array;
    }
    m_arr_val.push_back(std::move(val));
}

const JsonValue& JsonValue::operator[](size_t index) const {
    if (m_type == JsonType::Array && index < m_arr_val.size()) {
        return m_arr_val[index];
    }
    return s_null_val;
}

JsonValue& JsonValue::operator[](size_t index) {
    if (m_type != JsonType::Array) {
        m_type = JsonType::Array;
        m_arr_val.clear();
    }
    if (index >= m_arr_val.size()) {
        m_arr_val.resize(index + 1);
    }
    return m_arr_val[index];
}

const std::vector<std::pair<std::string, JsonValue>>& JsonValue::as_object() const {
    return m_obj_val;
}

std::vector<std::pair<std::string, JsonValue>>& JsonValue::as_object() {
    if (m_type != JsonType::Object) {
        m_type = JsonType::Object;
        m_obj_val.clear();
    }
    return m_obj_val;
}

bool JsonValue::contains(const std::string& key) const {
    return find(key) != nullptr;
}

const JsonValue* JsonValue::find(const std::string& key) const {
    if (m_type != JsonType::Object) return nullptr;
    for (const auto& pair : m_obj_val) {
        if (pair.first == key) return &pair.second;
    }
    return nullptr;
}

JsonValue* JsonValue::find(const std::string& key) {
    if (m_type != JsonType::Object) return nullptr;
    for (auto& pair : m_obj_val) {
        if (pair.first == key) return &pair.second;
    }
    return nullptr;
}

const JsonValue& JsonValue::get(const std::string& key) const {
    const JsonValue* v = find(key);
    return v ? *v : s_null_val;
}

void JsonValue::set(const std::string& key, JsonValue val) {
    if (m_type != JsonType::Object) {
        m_type = JsonType::Object;
        m_obj_val.clear();
    }
    for (auto& pair : m_obj_val) {
        if (pair.first == key) {
            pair.second = std::move(val);
            return;
        }
    }
    m_obj_val.emplace_back(key, std::move(val));
}

JsonValue& JsonValue::operator[](const std::string& key) {
    if (m_type != JsonType::Object) {
        m_type = JsonType::Object;
        m_obj_val.clear();
    }
    for (auto& pair : m_obj_val) {
        if (pair.first == key) {
            return pair.second;
        }
    }
    m_obj_val.emplace_back(key, JsonValue());
    return m_obj_val.back().second;
}

size_t JsonValue::size() const noexcept {
    if (m_type == JsonType::Array) return m_arr_val.size();
    if (m_type == JsonType::Object) return m_obj_val.size();
    return 0;
}

bool JsonValue::empty() const noexcept {
    return size() == 0;
}

bool JsonValue::operator==(const JsonValue& other) const {
    if (m_type != other.m_type) return false;
    switch (m_type) {
        case JsonType::Null: return true;
        case JsonType::Boolean: return m_bool_val == other.m_bool_val;
        case JsonType::Number:
            if (m_is_integer && other.m_is_integer) return m_int_val == other.m_int_val;
            return std::abs(m_num_val - other.m_num_val) < 1e-9;
        case JsonType::String: return m_str_val == other.m_str_val;
        case JsonType::Array: return m_arr_val == other.m_arr_val;
        case JsonType::Object: return m_obj_val == other.m_obj_val;
    }
    return false;
}

namespace {

void escapeJsonString(std::string_view s, std::string& out) {
    out.push_back('"');
    for (char c : s) {
        switch (c) {
            case '"': out.append("\\\""); break;
            case '\\': out.append("\\\\"); break;
            case '\b': out.append("\\b"); break;
            case '\f': out.append("\\f"); break;
            case '\n': out.append("\\n"); break;
            case '\r': out.append("\\r"); break;
            case '\t': out.append("\\t"); break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(c));
                    out.append(buf);
                } else {
                    out.push_back(c);
                }
                break;
        }
    }
    out.push_back('"');
}

void serializeImpl(const JsonValue& val, std::string& out, bool pretty, int indent) {
    switch (val.type()) {
        case JsonType::Null:
            out.append("null");
            break;
        case JsonType::Boolean:
            out.append(val.as_bool() ? "true" : "false");
            break;
        case JsonType::Number: {
            // Check if it should be serialized as an integer
            double d = val.as_double();
            if (std::floor(d) == d && !std::isinf(d) && !std::isnan(d) &&
                d >= -9223372036854775808.0 && d <= 9223372036854775807.0) {
                out.append(std::to_string(static_cast<int64_t>(d)));
            } else {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.6g", d);
                out.append(buf);
            }
            break;
        }
        case JsonType::String:
            escapeJsonString(val.as_string(), out);
            break;
        case JsonType::Array: {
            const auto& arr = val.as_array();
            if (arr.empty()) {
                out.append("[]");
                break;
            }
            out.push_back('[');
            for (size_t i = 0; i < arr.size(); ++i) {
                if (i > 0) out.append(pretty ? ", " : ",");
                serializeImpl(arr[i], out, pretty, indent + 1);
            }
            out.push_back(']');
            break;
        }
        case JsonType::Object: {
            const auto& obj = val.as_object();
            if (obj.empty()) {
                out.append("{}");
                break;
            }
            out.push_back('{');
            for (size_t i = 0; i < obj.size(); ++i) {
                if (i > 0) out.append(pretty ? ", " : ",");
                escapeJsonString(obj[i].first, out);
                out.append(pretty ? ": " : ":");
                serializeImpl(obj[i].second, out, pretty, indent + 1);
            }
            out.push_back('}');
            break;
        }
    }
}

class JsonParser {
public:
    explicit JsonParser(std::string_view json) : m_src(json), m_pos(0) {}

    bool parse(JsonValue& out, std::string* error_msg) {
        skipWhitespace();
        if (m_pos >= m_src.size()) {
            setError(error_msg, "Empty input");
            return false;
        }
        if (!parseValue(out, error_msg)) {
            return false;
        }
        skipWhitespace();
        if (m_pos < m_src.size()) {
            setError(error_msg, "Unexpected trailing characters at pos " + std::to_string(m_pos));
            return false;
        }
        return true;
    }

private:
    std::string_view m_src;
    size_t m_pos;

    void skipWhitespace() {
        while (m_pos < m_src.size()) {
            char c = m_src[m_pos];
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                m_pos++;
            } else {
                break;
            }
        }
    }

    char peek() const {
        return m_pos < m_src.size() ? m_src[m_pos] : '\0';
    }

    char get() {
        return m_pos < m_src.size() ? m_src[m_pos++] : '\0';
    }

    void setError(std::string* err, std::string_view msg) {
        if (err) {
            *err = std::string(msg) + " at offset " + std::to_string(m_pos);
        }
    }

    bool parseValue(JsonValue& out, std::string* error_msg) {
        skipWhitespace();
        char c = peek();
        if (c == '\0') {
            setError(error_msg, "Unexpected end of input");
            return false;
        }
        if (c == '{') return parseObject(out, error_msg);
        if (c == '[') return parseArray(out, error_msg);
        if (c == '"') return parseStringValue(out, error_msg);
        if (c == 't' || c == 'f') return parseBool(out, error_msg);
        if (c == 'n') return parseNull(out, error_msg);
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber(out, error_msg);

        setError(error_msg, std::string("Unexpected character '") + c + "'");
        return false;
    }

    bool parseObject(JsonValue& out, std::string* error_msg) {
        get(); // consume '{'
        out = JsonValue::object();
        skipWhitespace();
        if (peek() == '}') {
            get();
            return true;
        }

        while (m_pos < m_src.size()) {
            skipWhitespace();
            if (peek() != '"') {
                setError(error_msg, "Expected string key in object");
                return false;
            }
            std::string key;
            if (!parseStringRaw(key, error_msg)) return false;

            skipWhitespace();
            if (get() != ':') {
                setError(error_msg, "Expected ':' after key in object");
                return false;
            }

            JsonValue val;
            if (!parseValue(val, error_msg)) return false;
            out.set(key, std::move(val));

            skipWhitespace();
            char next = get();
            if (next == '}') return true;
            if (next != ',') {
                setError(error_msg, "Expected ',' or '}' in object");
                return false;
            }
        }
        setError(error_msg, "Unterminated object");
        return false;
    }

    bool parseArray(JsonValue& out, std::string* error_msg) {
        get(); // consume '['
        out = JsonValue::array();
        skipWhitespace();
        if (peek() == ']') {
            get();
            return true;
        }

        while (m_pos < m_src.size()) {
            JsonValue val;
            if (!parseValue(val, error_msg)) return false;
            out.push_back(std::move(val));

            skipWhitespace();
            char next = get();
            if (next == ']') return true;
            if (next != ',') {
                setError(error_msg, "Expected ',' or ']' in array");
                return false;
            }
        }
        setError(error_msg, "Unterminated array");
        return false;
    }

    bool parseStringValue(JsonValue& out, std::string* error_msg) {
        std::string s;
        if (!parseStringRaw(s, error_msg)) return false;
        out = JsonValue(std::move(s));
        return true;
    }

    bool parseStringRaw(std::string& out, std::string* error_msg) {
        get(); // consume opening quote '"'
        out.clear();
        while (m_pos < m_src.size()) {
            char c = get();
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (m_pos >= m_src.size()) {
                    setError(error_msg, "Unfinished escape sequence");
                    return false;
                }
                char esc = get();
                switch (esc) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    case 'u': {
                        if (m_pos + 4 > m_src.size()) {
                            setError(error_msg, "Incomplete unicode escape");
                            return false;
                        }
                        uint32_t cp = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = get();
                            cp <<= 4;
                            if (h >= '0' && h <= '9') cp |= (h - '0');
                            else if (h >= 'a' && h <= 'f') cp |= (h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') cp |= (h - 'A' + 10);
                            else {
                                setError(error_msg, "Invalid hex in unicode escape");
                                return false;
                            }
                        }
                        if (cp < 0x80) {
                            out.push_back(static_cast<char>(cp));
                        } else if (cp < 0x800) {
                            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        } else {
                            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        }
                        break;
                    }
                    default:
                        out.push_back(esc);
                        break;
                }
            } else {
                out.push_back(c);
            }
        }
        setError(error_msg, "Unterminated string");
        return false;
    }

    bool parseBool(JsonValue& out, std::string* error_msg) {
        if (m_src.substr(m_pos, 4) == "true") {
            m_pos += 4;
            out = JsonValue(true);
            return true;
        }
        if (m_src.substr(m_pos, 5) == "false") {
            m_pos += 5;
            out = JsonValue(false);
            return true;
        }
        setError(error_msg, "Invalid boolean literal");
        return false;
    }

    bool parseNull(JsonValue& out, std::string* error_msg) {
        if (m_src.substr(m_pos, 4) == "null") {
            m_pos += 4;
            out = JsonValue(nullptr);
            return true;
        }
        setError(error_msg, "Invalid null literal");
        return false;
    }

    bool parseNumber(JsonValue& out, std::string* error_msg) {
        size_t start = m_pos;
        if (peek() == '-') get();
        bool has_digits = false;
        while (std::isdigit(static_cast<unsigned char>(peek()))) {
            get();
            has_digits = true;
        }
        if (!has_digits) {
            setError(error_msg, "Invalid number format");
            return false;
        }
        bool is_float = false;
        if (peek() == '.') {
            is_float = true;
            get();
            while (std::isdigit(static_cast<unsigned char>(peek()))) {
                get();
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            is_float = true;
            get();
            if (peek() == '+' || peek() == '-') get();
            while (std::isdigit(static_cast<unsigned char>(peek()))) {
                get();
            }
        }

        std::string num_str(m_src.substr(start, m_pos - start));
        try {
            if (!is_float) {
                int64_t val = std::stoll(num_str);
                out = JsonValue(val);
                return true;
            } else {
                double val = std::stod(num_str);
                out = JsonValue(val);
                return true;
            }
        } catch (...) {
            // Fallback
            try {
                double val = std::stod(num_str);
                out = JsonValue(val);
                return true;
            } catch (...) {
                setError(error_msg, "Failed to parse number: " + num_str);
                return false;
            }
        }
    }
};

} // namespace

std::string JsonValue::serialize(bool pretty, int indent) const {
    std::string out;
    serializeImpl(*this, out, pretty, indent);
    return out;
}

JsonValue JsonValue::parse(std::string_view json, std::string* error_msg) {
    JsonValue out;
    JsonParser parser(json);
    if (!parser.parse(out, error_msg)) {
        return JsonValue();
    }
    return out;
}

bool JsonValue::tryParse(std::string_view json, JsonValue& out_val, std::string* error_msg) {
    JsonParser parser(json);
    return parser.parse(out_val, error_msg);
}

} // namespace motion_core::protocol
