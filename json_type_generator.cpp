// Input : line 1 = T; then per case: line A = root name, line B = compact JSON.
// Output: per-case blocks joined by a line `---`, ending with one '\n'.

#include <bits/stdc++.h>
using namespace std;

struct JsonValue;

enum class JsonType { Null, Bool, Number, String, Array, Object };

struct JsonValue {
    JsonType type = JsonType::Null;
    bool b = false;
    string numStr;
    string str;
    vector<JsonValue> arr;
    map<string, JsonValue> obj;
};

struct JsonParser {
    string s;
    size_t i = 0;

    void skipWs() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;
    }

    bool match(const string& lit) {
        if (s.compare(i, lit.size(), lit) != 0) return false;
        i += lit.size();
        return true;
    }

    JsonValue parseValue() {
        skipWs();
        if (match("null")) return JsonValue{};
        if (match("true")) { JsonValue v; v.type = JsonType::Bool; v.b = true; return v; }
        if (match("false")) { JsonValue v; v.type = JsonType::Bool; v.b = false; return v; }
        if (s[i] == '"') return parseString();
        if (s[i] == '[') return parseArray();
        if (s[i] == '{') return parseObject();
        return parseNumber();
    }

    JsonValue parseString() {
        JsonValue v;
        v.type = JsonType::String;
        i++;
        while (i < s.size()) {
            char c = s[i++];
            if (c == '"') break;
            if (c == '\\') {
                if (i >= s.size()) break;
                char e = s[i++];
                switch (e) {
                    case '"': v.str += '"'; break;
                    case '\\': v.str += '\\'; break;
                    case '/': v.str += '/'; break;
                    case 'b': v.str += '\b'; break;
                    case 'f': v.str += '\f'; break;
                    case 'n': v.str += '\n'; break;
                    case 'r': v.str += '\r'; break;
                    case 't': v.str += '\t'; break;
                    case 'u': {
                        if (i + 4 <= s.size()) {
                            i += 4; // skip unicode escapes
                        }
                        break;
                    }
                    default: v.str += e; break;
                }
            } else {
                v.str += c;
            }
        }
        return v;
    }

    JsonValue parseNumber() {
        JsonValue v;
        v.type = JsonType::Number;
        size_t start = i;
        if (s[i] == '-') i++;
        while (i < s.size() && isdigit(static_cast<unsigned char>(s[i]))) i++;
        if (i < s.size() && s[i] == '.') {
            i++;
            while (i < s.size() && isdigit(static_cast<unsigned char>(s[i]))) i++;
        }
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            i++;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) i++;
            while (i < s.size() && isdigit(static_cast<unsigned char>(s[i]))) i++;
        }
        v.numStr = s.substr(start, i - start);
        return v;
    }

    JsonValue parseArray() {
        JsonValue v;
        v.type = JsonType::Array;
        i++;
        skipWs();
        if (i < s.size() && s[i] == ']') { i++; return v; }
        while (true) {
            v.arr.push_back(parseValue());
            skipWs();
            if (i < s.size() && s[i] == ']') { i++; break; }
            if (i < s.size() && s[i] == ',') i++;
        }
        return v;
    }

    JsonValue parseObject() {
        JsonValue v;
        v.type = JsonType::Object;
        i++;
        skipWs();
        if (i < s.size() && s[i] == '}') { i++; return v; }
        while (true) {
            skipWs();
            JsonValue keyVal = parseString();
            skipWs();
            if (i < s.size() && s[i] == ':') i++;
            JsonValue val = parseValue();
            v.obj[keyVal.str] = std::move(val);
            skipWs();
            if (i < s.size() && s[i] == '}') { i++; break; }
            if (i < s.size() && s[i] == ',') i++;
        }
        return v;
    }

    JsonValue parse() {
        skipWs();
        return parseValue();
    }
};

struct TypeNode {
    bool hasString = false;
    bool hasNumber = false;
    bool hasBool = false;
    bool hasNull = false;
    bool hasArray = false;
    bool hasObject = false;
    int presentCount = 0;
    int objectCount = 0;
    unique_ptr<TypeNode> arrayElem;
    map<string, TypeNode> children;
};

static void mergeValue(TypeNode& node, const JsonValue& val) {
    switch (val.type) {
        case JsonType::String:
            node.hasString = true;
            break;
        case JsonType::Number:
            node.hasNumber = true;
            break;
        case JsonType::Bool:
            node.hasBool = true;
            break;
        case JsonType::Null:
            node.hasNull = true;
            break;
        case JsonType::Object:
            node.hasObject = true;
            node.objectCount++;
            for (const auto& [k, v] : val.obj) {
                node.children[k].presentCount++;
                mergeValue(node.children[k], v);
            }
            break;
        case JsonType::Array:
            node.hasArray = true;
            if (!node.arrayElem) node.arrayElem = make_unique<TypeNode>();
            for (const auto& elem : val.arr) {
                mergeValue(*node.arrayElem, elem);
            }
            break;
    }
}

static void mergeObject(TypeNode& root, const JsonValue& obj) {
    for (const auto& [k, v] : obj.obj) {
        root.children[k].presentCount++;
        mergeValue(root.children[k], v);
    }
}

static string capitalizeKey(const string& key) {
    if (key.empty()) return key;
    string r = key;
    r[0] = static_cast<char>(toupper(static_cast<unsigned char>(r[0])));
    return r;
}

struct TypeGenerator {
    string rootName;
    TypeNode root;
    int rootObjectCount = 0;
    set<string> usedNames;
    map<TypeNode*, string> nodeNames;

    explicit TypeGenerator(string rn) : rootName(std::move(rn)) {}

    string allocateName(const string& base) {
        if (!usedNames.count(base)) {
            usedNames.insert(base);
            return base;
        }
        for (int suffix = 2; ; suffix++) {
            string candidate = base + to_string(suffix);
            if (!usedNames.count(candidate)) {
                usedNames.insert(candidate);
                return candidate;
            }
        }
    }

    void assignNames(TypeNode& node, const string& name) {
        nodeNames[&node] = name;
        for (auto& [key, child] : node.children) {
            if (child.hasObject) {
                string nm = allocateName(capitalizeKey(key));
                assignNames(child, nm);
            } else if (child.hasArray && child.arrayElem && child.arrayElem->hasObject) {
                string nm = allocateName(capitalizeKey(key));
                assignNames(*child.arrayElem, nm);
            }
        }
    }

    vector<string> collectTypeStrings(const TypeNode& node) const {
        vector<string> parts;
        if (node.hasBool) parts.push_back("boolean");
        if (node.hasNull) parts.push_back("null");
        if (node.hasNumber) parts.push_back("number");
        if (node.hasString) parts.push_back("string");
        if (node.hasObject) {
            auto it = nodeNames.find(const_cast<TypeNode*>(&node));
            if (it != nodeNames.end()) parts.push_back(it->second);
        }
        return parts;
    }

    string arrayTypeString(const TypeNode& elem) const {
        vector<string> parts = collectTypeStrings(elem);
        if (parts.empty()) return "unknown[]";
        sort(parts.begin(), parts.end());
        if (parts.size() == 1) return parts[0] + "[]";
        return "(" + joinUnion(parts) + ")[]";
    }

    static string joinUnion(const vector<string>& parts) {
        string r;
        for (size_t i = 0; i < parts.size(); i++) {
            if (i) r += " | ";
            r += parts[i];
        }
        return r;
    }

    string fieldTypeString(const TypeNode& node) const {
        vector<string> parts = collectTypeStrings(node);
        if (node.hasArray) {
            parts.push_back(arrayTypeString(*node.arrayElem));
        }
        if (parts.empty()) return "unknown";
        sort(parts.begin(), parts.end());
        if (parts.size() == 1) return parts[0];
        return joinUnion(parts);
    }

    string formatInterface(const string& name, const TypeNode& node, int levelObjectCount) const {
        if (node.children.empty()) {
            return "export interface " + name + " {}";
        }
        string out = "export interface " + name + " {\n";
        for (const auto& [key, child] : node.children) {
            out += "  " + key;
            if (child.presentCount < levelObjectCount) out += "?";
            out += ": " + fieldTypeString(child) + ";\n";
        }
        out += "}";
        return out;
    }

    string generate() {
        usedNames.insert(rootName);
        assignNames(root, rootName);

        vector<pair<string, const TypeNode*>> interfaces;
        interfaces.emplace_back(rootName, &root);
        for (const auto& [ptr, name] : nodeNames) {
            if (ptr != &root) {
                interfaces.emplace_back(name, ptr);
            }
        }
        sort(interfaces.begin(), interfaces.end(),
             [](const auto& a, const auto& b) { return a.first < b.first; });

        string out;
        for (size_t idx = 0; idx < interfaces.size(); idx++) {
            if (idx) out += "\n\n";
            const string& name = interfaces[idx].first;
            const TypeNode* node = interfaces[idx].second;
            int levelCount = (node == &root) ? rootObjectCount : node->objectCount;
            out += formatInterface(name, *node, levelCount);
        }
        return out;
    }
};

static string solve(const string& rootName, const string& jsonText) {
    // TODO: implement.
    JsonParser parser{jsonText};
    JsonValue parsed = parser.parse();

    TypeGenerator gen(rootName);

    if (parsed.type == JsonType::Array) {
        gen.rootObjectCount = static_cast<int>(parsed.arr.size());
        for (const auto& elem : parsed.arr) {
            if (elem.type == JsonType::Object) {
                mergeObject(gen.root, elem);
            }
        }
    }

    return gen.generate();
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int t;
    cin >> t;
    cin.ignore();

    string out;
    for (int i = 0; i < t; i++) {
        string rootName, jsonText;
        getline(cin, rootName);
        getline(cin, jsonText);
        if (i > 0) out += "\n---\n";
        out += solve(rootName, jsonText);
    }
    out += '\n';

    cout << out;
    return 0;
}
