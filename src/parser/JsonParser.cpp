#include "parser/JsonParser.h"
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <string>

// 读取JSON文件内容
json JsonParser::readJsonFile(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开JSON文件：" + filePath);
    }

    json jsonData;
    try {
        file >> jsonData;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON解析失败：" + std::string(e.what()));
    }

    file.close();
    return jsonData;
}

// 解析单个节点
Node JsonParser::parseNode(const json& nodeJson) {
    Node node;
    // 从JSON中提取节点核心字段（需与你的JSON文件结构匹配，若字段名不同需修改）
    node.id = nodeJson.value("id", -1);          // 节点ID，默认-1（无效）
    node.type = nodeJson.value("type", -1);      // 节点类型，默认-1（无效）
    node.x = nodeJson.value("x", 0.0);           // X坐标，默认0.0
    node.y = nodeJson.value("y", 0.0);           // Y坐标，默认0.0
    node.allowPass = nodeJson.value("allowPass", 1); // 是否允许通行（0/1）
    node.allowRot = nodeJson.value("allowRot", 1);   // 是否允许转向（0/1）

    auto parseTypeList = [&](const char* keyLegacy, const char* keyNew, std::vector<std::string>& out){
        if (nodeJson.contains(keyNew) && nodeJson[keyNew].is_array()) {
            for (const auto& v : nodeJson[keyNew]) {
                if (v.is_string()) out.push_back(v.get<std::string>());
                else if (v.is_number_integer()) out.push_back(std::to_string(v.get<int>()));
                else out.push_back(v.dump());
            }
            return;
        }
        if (nodeJson.contains(keyLegacy) && nodeJson[keyLegacy].is_array()) {
            for (const auto& v : nodeJson[keyLegacy]) {
                if (v.is_string()) out.push_back(v.get<std::string>());
                else if (v.is_number_integer()) out.push_back(std::to_string(v.get<int>()));
                else out.push_back(v.dump());
            }
        }
    };

    // 支持新旧两个字段名：allowPassAmr / allowPassAmrTypeList；allowRotAmr / allowRotAmrTypeList
    parseTypeList("allowPassAmr", "allowPassAmrTypeList", node.allowPassAmrTypeList);
    parseTypeList("allowRotAmr", "allowRotAmrTypeList", node.allowRotAmrTypeList);

    return node;
}

// 解析单个边
Edge JsonParser::parseEdge(const json& edgeJson) {
    Edge edge;
    // 从JSON中提取边核心字段（需与你的JSON文件结构匹配）
    edge.startNodeId = edgeJson.value("startNode", -1); // 起点ID，默认-1
    edge.endNodeId = edgeJson.value("endNode", -1);     // 终点ID，默认-1
    edge.weight = edgeJson.value("weight", 0.0);        // 边权重（距离），默认0.0
    edge.mode = edgeJson.value("mode", 0);              // 边模式，默认0（跳过）
    edge.direction = 1;
    edge.maxSpeed = edgeJson.value("maxSpeed", 0.0);    // 边限速（mm/s，0表示未指定）
    return edge;
}

// 解析单个区域
Area JsonParser::parseArea(const json& areaJson) {
    Area area;
    // 从JSON中提取区域核心字段（需与你的JSON文件结构匹配）
    area.minX = areaJson.value("minX", 0.0);
    area.minY = areaJson.value("minY", 0.0);
    double width = areaJson.value("width", 0.0);
    double height = areaJson.value("height", 0.0);
    area.maxX = area.minX + width; // 计算区域最大X坐标
    area.maxY = area.minY + height; // 计算区域最大Y坐标
    area.maxSpeed = areaJson.value("maxSpeed", 0.0); // 区域最大速度

    return area;
}

// 核心解析函数：读取JSON文件并生成节点、边、区域列表
void JsonParser::parseMapFile(
    const std::string& filePath,
    std::vector<Node>& nodes,
    std::vector<Edge>& edges,
    std::vector<Area>& areas,
    double& globalMaxSpeed
) {
    json jsonData = readJsonFile(filePath);
    parseMapDocument(jsonData, nodes, edges, areas, globalMaxSpeed);
}

void JsonParser::parseMapDocument(
    const json& jsonData,
    std::vector<Node>& nodes,
    std::vector<Edge>& edges,
    std::vector<Area>& areas,
    double& globalMaxSpeed
) {
    if (jsonData.contains("info") && jsonData["info"].contains("maxSpeed")) {
        globalMaxSpeed = jsonData["info"]["maxSpeed"];
    } else {
        throw std::runtime_error("JSON文件中未找到全局最大速度字段（info.maxSpeed）");
    }

    // 3. 解析节点列表（对应MATLAB的map_data.node）
    if (jsonData.contains("node") && jsonData["node"].is_array()) {
        for (const auto& nodeJson : jsonData["node"]) {
            Node node = parseNode(nodeJson);
            nodes.push_back(node);
        }
    } else {
        throw std::runtime_error("JSON文件中未找到节点列表字段（node）");
    }

    // 4. 解析边列表（对应MATLAB的map_data.edge）
    if (jsonData.contains("edge") && jsonData["edge"].is_array()) {
        for (const auto& edgeJson : jsonData["edge"]) {
            Edge edge = parseEdge(edgeJson);
            edges.push_back(edge);
        }
    } else {
        std::cerr << "警告：JSON文件中未找到边列表字段（edge），边列表为空" << std::endl;
    }

    // 5. 解析区域列表（对应MATLAB的map_data.area）
    if (jsonData.contains("area") && jsonData["area"].is_array()) {
        for (const auto& areaJson : jsonData["area"]) {
            Area area = parseArea(areaJson);
            areas.push_back(area);
        }
    } else {
        std::cerr << "警告：JSON文件中未找到区域列表字段（area），区域列表为空" << std::endl;
    }

    if (const char* v = std::getenv("RECEIVER_LOG_STARTUP_SUMMARY")) {
        std::string s(v);
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        if (s == "1" || s == "true" || s == "yes" || s == "on") {
            std::cout << "JSON解析完成：节点数=" << nodes.size() << "，边数=" << edges.size() << "，区域数=" << areas.size() << std::endl;
        }
    }
}
