#ifndef JSONPARSER_H
#define JSONPARSER_H

#include <string>
#include <vector>
#include "data/MapInfo.h"
#include "lib/nlohmann/json.hpp"

// 简化json命名空间
using json = nlohmann::json;

class JsonParser {
private:
    // 解析单个节点（从JSON对象转换为Node结构）
    Node parseNode(const json& nodeJson);
    // 解析单个边（从JSON对象转换为Edge结构）
    Edge parseEdge(const json& edgeJson);
    // 解析单个区域（从JSON对象转换为Area结构）
    Area parseArea(const json& areaJson);
    // 读取JSON文件内容并返回json对象
    json readJsonFile(const std::string& filePath);

public:
    // 构造函数（空实现）
    JsonParser() = default;
    // 析构函数（空实现）
    ~JsonParser() = default;

    // 核心解析函数：读取JSON地图文件，输出节点、边、区域列表和全局最大速度
    void parseMapFile(
        const std::string& filePath,
        std::vector<Node>& nodes,
        std::vector<Edge>& edges,
        std::vector<Area>& areas,
        double& globalMaxSpeed
    );

    void parseMapDocument(
        const json& jsonData,
        std::vector<Node>& nodes,
        std::vector<Edge>& edges,
        std::vector<Area>& areas,
        double& globalMaxSpeed
    );
};

#endif // JSONPARSER_H
