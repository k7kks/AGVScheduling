// agv_cluster_scheduling/include/common/RabbitMQConfig.h
#ifndef RABBITMQ_CONFIG_H
#define RABBITMQ_CONFIG_H

#include <string>
#include <cstdlib>
#include <algorithm>

struct RabbitMQConfig {
    std::string host = "127.0.0.1";
    int port = 5672;
    std::string username = "guest";
    std::string password = "guest";
    // 虚拟主机（AMQP 地址的一部分），默认使用根vhost
    std::string vhost = "/";
    // 业务 <-> 算法 MQ 资源（默认与外部对接规范保持一致）
    std::string dispToAlgoExchange = "DispToAlgoExchange";
    std::string dispToAlgoQueue = "DispToAlgoQueue";
    std::string dispToAlgoRoutingKey = "AssignmentTaskRequest";
    std::string algoToDispExchange = "AlgoToDispExchange";
    std::string algoToDispQueue = "AlgoToDispQueue";
    std::string dispToAlgoBindingKey = "#";
    std::string algoToDispRoutingKey = "AssignmentTaskResponse";
    std::string algoToDispBindingKey = "#";
    // 算法 -> 交通服务路径发布
    std::string algoToTrafficExchange = "AlgoToTrafficExchange";
    std::string algoToTrafficQueue = "AlgoToTrafficQueue";
    std::string algoToTrafficRoutingKey = "SendPathInfo";
    std::string algoToTrafficBindingKey = "#";
};

// 从环境变量覆盖默认连接参数
// 支持：AMQP_HOST, AMQP_PORT, AMQP_USER, AMQP_PASS, AMQP_VHOST
// 解析 RABBITMQ_URL（amqp://user:pass@host:port/vhost），若存在则覆盖 base
inline void RabbitMQConfigOverrideFromUrl(RabbitMQConfig& base) {
    const char* url = std::getenv("RABBITMQ_URL");
    if (!url || !*url) return;
    std::string s(url);
    // 去掉协议前缀
    auto pos = s.find("://");
    std::string rest = (pos != std::string::npos) ? s.substr(pos + 3) : s;
    // 拆分凭据与主机
    std::string cred, hostpart;
    auto at = rest.find('@');
    if (at != std::string::npos) { cred = rest.substr(0, at); hostpart = rest.substr(at + 1); }
    else { hostpart = rest; }
    if (!cred.empty()) {
        auto colon = cred.find(':');
        if (colon != std::string::npos) { base.username = cred.substr(0, colon); base.password = cred.substr(colon + 1); }
        else { base.username = cred; }
    }
    // host[:port][/vhost]
    std::string hostport = hostpart, vhost;
    auto slash = hostpart.find('/');
    if (slash != std::string::npos) { hostport = hostpart.substr(0, slash); vhost = hostpart.substr(slash + 1); }
    auto colon2 = hostport.find(':');
    if (colon2 != std::string::npos) { base.host = hostport.substr(0, colon2); try { base.port = std::stoi(hostport.substr(colon2 + 1)); } catch (...) {} }
    else if (!hostport.empty()) { base.host = hostport; }
    if (!vhost.empty()) {
        // 解码常见的 %2F → /
        std::string decoded; decoded.reserve(vhost.size());
        for (size_t i = 0; i < vhost.size(); ++i) {
            if (vhost[i] == '%' && i + 2 < vhost.size()) {
                std::string hex = vhost.substr(i + 1, 2);
                std::transform(hex.begin(), hex.end(), hex.begin(), ::toupper);
                if (hex == "2F") { decoded.push_back('/'); i += 2; continue; }
            }
            decoded.push_back(vhost[i]);
        }
        base.vhost = decoded;
    }
}

inline RabbitMQConfig RabbitMQConfigFromEnv(RabbitMQConfig base = {}) {
    if (const char* v = std::getenv("AMQP_HOST")) base.host = v;
    if (const char* v = std::getenv("AMQP_PORT")) {
        try { base.port = std::stoi(v); } catch (...) {}
    }
    if (const char* v = std::getenv("AMQP_USER")) base.username = v;
    if (const char* v = std::getenv("AMQP_PASS")) base.password = v;
    if (const char* v = std::getenv("AMQP_VHOST")) base.vhost = v;
    // 允许使用 RABBITMQ_URL 一次性覆盖（与 Node 端一致）
    RabbitMQConfigOverrideFromUrl(base);
    return base;
}

#endif // RABBITMQ_CONFIG_H
