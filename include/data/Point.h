#ifndef POINT_H
#define POINT_H

// Point结构的定义
class Point {
private:
    int x;          // 坐标x,mm
    int y;          // 坐标y,mm
    int angle;      // 角度，度
    int nodeId;     // 节点Id
    int pointType;  // 点类型

public:
    Point(int x = 0, int y = 0, int angle = 0, int nodeId = -1, int pointType = 0)
        : x(x), y(y), angle(angle), nodeId(nodeId), pointType(pointType) {}

    // Getter方法
    int getX() const { return x; }
    int getY() const { return y; }
    int getAngle() const { return angle; }
    int getNodeId() const { return nodeId; }
    int getPointType() const { return pointType; }

    // Setter方法
    void setX(int x) { this->x = x; }
    void setY(int y) { this->y = y; }
    void setAngle(int angle) { this->angle = angle; }
    void setNodeId(int nodeId) { this->nodeId = nodeId; }
    void setPointType(int pointType) { this->pointType = pointType; }

    // 显示点信息
    void displayInfo() const {
        printf("  点坐标: (%d, %d), 角度: %d, 节点Id: %d, 点类型: %d\n", x, y, angle, nodeId, pointType);
    }
};
#endif // POINT_H