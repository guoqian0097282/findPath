#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <utility>
#include <algorithm>

using namespace std;

// 定义栅格地图中的节点
struct Node {
    int x, y;       // 节点的坐标
    int dist;       // 从起点到该节点的距离
    bool visited;   // 是否已访问
    Node* parent;   // 路径中的前驱节点
    
    Node(int x, int y) : x(x), y(y), dist(INT_MAX), visited(false), parent(nullptr) {}
};

// 自定义优先队列的比较函数
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->dist > b->dist;
    }
};

class DijkstraGrid {
private:
    vector<vector<Node*>> grid;  // 栅格地图
    int rows, cols;              // 地图的行数和列数
    
    // 检查坐标是否在地图范围内
    bool isValid(int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }
    
    // 获取相邻节点（4连通或8连通）
    vector<Node*> getNeighbors(Node* node, bool eightConnected = false) {
        vector<Node*> neighbors;
        int x = node->x;
        int y = node->y;
        
        // 4连通方向
        int dx4[] = {-1, 1, 0, 0};
        int dy4[] = {0, 0, -1, 1};
        
        // 8连通方向
        int dx8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        int dy8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        
        int* dx = dx4;
        int* dy = dy4;
        int count = 4;
        
        if (eightConnected) {
            dx = dx8;
            dy = dy8;
            count = 8;
        }
        
        for (int i = 0; i < count; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (isValid(nx, ny)) {
                neighbors.push_back(grid[nx][ny]);
            }
        }
        
        return neighbors;
    }
    
public:
    DijkstraGrid(int rows, int cols) : rows(rows), cols(cols) {
        // 初始化栅格地图
        grid.resize(rows, vector<Node*>(cols));
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                grid[i][j] = new Node(i, j);
            }
        }
    }
    
    ~DijkstraGrid() {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                delete grid[i][j];
            }
        }
    }
    
    // 执行Dijkstra算法
    void findPath(int startX, int startY, int goalX, int goalY, bool eightConnected = false) {
        if (!isValid(startX, startY) || !isValid(goalX, goalY)) {
            cout << "起点或终点不在有效范围内!" << endl;
            return;
        }
        
        // 重置所有节点状态
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                grid[i][j]->dist = INT_MAX;
                grid[i][j]->visited = false;
                grid[i][j]->parent = nullptr;
            }
        }
        
        // 设置起点
        Node* startNode = grid[startX][startY];
        startNode->dist = 0;
        
        // 使用优先队列（最小堆）
        priority_queue<Node*, vector<Node*>, CompareNode> pq;
        pq.push(startNode);
        
        while (!pq.empty()) {
            Node* current = pq.top();
            pq.pop();
            
            // 如果已经访问过，跳过
            if (current->visited) continue;
            
            current->visited = true;
            
            // 如果到达目标点，提前退出
            if (current->x == goalX && current->y == goalY) {
                break;
            }
            
            // 检查所有邻居
            vector<Node*> neighbors = getNeighbors(current, eightConnected);
            for (Node* neighbor : neighbors) {
                // 计算新的距离（假设相邻节点间的距离为1）
                int newDist = current->dist + 1;
                
                // 如果找到更短的路径
                if (newDist < neighbor->dist) {
                    neighbor->dist = newDist;
                    neighbor->parent = current;
                    pq.push(neighbor);
                }
            }
        }
        
        // 回溯路径
        Node* node = grid[goalX][goalY];
        if (node->dist == INT_MAX) {
            cout << "无法到达目标点!" << endl;
            return;
        }
        
        vector<pair<int, int>> path;
        while (node != nullptr) {
            path.emplace_back(node->x, node->y);
            node = node->parent;
        }
        reverse(path.begin(), path.end());
        
        // 打印路径
        cout << "最短路径长度: " << path.size() - 1 << endl;
        cout << "路径: ";
        for (auto p : path) {
            cout << "(" << p.first << "," << p.second << ") ";
        }
        cout << endl;
    }
    
    // 打印地图和路径
    void printMapWithPath(int startX, int startY, int goalX, int goalY) {
        vector<vector<char>> display(rows, vector<char>(cols, '.'));
        
        // 标记路径
        Node* node = grid[goalX][goalY];
        if (node->dist != INT_MAX) {
            while (node != nullptr) {
                display[node->x][node->y] = '@';
                node = node->parent;
            }
        }
        
        // 标记起点和终点
        display[startX][startY] = 'S';
        display[goalX][goalY] = 'G';
        
        // 打印地图
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                cout << display[i][j] << " ";
            }
            cout << endl;
        }
    }
};