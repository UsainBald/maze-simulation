#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <limits>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <chrono>

using namespace std;

class Maze {
public:
    vector<vector<int>> grid;
    vector<pair<int, int>> path; // Store the path here
    pair<int, int> start;
    pair<int, int> end;
    int rows, cols;
    Maze(int r, int c) : rows(r), cols(c) {
        grid = vector<vector<int>>(rows, vector<int>(cols, 0));
    }

    void setWall(int r, int c) {
        grid[r][c] = 1;
    }

    void setStart(int r, int c) {
        start = {r, c};
    }

    void setEnd(int r, int c) {
        end = {r, c};
    }

    void printMaze() {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (make_pair(i, j) == start) cout << "S ";
                else if (make_pair(i, j) == end) cout << "E ";
                else if (find(path.begin(), path.end(), make_pair(i, j)) != path.end()) cout << "O "; // Mark the path
                else if (grid[i][j] == 1) cout << "# ";
                else cout << ". ";
            }
            cout << endl;
        }
    }

    void generateRandomMaze() {
        srand(time(0));
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                grid[i][j] = (rand() % 100 < 25) ? 1 : 0; // 30% chance to generate a wall
            }
        }
        grid[start.first][start.second] = 0;
        grid[end.first][end.second] = 0;
    }
};

class PathFinding {
    Maze &maze;

    struct Node {
        int r, c, cost, heuristic;
        Node(int row, int col, int c, int h) : r(row), c(col), cost(c), heuristic(h) {}
        bool operator<(const Node &other) const {
            return cost + heuristic > other.cost + other.heuristic;
        }
    };

public:
    PathFinding(Maze &m) : maze(m) {}

    vector<pair<int, int>> getNeighbors(int r, int c) {
        vector<pair<int, int>> neighbors;
        if (r > 0 && maze.grid[r - 1][c] == 0) neighbors.push_back({r - 1, c});
        if (r < maze.rows - 1 && maze.grid[r + 1][c] == 0) neighbors.push_back({r + 1, c});
        if (c > 0 && maze.grid[r][c - 1] == 0) neighbors.push_back({r, c - 1});
        if (c < maze.cols - 1 && maze.grid[r][c + 1] == 0) neighbors.push_back({r, c + 1});
        return neighbors;
    }

    vector<pair<int, int>> aStar() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        priority_queue<Node> pq;
        vector<vector<int>> cost(maze.rows, vector<int>(maze.cols, INT_MAX));
        vector<vector<pair<int, int>>> parent(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        pq.push(Node(maze.start.first, maze.start.second, 0, heuristic(maze.start.first, maze.start.second)));
        cost[maze.start.first][maze.start.second] = 0;

        while (!pq.empty()) {
            nodeCount++;
            Node current = pq.top();
            pq.pop();
            if (current.r == maze.end.first && current.c == maze.end.second) {
                auto endTime = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                cout << "A* Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                int newCost = cost[current.r][current.c] + 1;
                if (newCost < cost[neighbor.first][neighbor.second]) {
                    cost[neighbor.first][neighbor.second] = newCost;
                    pq.push(Node(neighbor.first, neighbor.second, newCost, heuristic(neighbor.first, neighbor.second)));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> dijkstra() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        priority_queue<Node> pq;
        vector<vector<int>> cost(maze.rows, vector<int>(maze.cols, INT_MAX));
        vector<vector<pair<int, int>>> parent(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        pq.push(Node(maze.start.first, maze.start.second, 0, 0));
        cost[maze.start.first][maze.start.second] = 0;

        while (!pq.empty()) {
            nodeCount++;
            Node current = pq.top();
            pq.pop();
            if (current.r == maze.end.first && current.c == maze.end.second) {
                auto endTime = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                cout << "Dijkstra's Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                int newCost = cost[current.r][current.c] + 1;
                if (newCost < cost[neighbor.first][neighbor.second]) {
                    cost[neighbor.first][neighbor.second] = newCost;
                    pq.push(Node(neighbor.first, neighbor.second, newCost, 0));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> bfs() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        queue<Node> q;
        vector<vector<bool>> visited(maze.rows, vector<bool>(maze.cols, false));
        vector<vector<pair<int, int>>> parent(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        q.push(Node(maze.start.first, maze.start.second, 0, 0));
        visited[maze.start.first][maze.start.second] = true;

        while (!q.empty()) {
            nodeCount++;
            Node current = q.front();
            q.pop();
            if (current.r == maze.end.first && current.c == maze.end.second) {
                auto endTime = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                cout << "BFS Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                if (!visited[neighbor.first][neighbor.second]) {
                    visited[neighbor.first][neighbor.second] = true;
                    q.push(Node(neighbor.first, neighbor.second, 0, 0));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> dfs() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        stack<Node> s;
        vector<vector<bool>> visited(maze.rows, vector<bool>(maze.cols, false));
        vector<vector<pair<int, int>>> parent(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        s.push(Node(maze.start.first, maze.start.second, 0, 0));
        visited[maze.start.first][maze.start.second] = true;

        while (!s.empty()) {
            nodeCount++;
            Node current = s.top();
            s.pop();
            if (current.r == maze.end.first && current.c == maze.end.second) {
                auto endTime = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                cout << "DFS Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                if (!visited[neighbor.first][neighbor.second]) {
                    visited[neighbor.first][neighbor.second] = true;
                    s.push(Node(neighbor.first, neighbor.second, 0, 0));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> bidirectionalBFS() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        queue<Node> qStart, qEnd;
        vector<vector<bool>> visitedStart(maze.rows, vector<bool>(maze.cols, false));
        vector<vector<bool>> visitedEnd(maze.rows, vector<bool>(maze.cols, false));
        vector<vector<pair<int, int>>> parentStart(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        vector<vector<pair<int, int>>> parentEnd(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));

        qStart.push(Node(maze.start.first, maze.start.second, 0, 0));
        qEnd.push(Node(maze.end.first, maze.end.second, 0, 0));
        visitedStart[maze.start.first][maze.start.second] = true;
        visitedEnd[maze.end.first][maze.end.second] = true;

        while (!qStart.empty() && !qEnd.empty()) {
            if (!qStart.empty()) {
                nodeCount++;
                Node currentStart = qStart.front();
                qStart.pop();
                if (visitedEnd[currentStart.r][currentStart.c]) {
                    auto endTime = chrono::high_resolution_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                    cout << "Bidirectional BFS Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                    return reconstructBidirectionalPath(parentStart, parentEnd, {currentStart.r, currentStart.c});
                }
                for (auto &neighbor : getNeighbors(currentStart.r, currentStart.c)) {
                    if (!visitedStart[neighbor.first][neighbor.second]) {
                        visitedStart[neighbor.first][neighbor.second] = true;
                        qStart.push(Node(neighbor.first, neighbor.second, 0, 0));
                        parentStart[neighbor.first][neighbor.second] = {currentStart.r, currentStart.c};
                    }
                }
            }

            if (!qEnd.empty()) {
                nodeCount++;
                Node currentEnd = qEnd.front();
                qEnd.pop();
                if (visitedStart[currentEnd.r][currentEnd.c]) {
                    auto endTime = chrono::high_resolution_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                    cout << "Bidirectional BFS Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                    return reconstructBidirectionalPath(parentStart, parentEnd, {currentEnd.r, currentEnd.c});
                }
                for (auto &neighbor : getNeighbors(currentEnd.r, currentEnd.c)) {
                    if (!visitedEnd[neighbor.first][neighbor.second]) {
                        visitedEnd[neighbor.first][neighbor.second] = true;
                        qEnd.push(Node(neighbor.first, neighbor.second, 0, 0));
                        parentEnd[neighbor.first][neighbor.second] = {currentEnd.r, currentEnd.c};
                    }
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> bidirectionalDijkstra() {
        auto startTime = chrono::high_resolution_clock::now();
        int nodeCount = 0;

        priority_queue<Node> pqStart, pqEnd;
        vector<vector<int>> costStart(maze.rows, vector<int>(maze.cols, INT_MAX));
        vector<vector<int>> costEnd(maze.rows, vector<int>(maze.cols, INT_MAX));
        vector<vector<pair<int, int>>> parentStart(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));
        vector<vector<pair<int, int>>> parentEnd(maze.rows, vector<pair<int, int>>(maze.cols, {-1, -1}));

        pqStart.push(Node(maze.start.first, maze.start.second, 0, 0));
        pqEnd.push(Node(maze.end.first, maze.end.second, 0, 0));
        costStart[maze.start.first][maze.start.second] = 0;
        costEnd[maze.end.first][maze.end.second] = 0;

        while (!pqStart.empty() && !pqEnd.empty()) {
            if (!pqStart.empty()) {
                nodeCount++;
                Node currentStart = pqStart.top();
                pqStart.pop();
                if (costEnd[currentStart.r][currentStart.c] < INT_MAX) {
                    auto endTime = chrono::high_resolution_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                    cout << "Bidirectional Dijkstra Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                    return reconstructBidirectionalPath(parentStart, parentEnd, {currentStart.r, currentStart.c});
                }
                for (auto &neighbor : getNeighbors(currentStart.r, currentStart.c)) {
                    int newCost = costStart[currentStart.r][currentStart.c] + 1;
                    if (newCost < costStart[neighbor.first][neighbor.second]) {
                        costStart[neighbor.first][neighbor.second] = newCost;
                        pqStart.push(Node(neighbor.first, neighbor.second, newCost, 0));
                        parentStart[neighbor.first][neighbor.second] = {currentStart.r, currentStart.c};
                    }
                }
            }

            if (!pqEnd.empty()) {
                nodeCount++;
                Node currentEnd = pqEnd.top();
                pqEnd.pop();
                if (costStart[currentEnd.r][currentEnd.c] < INT_MAX) {
                    auto endTime = chrono::high_resolution_clock::now();
                    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
                    cout << "Bidirectional Dijkstra Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                    return reconstructBidirectionalPath(parentStart, parentEnd, {currentEnd.r, currentEnd.c});
                }
                for (auto &neighbor : getNeighbors(currentEnd.r, currentEnd.c)) {
                    int newCost = costEnd[currentEnd.r][currentEnd.c] + 1;
                    if (newCost < costEnd[neighbor.first][neighbor.second]) {
                        costEnd[neighbor.first][neighbor.second] = newCost;
                        pqEnd.push(Node(neighbor.first, neighbor.second, newCost, 0));
                        parentEnd[neighbor.first][neighbor.second] = {currentEnd.r, currentEnd.c};
                    }
                }
            }
        }
        return {};
    }


private:
    int heuristic(int r, int c) {
        return abs(r - maze.end.first) + abs(c - maze.end.second);
    }

    vector<pair<int, int>> reconstructPath(vector<vector<pair<int, int>>> &parent) {
        vector<pair<int, int>> path;
        for (pair<int, int> at = maze.end; at != make_pair(-1, -1); at = parent[at.first][at.second]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    vector<pair<int, int>> reconstructBidirectionalPath(vector<vector<pair<int, int>>> &parentStart, vector<vector<pair<int, int>>> &parentEnd, pair<int, int> meetPoint) {
        vector<pair<int, int>> path;
        for (pair<int, int> at = meetPoint; at != make_pair(-1, -1); at = parentStart[at.first][at.second]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        for (pair<int, int> at = parentEnd[meetPoint.first][meetPoint.second]; at != make_pair(-1, -1); at = parentEnd[at.first][at.second]) {
            path.push_back(at);
        }
        return path;
    }
};

void displayAlgorithmDetails() {
    cout << "Choose the algorithm:\n";
    cout << "1. A* (A Star)\n   - A* is a best-first search algorithm that finds the shortest path from start to end. It uses a heuristic to estimate the distance to the goal.\n";
    cout << "2. Dijkstra\n   - Dijkstra's algorithm is a pathfinding algorithm that computes the shortest path from the starting node to all other nodes in the graph.\n";
    cout << "3. BFS (Breadth-First Search)\n   - BFS explores the maze level by level, finding the shortest path in an unweighted maze.\n";
    cout << "4. DFS (Depth-First Search)\n   - DFS explores as far as possible along each branch before backtracking, making it less memory intensive but not guaranteed to find the shortest path.\n";
    cout << "5. Bidirectional BFS\n   - This approach uses two simultaneous BFS searches from both the start and the end, meeting in the middle. It's efficient for finding the shortest path when both endpoints are known.\n";
    cout << "6. Bidirectional Dijkstra\n   - Similar to the bidirectional BFS but with pathfinding that considers costs, making it suitable for weighted graphs.\n\n";
}

int main() {
    int rows, cols, startRow, startCol, endRow, endCol, walls;
    cout << "Enter the number of rows and columns: ";
    cin >> rows >> cols;
    Maze maze(rows, cols);

    cout << "Enter the start position (row and column): ";
    cin >> startRow >> startCol;
    maze.setStart(startRow, startCol);

    cout << "Enter the end position (row and column): ";
    cin >> endRow >> endCol;
    maze.setEnd(endRow, endCol);

    int choice;
    cout << "Choose an option:\n1. Manual input\n2. Generate random maze\n";
    cin >> choice;

    if (choice == 1) {
        cout << "Enter the number of walls: ";
        cin >> walls;
        cout << "Enter the positions of walls (row and column): " << endl;
        for (int i = 0; i < walls; ++i) {
            int r, c;
            cin >> r >> c;
            maze.setWall(r, c);
        }
    } else if (choice == 2) {
        maze.generateRandomMaze();
    }

    maze.printMaze();

    PathFinding pf(maze);

    displayAlgorithmDetails();
    cin >> choice;

    vector<pair<int, int>> path;
    switch (choice) {
        case 1:
            path = pf.aStar();
            break;
        case 2:
            path = pf.dijkstra();
            break;
        case 3:
            path = pf.bfs();
            break;
        case 4:
            path = pf.dfs();
            break;
        case 5:
            path = pf.bidirectionalBFS();
            break;
        case 6:
            path = pf.bidirectionalDijkstra();
            break;
        default:
            cout << "Invalid choice!" << endl;
            return 1;
    }

    maze.path = path;
    maze.printMaze();

    if (path.empty()) {
        cout << "No path found!" << endl;
    } else {
        cout << "Path found: ";
        for (auto &p : path) {
            cout << "(" << p.first << ", " << p.second << ") ";
        }
        cout << endl;
    }

    return 0;
}
