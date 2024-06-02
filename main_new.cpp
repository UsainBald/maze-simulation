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
#include <set>

using namespace std;

class Maze {
public:
    vector<vector<int>> grid;
    vector<pair<int, int>> path;
    pair<int, int> start = {-1, -1};
    pair<int, int> end = {-1, -1};
    int rows, cols;
    bool isWeighted;
    vector<vector<int>> weights;

    Maze(int r, int c, bool weighted = false) : rows(r), cols(c), isWeighted(weighted) {
        grid = vector<vector<int>>(rows, vector<int>(cols, 0));
        if (isWeighted) {
            weights = vector<vector<int>>(rows, vector<int>(cols, 1));
        }
    }

    void setWall(int r, int c) {
        grid[r][c] = 1;
    }

    void setStart(int r, int c) {
        start = {r, c};
        grid[r][c] = 0;
    }

    void setEnd(int r, int c) {
        end = {r, c};
        grid[r][c] = 0;
    }

    void setWeight(int r, int c, int weight) {
        if (isWeighted) {
            weights[r][c] = weight;
        }
    }

    void printMaze() {
        set<pair<int, int>> path_set(path.begin(), path.end());
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (make_pair(i, j) == start) cout << "S ";
                else if (make_pair(i, j) == end) cout << "E ";
                else if (find(path.begin(), path.end(), make_pair(i, j)) != path.end()) cout << "O ";
                else if (grid[i][j] == 1) cout << "# ";
                else if (isWeighted) cout << weights[i][j] << " ";
                else cout << ". ";
            }
            cout << endl;
        }
    }

    void generateRandomMaze() {
        srand(time(0));
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                grid[i][j] = (rand() % 100 < 25) ? 1 : 0;
                if (isWeighted && grid[i][j] == 0) {
                    weights[i][j] = rand() % 9 + 1;  // random weights between 1 and 9
                }
            }
        }
        // grid[start.first][start.second] = 0;
        // grid[end.first][end.second] = 0;
    }

    void inputWeights() {
        if (!isWeighted) return;
        cout << "Enter weights for each cell (1-9):\n";
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (grid[i][j] == 0) {
                    cout << "Weight for cell (" << i << ", " << j << "): ";
                    cin >> weights[i][j];
                }
            }
        }
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

    int getCost(int r, int c) {
        return maze.isWeighted ? maze.weights[r][c] : 1;
    }

    vector<pair<int, int>> aStar() {
        return search(1, 1);
    }

    vector<pair<int, int>> dijkstra() {
        return search(1, 0);
    }

    vector<pair<int, int>> pureHeuristic() {
        return search(0, 1);
    }

    vector<pair<int, int>> weightedAStar() {
        return search(1, 10);
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
                cout << "Path cost: " << current.cost << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                if (!visited[neighbor.first][neighbor.second]) {
                    visited[neighbor.first][neighbor.second] = true;
                    q.push(Node(neighbor.first, neighbor.second, current.cost + getCost(neighbor.first, neighbor.second), 0));
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
                cout << "Path cost: " << current.cost << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                if (!visited[neighbor.first][neighbor.second]) {
                    visited[neighbor.first][neighbor.second] = true;
                    s.push(Node(neighbor.first, neighbor.second, current.cost + getCost(neighbor.first, neighbor.second), 0));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
                }
            }
        }
        return {};
    }

    vector<pair<int, int>> search(int w1, int w2) {
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
                cout << "Algorithm completed in: " << duration << " ms, Nodes visited: " << nodeCount << endl;
                cout << "Path cost: " << cost[current.r][current.c] << endl;
                return reconstructPath(parent);
            }
            for (auto &neighbor : getNeighbors(current.r, current.c)) {
                int newCost = cost[current.r][current.c] + getCost(neighbor.first, neighbor.second);
                if (newCost < cost[neighbor.first][neighbor.second]) {
                    cost[neighbor.first][neighbor.second] = newCost;
                    pq.push(Node(neighbor.first, neighbor.second, w1 * newCost, w2 * heuristic(neighbor.first, neighbor.second)));
                    parent[neighbor.first][neighbor.second] = {current.r, current.c};
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
        maze.path = path;
        return path;
    }
};


int main() {
    int rows, cols;
    cout << "Enter the number of rows: ";
    cin >> rows;
    cout << "Enter the number of columns: ";
    cin >> cols;

    bool isWeighted;
    cout << "Do you want to create a weighted maze? (1 for Yes, 0 for No): ";
    cin >> isWeighted;

    Maze maze(rows, cols, isWeighted);

    int choice;
    cout << "Choose maze generation method (1 for manual input, 2 for random generation): ";
    cin >> choice;

    if (choice == 1) {
        cout << "Enter maze grid (0 for empty, 1 for wall):\n";
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                cin >> maze.grid[i][j];
            }
        }
        if (isWeighted) {
            maze.inputWeights();
        }
    } else {
        maze.generateRandomMaze();
    }

    maze.printMaze();

    int startR, startC, endR, endC;
    cout << "Enter start cell (row and column): ";
    cin >> startR >> startC;
    maze.setStart(startR, startC);

    cout << "Enter end cell (row and column): ";
    cin >> endR >> endC;
    maze.setEnd(endR, endC);

    PathFinding pf(maze);

    while (true) {
        int algoChoice;
        cout << "Choose pathfinding algorithm:\n";
        cout << "1 - A* (g = 1, h = 1);\n";
        cout << "2 - Dijkstra (g = 1, h = 0);\n";
        cout << "3 - BFS;\n";
        cout << "4 - DFS;\n";
        cout << "5 - Greedy best-first search (g = 0; h = 1);\n";
        cout << "6 - Weighted A* (g = 1; h = 10);\n";
        cout << "7 - Custom heuristic(g, h - custom);\n";
        cout << "0 - Exit\n";
        cin >> algoChoice;

        if (algoChoice == 0) {
            cout << "Exiting program.\n";
            break;
        }

        vector<pair<int, int>> path;
        if (algoChoice == 1) {
            path = pf.aStar();
        } else if (algoChoice == 2) {
            path = pf.dijkstra();
        } else if (algoChoice == 3) {
            path = pf.bfs();
        } else if (algoChoice == 4) {
            path = pf.dfs();
        } else if (algoChoice == 5) {
            path = pf.pureHeuristic();
        } else if (algoChoice == 6) {
            path = pf.weightedAStar();
        } else if (algoChoice == 7) {
            int _g, _h;
            cout << "Enter g: ";
            cin >> _g;
            cout << "Enter h: ";
            cin >> _h;
            path = pf.search(_g, _h);
        }

        if (!path.empty()) {
            maze.printMaze();
        } else {
            cout << "No path found.\n";
        }
    }

    return 0;
}
