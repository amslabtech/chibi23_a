
#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <cmath>
#include <algorithm>


using namespace std;

// ノードの構造体
struct Node {
		int x, y;
		double f, g, h;

		Node(int x, int y, double g, double h) : x(x), y(y), g(g), h(h), f(g + h) {}
};

struct NodeComparator {
		bool operator()(const Node &n1, const Node &n2) {
				return n1.f > n2.f;
		}
};

vector<pair<int, int>> neighbors = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

bool isValid(int x, int y, int rows, int cols) {
		return x >= 0 && x < rows && y >= 0 && y < cols;
}

double heuristic(int x1, int y1, int x2, int y2) {
		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

vector<pair<int, int>> a_star(vector<vector<int>> &grid, pair<int, int> start, pair<int, int> goal) {
		int rows = grid.size(), cols = grid[0].size();

		printf("rows=%d, cols=%d\n", rows, cols); // デバッグ用

		vector<vector<bool>> visited(rows, vector<bool>(cols, false));
		vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-1, -1}));

		printf("came_from=%d, %d\n", came_from[0][0].first, came_from[0][0].second); // デバッグ用

		priority_queue<Node, vector<Node>, NodeComparator> open_set;
		open_set.push(Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second)));

		while (!open_set.empty()) {
			printf("open_set=%d, %d, %f, %f, %f\n", open_set.top().x, open_set.top().y, open_set.top().f, open_set.top().g, open_set.top().h); // デバッグ用

				Node current = open_set.top(); // 最後の要素を取り出す
				open_set.pop(); // 取り出した要素を削除する
				int x = current.x, y = current.y;

				printf("x=%d, y=%d\n", x, y); // デバッグ用

				if (x == goal.first && y == goal.second) {
						vector<pair<int, int>> path;
						while (x != start.first || y != start.second) {
								path.push_back({x, y});
								tie(x, y) = came_from[x][y];
						}
						path.push_back({x, y});
						reverse(path.begin(), path.end());
						return path;
				}

				if (visited[x][y]) continue;
				visited[x][y] = true;

				for (auto &neighbor : neighbors) {
						int nx = x + neighbor.first, ny = y + neighbor.second;

						if (isValid(nx, ny, rows, cols) && grid[nx][ny] == 0 && !visited[nx][ny]) {
								double tentative_g = current.g + 1;
								double h = heuristic(nx, ny, goal.first, goal.second);
								open_set.push(Node(nx, ny, tentative_g, h));
								came_from[nx][ny] = {x, y};
						}
				}
		}

		return {};  // 経路が見つからない場合、空のベクタを返す
}

int main() {
		vector<vector<int>> grid = {
				{0, 0, 0, 0, 0},
				{0, 0, 0, 0, 0},
				{1, 1, 1, 0, 1},
				{0, 0, 0, 0, 0},
		};

		pair<int, int> start = {0, 0};
		pair<int, int> goal = {3, 4};

		vector<pair<int, int>> path = a_star(grid, start, goal);

		// if (path.empty()) {
		// 		cout << "経路が見つかりませんでした。" << endl;
		// } else {
				cout << "見つかった経路：" << endl;
				for (const auto &p : path) {
						cout << "(" << p.first << ", " << p.second << ")" << endl;
				}
		// }

		return 0;
}
