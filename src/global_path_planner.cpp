#include "global_path_planner.h"

AstarPath::AstarPath():private_nh("~")
{
    private_nh.param("hz",hz,{10});                                     //実行した後に、hzの値を変えることができる。　{}はデフォルト値
    private_nh.param("map_check",map_check,{false});
    sub_map = nh.subscribe("/map",10,&AstarPath::map_callback,this);    //"/map"からマップをもらい、callback関数に送る
    pub_path = nh.advertise<nav_msgs::Path>("/path",1);
}

void AstarPath::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
//const nav_msgs::Odometry::ConstPtr は、const型(内容を書き換えられない)、nav_msgsパッケージに含まれる、Odometry型のメッセージの、const型ポインタを表している
//&msgの&は、参照型(内容を書き換えられるように変数を渡すことができる)という意味ですが、(const型なので)ここでは特に気にする必要はない
{
    if(map_check){
        return;     //exit from processing on the way
    }
    else
    {
        the_map = *msg;
        int row_count = the_map.info.height;          //row_count = 4000
        int col_count = the_map.info.width;           //col_count = 4000
        map_grid = vector<vector<int>>(row_count,vector<int>(col_count,0));

        //change 1D the_map to 2D
        for(int i=0; i<row_count; i++)
        {
            for(int j=0; j<col_count; j++)
            {
                map_grid[i][j] = the_map.data[i+j*row_count];
            }
        }
        // origin mean point which is edge of left down
        origin.first = the_map.info.origin.position.x;      //origin.first = -100
        origin.second = the_map.info.origin.position.y;      //origin.first = -100
				map_check = true;
    }
}

vector<pair<int, int>> neighbors = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

bool AstarPath::isValid(int x, int y, int rows, int cols) {
		return x >= 0 && x < rows && y >= 0 && y < cols;
}

double AstarPath::heuristic(int x1, int y1, int x2, int y2) {
		return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

vector<pair<int, int>> AstarPath::a_star(vector<vector<int>> &map_grid, pair<int, int> start, pair<int, int> goal) {
	int rows = map_grid.size();
	int cols = map_grid[0].size();

	printf("rows=%d, cols=%d\n", rows, cols); // デバッグ用

	vector<vector<bool>> visited(rows, vector<bool>(cols, false));
	vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-1, -1}));

	printf("came_from=%d, %d\n", came_from[0][0].first, came_from[0][0].second); // デバッグ用

	priority_queue<Node, vector<Node>, CompareNode> open_set;
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

			if (isValid(nx, ny, rows, cols) && map_grid[nx][ny] == 0 && !visited[nx][ny]) {
				double tentative_g = current.g + 1;
				double h = heuristic(nx, ny, goal.first, goal.second);
				open_set.push(Node(nx, ny, tentative_g, h));
				came_from[nx][ny] = {x, y};
			}
		}
	}

	return {};  // 経路が見つからない場合、空のベクタを返す
}

// 複数のゴールを設定した場合の経路計画
vector<pair<int, int>> AstarPath::path_for_multi_goal() {
		vector<pair<int, int>> path;
		pair<int, int> current = start;
		for (auto &goal : goals) {
				auto p = a_star(map_grid, current, goal);
				path.insert(path.end(), p.begin(), p.end());
				current = goal;
		}
		return path;
}

// global_path を global_path_msgs に変換する
// - globalpath.first -> global_path_msgs.pose.position.x, global_path.second -> global_path_msgs.pose.position.y
void AstarPath::assign_global_path_msgs()
{
	float resolution = the_map.info.resolution; // 多分マップの解像度を設定する
	int row_count = the_map.info.height;          //row_count = 4000
	int col_count = the_map.info.width;           //col_count = 4000

	for(auto &p : global_path)
	{
		geometry_msgs::PoseStamped global_path_point;
		global_path_point.pose.position.x = (p.first - row_count / 2) * resolution;
		global_path_point.pose.position.y = (p.second - col_count / 2) * resolution;
		global_path_point.pose.position.z = 0;
		global_path_point.pose.orientation.x = 0;
		global_path_point.pose.orientation.y = 0;
		global_path_point.pose.orientation.z = 0;
		global_path_point.pose.orientation.w = 1;
		global_path_point.header.frame_id = "map";
		global_path_point.header.stamp = ros::Time::now();
		global_path_msgs.poses.push_back(global_path_point);
	}
}

void AstarPath::process()
{
	ros::Rate loop_rate(hz);
    while(ros::ok())
    {
			if(global_path.empty() && !map_grid.empty() && map_check)
			{
				global_path = path_for_multi_goal();
				assign_global_path_msgs();
				pub_path.publish(global_path_msgs);
			}

			ros::spinOnce();
			loop_rate.sleep();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "global_path_planner");           //node name "Global_path_planner"
		AstarPath astar;
		astar.start = {2000,2000}; // スタート地点を設定
		astar.goals = { {2160,2005},{2320,2010},{2320,2290},{2115,2287},{2000,2285},{1660,2280},{1660,1990},{1830,1995},{2000,2000} }; // 中継地点を含む複数のゴールを設定
		astar.process();
		return 0;
}



