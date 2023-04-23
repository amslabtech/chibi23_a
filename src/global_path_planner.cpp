#include "global_path_planner.h"

AstarPath::AstarPath():private_nh("~")
{
    private_nh.param("hz",hz,{10});                                     //実行した後に、hzの値を変えることができる。　{}はデフォルト値
    private_nh.param("map_check",map_check,{false});
    sub_map = nh.subscribe("/map",10,&AstarPath::map_callback,this);    //"/map"からマップをもらい、callback関数に送る
    pub_path = nh.advertise<nav_msgs::Path>("/global_path",1);

		// パラメータの読み込み 
		// FIXME: 関数に処理を分割したい
		vector<int> start_param;
		private_nh.getParam("start_param", start_param);
		start_position.first = start_param[0];
		start_position.second = start_param[1];

		vector<int> goal_x_params;
		vector<int> goal_y_params;

		private_nh.getParam("goals_x", goal_x_params);
		private_nh.getParam("goals_y", goal_y_params);

		if (goal_x_params.size( ) != goal_y_params.size()) ROS_ERROR("goals_x and goals_y are not same size");

		for( int i=0; i<goal_x_params.size(); ++i ) {
							goal_positions[i].first = goal_x_params[0];
							goal_positions[i].second = goal_y_params[1];
		 }
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
				map_resolution = the_map.info.resolution;      // マップの解像度を設定する
        map_row_length = the_map.info.height;          //row_count = 4000
        map_col_length = the_map.info.width;           //col_count = 4000
        map_grid = vector<vector<int>>(map_row_length,vector<int>(map_col_length,0));

        //change 1D the_map to 2D
        for(int i=0; i<map_row_length; i++)
        {
            for(int j=0; j<map_col_length; j++)
            {
                map_grid[i][j] = the_map.data[i+j*map_row_length];
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


	vector<vector<bool>> visited(rows, vector<bool>(cols, false));
	vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-1, -1}));

	priority_queue<Node, vector<Node>, CompareNode> open_set;
	open_set.push(Node(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second)));

	while (!open_set.empty()) {

		Node current = open_set.top(); // 最後の要素を取り出す
		open_set.pop(); // 取り出した要素を削除する
		int x = current.x, y = current.y;


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
	for(auto &p : global_path)
	{
	  printf("x=%lf, y=%lf, \n",( p.first - map_row_length/2 )* map_resolution,( p.second - map_col_length /2) * map_resolution); // デバッグ用a
		geometry_msgs::PoseStamped global_path_point;
		global_path_point.pose.position.x = (p.first - map_row_length / 2) * map_resolution;
		global_path_point.pose.position.y = (p.second - map_col_length / 2) * map_resolution;
		global_path_point.pose.position.z = 0;
		global_path_point.pose.orientation.x = 0;
		global_path_point.pose.orientation.y = 0;
		global_path_point.pose.orientation.z = 0;
		global_path_point.pose.orientation.w = 1;
		global_path_msgs.poses.push_back(global_path_point);
	}
		global_path_msgs.header.frame_id = "map";
		global_path_msgs.header.stamp = ros::Time::now();
}

void AstarPath::convert_map_position_to_node_index()
{
	start.first = (start_position.first / map_resolution) + map_row_length / 2;
	start.second = (start_position.second / map_resolution) + map_col_length / 2;

	for(auto &goal : goal_positions)
	{
		goals.push_back({ (goal.first / map_resolution) + map_row_length / 2, (goal.second / map_resolution) + map_col_length / 2 });
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
		astar.convert_map_position_to_node_index();
		astar.process();
		return 0;
}
