#include "global_path_planner.h"

AstarPath::AstarPath():private_nh("~")
{
    private_nh.param("hz",hz,{10});                                     //実行した後に、hzの値を変えることができる。　{}はデフォルト値
    private_nh.param("path_check",path_check,{false});
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
        int row = the_map.info.height;          //row = 4000
        int col = the_map.info.width;           //col = 4000
        map_grid = vector<vector<int>>(row,vector<int>(col,0));

        //change 1D the_map to 2D
        for(int i=0; i<row; i++)
        {
            for(int j=0; j<col; j++)
            {
                map_grid[i][j] = the_map.data[i+j*row];
            }
        }
        // origin mean point which is edge of left down
        origin.x = the_map.info.origin.position.x;      //origin.x = -100
        origin.y = the_map.info.origin.position.y;      //origin.y = -100
    }
}