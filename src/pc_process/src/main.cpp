#include "ros/ros.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <vector>
#include <sys/time.h>
#include <ctime>

#include "pc_to_mesh.hpp"
// #include "test1.hpp"


// string path_read_clod = "/home/wyh/Desktop/Kinect_data/pcd221216/1-7/"; //最后要包含一个斜杠
// string path_output_cloud = "/home/wyh/Desktop/Kinect_data/pcd221216/1-7/output/";
string path_read_clod = "/home/wyh/Desktop/Kinect_data/pcd230223/2-12/"; //最后要包含一个斜杠
string path_output_cloud = "/home/wyh/Desktop/Kinect_data/pcd230223/2-12/output/";
// string path_read_clod = "/home/wyh/Desktop/realsense_data/"; //最后要包含一个斜杠
// string path_output_cloud = "/home/wyh/Desktop/realsense_data/output/";

int step_n=0;

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"pc_process_main");
    
    // test1(argc,argv);
    // pc_to_mesh(argc,argv);

    
    string temp_string = "/home/wyh/Desktop/Kinect_data/pcd230403/";
    // path_output_cloud = "/home/wyh/Desktop/Kinect_data/pcd230403/output/";
    // for(step_n=0; step_n<1; ++step_n)
    // {
    //     path_read_clod = temp_string + to_string(step_n)+'/';
    //     // path_output_cloud = temp_string + to_string(step_n)+'/';
    //     // pc_process_online(argc,argv);
    //     pc_to_mesh();
    // }


    //2023.04.03
    //2023.04.14
    //2023.04.19
    //2023.04.27
    //实验数据，采用手动画线提取---------------------------------------------------------------------------------------------------
    //为了确保各步骤坐标系相同，先对step0进行平面拟合z轴校准（下面两行代码），然后旧采用该校准的变换矩阵校准所有step
    // temp_string = "/home/wyh/Desktop/Kinect_data/pcd230427/";
    // path_output_cloud = "/home/wyh/Desktop/Kinect_data/pcd230427/line_manual/";
    // for(step_n=0; step_n<2; ++step_n)
    // {
    //     cout<<"-----------------\nstep "<<step_n<<endl;
    //     path_read_clod = temp_string + to_string(step_n)+'/';
    //     pc_line_manual();
    // }


    //xy州及原点转换------------------------------------------------------------------------------------------------------------
    // for(step_n=0; step_n<2; ++step_n)
    // {
    //     cout<<"-----------------\nstep "<<step_n<<endl;

    //     axis_move();
    // }

    //2023.04.14
    //2023.04.19
    //手动edge--------------------------------------------------------------------------------------------------------
    //函数里也有一个路径要改
    temp_string = "/home/wyh/Desktop/Kinect_data/pcd230427/";
    path_output_cloud = "/home/wyh/Desktop/Kinect_data/pcd230427/edge/";
    for(step_n=0; step_n<2; ++step_n)
    {
        cout<<"-----------------\nstep "<<step_n<<endl;
        path_read_clod = temp_string + to_string(step_n)+'/';
        pc_edge_manual();
    }


    return 0;
}
