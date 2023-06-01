#ifndef CAPTURE
#define CAPTURE


#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <vector>
#include <sys/time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <ctime>
//======= kbhit 相关 =======
#include <curses.h>
#include <termio.h>
#include <unistd.h>
// #include <term.h>  //有冲突？且看不出作用

//======= 点云相关 =========
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <pcl-1.8/pcl/filters/voxel_grid.h>
// #include <pcl-1.8/pcl/point_types.h>
// #include <pcl-1.8/pcl/point_cloud.h>
// #include <pcl-1.8/pcl/io/pcd_io.h>
// #include <eigen3/Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include <thread>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// ============================================================
#include "mine_time.hpp"


using namespace std;

void init_keyboard();
void close_keyboard();
char readch();
int kbhit();


int camera_init();
void camera_close();
void capture_once();

void depth_to_point_cloud(k4a_image_t & image_color, k4a_image_t & image_depth,
      k4a::transformation & k4aTransformation,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
     cv::Mat & mat_output);//生成RGB点云，是否要用RGB来筛选，在其中添加

void segmentation_region_growing_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    std::vector <pcl::PointIndices> & clusters);

int main_new();
int main_old();

#endif //CATURE