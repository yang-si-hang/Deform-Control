#ifndef PC_TO_MESH
#define PC_TO_MESH

#include <stdlib.h>
#include <unistd.h>
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
#include <math.h>
#include <algorithm>

//======= 点云相关 =========
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
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

#include <pcl/filters/voxel_grid.h>           //用于体素网格化的滤波类头文件 
#include <pcl/filters/filter.h>             //滤波相关头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>          //滤波相关类头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/radius_outlier_removal.h> //统计方法去除离群点
#include <pcl/filters/approximate_voxel_grid.h>  //ApproximateVoxelGrid 
#include <pcl/common/transforms.h> 
#include <pcl/filters/uniform_sampling.h>//均匀采样
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/distances.h>


// #include "farthest_point_sampling.h"

// #include "farthest_point_sampling.hpp"





#include "feature_extraction.hpp"
#include "mine_time.hpp"


enum COLOR_NAME{RED, BLUE, BLACK, WHITE, BLACK_RED, YELLOW, RED_WHITE, ALL};

//计算相机坐标变换
void pose_init(Eigen::Matrix<double,4,4> & T_world_camera);
//下采样
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double length);//体素
void DownSample_FPS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, 
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, 
					int size_out);//最远点
//去除离群点
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out, double radius, int num_of_surround_points);
//对点云的预处理， 点云下采样 + 去离群点滤波
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double l_DownSample, double r_OutlierFilter, int n_OF_threshold);
//遍历点，筛颜色
void color_depend_extract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out,enum COLOR_NAME color_name) ;
//复制点云，复制多层
void add_cloud_layers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out, double translation_x, double translation_y, double translation_z, int num_of_added_layers);
//根据坐标范围去除点
void cut_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out); //根据x值去除部分云
//从png，按颜色提取图像
void png_color_extract(const cv::Mat & img_input,cv::Mat & img_output,cv::Mat & mask,enum COLOR_NAME color_name);
//二值化图像取反
void maks_img_invert(cv::Mat & src, cv::Mat & output);
//去除二值化图像的离群点
void remove_Outlier(cv::Mat &mask_in, cv::Mat &mask_out, int min_connect_pixcels=10, int connectivity=4);
//根据二值图像、完整原始png图像提取点云。
void img_cloud_match(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_origin,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_output,
						cv::Mat png_origin,
						cv::Mat mask);
//将几个点云放一起分别染色，用于观察
void combin_clouds(	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_1,
					const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_2,
					const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_3,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined);
//校正z轴
void z_axis_adjust(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_before_zCorrect);



//2023.04.03实验数据，采用手动画线提取,替代pc_to_mesh()
int pc_line_manual();

//手动修正xy轴，2023.04.03数据  
int xy_axis_adjust();

//手动更改各轴排序及原点位置，2023.04.03数据  
int axis_move();

//2023.04.14实验数据，采用手动edge提取=================================================
int pc_edge_manual();


//main
int pc_to_mesh();


#endif //PC_TO_MESH