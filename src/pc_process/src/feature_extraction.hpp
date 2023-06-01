#ifndef FEATURE_EXTRAXTION
#define FEATURE_EXTRAXTION

#include <iostream>
#include <algorithm>

#include <pcl/io/pcd_io.h>                    
#include <pcl/point_types.h>                         
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/integral_image_normal.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/sample_consensus/ransac.h>
// #include <pcl/visualization/cloud_viewer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include"pc_to_mesh.hpp"

using namespace std;

void fitMultipleLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::ModelCoefficients>& lineCoff);
// void fitMultipleLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::ModelCoefficients>& lineCoff);


//骨架提取相关函数
void thinH(const cv::Mat &src, cv::Mat &dst);
void thinV(const cv::Mat &src, cv::Mat &dst);
void skeleton_extract(cv::Mat &mask,cv::Mat & img_output);//主体

//边缘提取相关函数
void edge_extract_Canny(cv::Mat & bgr_in,cv::Mat & Canny_edge_img);//bgr图像canny算法边缘提取
void edge_extract_binary(cv::Mat & mask_in, cv::Mat &mask_edge, cv::Mat &draw_img);//二值化图像边缘提取

//计算各点法向量
void normal_estimation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                        pcl::PointCloud<pcl::Normal>::Ptr normals,
                        const double radius,
                        const bool save_colored_normals = 0,
                        const string path_output_cloud="",
                        const int pcd_num=0);//输入点云、法向量输出数据、临近点半径(单位 m)


//根据点云各点法向量，提取边界(法向量突变)，同时提取非边界点，并对非边界点(默认进行)去除离群点处理
void boundary_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                        pcl::PointCloud<pcl::Normal>::Ptr normals,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_boundary = NULL,
                        double boundary_search_radius = 0.005,
                        double boundary_search_angel_degree = 20.0,
                        bool cloud_no_boudary_OutlierFielter = 1,
                        double outfilter_radius = 0.011,
                        int outfilter_n_threshold = 50);

//提取点云最高(z轴)的一团点,取 z >（z_top-z_range） &&   距离最高点距离<r 范围的点
void top_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_points,
                    double z_range=0.002,
                    double radius=0.005);


//RANSAC法提取平面
void plane_extracte_RANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                                Eigen::VectorXf & coefficient,
                                double distance_threshold,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane);

//根据mask提取标记圆点
void points_marker_extract_mask(cv::Mat & mask_in, cv::Mat &mask_point_centers, cv::Mat &draw_img );

//根据mask提取标记线
void line_extract_mask(cv::Mat & mask_in, cv::Mat &mask_line, cv::Mat &draw_img);


#endif //FEATURE_EXTRAXTION