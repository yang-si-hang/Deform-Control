#include "feature_extraction.hpp"


#include <iostream>
#include <pcl/io/pcd_io.h>                    
#include <pcl/point_types.h>                         
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;


extern string path_read_clod;
extern string path_output_cloud;
extern int step_n;

void fitMultipleLines(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::ModelCoefficients>& lineCoff)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;               // 创建拟合对象
	seg.setOptimizeCoefficients(true);                     // 设置对估计模型参数进行优化处理
	seg.setModelType(pcl::SACMODEL_LINE);                  // 设置拟合模型为直线模型
	seg.setMethodType(pcl::SAC_RANSAC);                    // 设置拟合方法为RANSAC
	seg.setMaxIterations(1000);                             // 设置最大迭代次数
	seg.setDistanceThreshold(0.005);                       // 判断是否为模型内点的距离阀值/设置误差容忍范围

	int i = 0, nr_points = cloud->points.size();
	int k = 0;
	while (k < 5 && cloud->points.size() > 0.1 * nr_points)// 从0循环到5执行6次，并且每次cloud的点数必须要大于原始总点数的0.1倍
	{
		pcl::ModelCoefficients coefficients;
		seg.setInputCloud(cloud);                         // 输入点云						 
		seg.segment(*inliers, coefficients);              // 内点的索引，模型系数

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outside(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (inliers->indices.size() > 5) // 判断提取直线上的点数是否小于20个点，小于的话该直线不合格
		{
			lineCoff.push_back(coefficients);             // 将参数保存进vector中
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // 创建点云提取对象
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);                   // 设置为false，表示提取内点
			extract.filter(*cloud_line);

			extract.setNegative(true);                    // true提取外点（该直线之外的点）
			extract.filter(*outside);                     // outside为外点点云
			cloud.swap(outside);                          // 将cloud_f中的点云赋值给cloud
		}
		else
		{
			PCL_ERROR("Could not estimate a line model for the given dataset.\n");
			break;
		}

		pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd221209/全_夹子带纸/output/line_"+to_string(i + 1)+".pcd",*cloud_line);

		i++;
		k++;
	}
	cout << "一共拟合出" << lineCoff.size() << "条直线，直线系数分别为：\n" << endl;

	for (auto l : lineCoff)
	{
		cout << "直线方程为：\n"
			<< "   (x - " << l.values[0] << ") / " << l.values[3]
			<< " = (y - " << l.values[1] << ") / " << l.values[4]
			<< " = (z - " << l.values[2] << ") / " << l.values[5] << endl;

	}
}



// int main(int argc, char** argv)
// {
// 	// 加载点云
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data//L.pcd", *cloud) == -1)
// 	{
// 		PCL_ERROR("点云读取失败 \n");
// 		return (-1);
// 	}

// 	vector<pcl::ModelCoefficients> LinesCoefficients;
// 	fitMultipleLines(cloud, LinesCoefficients);

// 	cout << "一共拟合出" << LinesCoefficients.size() << "条直线，直线系数分别为：\n" << endl;

// 	for (auto l : LinesCoefficients)
// 	{
// 		cout << l.values[0] << "," << l.values[1]
// 			<< "," << l.values[2] << "," << l.values[3]
// 			<< "," << l.values[4] << "," << l.values[5] << endl;
// 	}

// 	return 0;
// }





//骨架提取（二值图像 mask，输出图像）=================================================
void skeleton_extract(cv::Mat &mask,cv::Mat & img_output)
{
	cv::Mat tmp = mask.clone();
	img_output = mask.clone();

	for (int i = 0; i < 10; i++)
	{
		thinV(img_output, tmp);
		img_output = tmp.clone(); ///< 多次拷贝
		thinH(tmp, img_output);
		tmp = img_output.clone();
	}
}


void thinH(const cv::Mat &src, cv::Mat &dst)
{
    int rows = src.rows;
    int cols = src.cols;

    // 映射表
    uchar array[] = { 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0,
                      1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0
                    };

    uchar NEXT = 1;

    for (int j = 0; j < cols; j++)
	{
        for (int i = 0; i < rows; i++)
		{

            if (NEXT == 0)
			{
                NEXT = 1;
            } 
			else
			{
                int M = 1;
                if (i > 0 && i < rows - 1)
                    M = src.at<uchar>(i - 1, j) + src.at<uchar>(i, j) + src.at<uchar>(i + 1, j);

                if (src.at<uchar>(i, j) == 0 && M != 0)
				{
                    uchar a[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

                    for (int k = 0; k < 3; k++) {
                        for (int l = 0; l < 3; l++) {
                            // 如果 3 * 3 矩阵的点不在边界且这些值为 0，也就是黑色的点
                            int posy = i - 1 + k;
                            int posx = j - 1 + l;

                            if (-1 < (posy) && (posy) < rows &&
                                    -1 < (posx) && (posx) < cols &&
                                    dst.at<uchar>(posy, posx) == 255){
                                a[k * 3 + l] = 1;
                            }
                        }
                    }

                    int sum = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128;
                    dst.at<uchar>(i, j) = array[sum] * 255;
                    if (array[sum] == 1)
                        NEXT = 0;
                }
            }
        }
    }
}


void thinV(const cv::Mat &src, cv::Mat &dst)
{
    int rows = src.rows;
    int cols = src.cols;

    // 映射表
    uchar array[] = { 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
                      0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                      1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0,
                      1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0
                    };

    uchar NEXT = 1;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {

            if (NEXT == 0) {
                NEXT = 1;
            } else {
                int M = 1;
                if (j > 0 && j <cols - 1)
                    M = src.at<uchar>(i, j - 1) + src.at<uchar>(i, j) + src.at<uchar>(i, j + 1);

                if (src.at<uchar>(i, j) == 0 && M != 0) {
                    uchar a[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

                    for (int k = 0; k < 3; k++) {
                        for (int l = 0; l < 3; l++) {

                            // 如果 3 * 3 矩阵的点不在边界且这些值为 0，也就是黑色的点
                            int posy = i - 1 + k;
                            int posx = j - 1 + l;

                            if (-1 < (posy) && (posy) < rows &&
                                    -1 < (posx) && (posx) < cols &&
                                    dst.at<uchar>(posy, posx) == 255) {
                                a[k * 3 + l] = 1;
                            }
                        }
                    }

                    int sum = a[0] * 1 + a[1] * 2 + a[2] * 4 + a[3] * 8 + a[5] * 16 + a[6] * 32 + a[7] * 64 + a[8] * 128;
                    dst.at<uchar>(i, j) = array[sum] * 255;
                    if (array[sum] == 1)
                        NEXT = 0;
                }
            }
        }
    }
}


//===============================================================================

//输入bgr图像,该函数将其转gray后canny算法提取边缘
void edge_extract_Canny(cv::Mat & bgr_in,cv::Mat & Canny_edge_img)
{
	cv::Mat gray_img;
    cv::cvtColor(bgr_in,gray_img,cv::COLOR_BGR2GRAY);
    imshow("gray",gray_img);
    cv::Canny(gray_img,Canny_edge_img,50,150);
    imshow("Canny_edge",Canny_edge_img);

}

//由二值化图像提取轮廓，（并取出最长的边缘）
void edge_extract_binary(cv::Mat & mask_in, cv::Mat &mask_edge, cv::Mat &draw_img)
{
    cv::Mat one_contour_mat;
    vector<vector<cv::Point>> contours;
    cv::findContours(mask_in,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    mask_edge = cv::Mat::zeros(mask_in.size(),CV_8U);

    // for(int i=0;i<contours.size();++i)//遍历，所有边缘都显示
    // {
    //     one_contour_mat = draw_img.clone();
    //     cv::drawContours(one_contour_mat,contours,i,cv::Scalar(255,255,0));
    //     imshow("edge_binary_"+to_string(i),one_contour_mat);
    // }


    int j=0;
    for(int i=1;i<contours.size();++i)//显示最长的边缘
    {
        if(contours[i].size()>contours[j].size()) j=i;
    }
    cv::drawContours(draw_img,contours,j,cv::Scalar(255,255,0));
    cv::drawContours(mask_edge,contours,j,cv::Scalar(255));
    // cv::imshow("edge_binary_"+to_string(j),draw_img);
    
    cout<<"检测到边缘数量："<<contours.size()<<'\n'
        <<"最长的边缘序号："<<j<<"(0~"<<contours.size()-1<<")"<<endl;
}


//计算各点法向量
void normal_estimation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                        pcl::PointCloud<pcl::Normal>::Ptr normals,
                        const double radius,
                        const bool save_colored_normals ,
                        const string path_output_cloud,
                        const int pcd_num)//输入点云、法向量输出数据、临近点半径(单位 m)
{
    //创建法线估计估计向量
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud_input);
    //创建一个空的KdTree对象，并把它传递给法线估计向量
    //基于给出的输入数据集，KdTree将被建立
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    norm_est.setSearchMethod (tree);
    //半径
    norm_est.setRadiusSearch (radius);
    norm_est.compute (*normals);

    // //可视化
    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // viewer.setBackgroundColor (0.0, 0.0, 0.0);
    // viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_input, normals);

    // while (!viewer.wasStopped ())
    // {
    //     viewer.spinOnce ();
    // }

    if(save_colored_normals)
    {    
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normal_color(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloud_normal_color = *cloud_input;
        for(int i=0;i<cloud_input->size();++i)
        {
            double x,y,z=normals->points[i].normal_z;
            if(z<0)
            {
                x = - normals->points[i].normal_x;
                y = - normals->points[i].normal_y;
                z = -z;
            }       

            else
            {
                x =  normals->points[i].normal_x;
                y =  normals->points[i].normal_y;
            }
        
            cloud_normal_color->points[i].r = (y+1)/2*255;
            cloud_normal_color->points[i].g = (x+1)/2*255;
            cloud_normal_color->points[i].b = (z+1)/2*255;

            // cloud_normal_color->points[i].r = 0;
            // cloud_normal_color->points[i].g = (x+1)/2*255;
            // cloud_normal_color->points[i].b = (z+1)/2*255;

            // if(y>0)
            // {
            //     cloud_normal_color->points[i].r=255;
            //     cloud_normal_color->points[i].g=0;
            //     cloud_normal_color->points[i].b=0;
            // }
            // else
            // {
            //     cloud_normal_color->points[i].r=0;
            //     cloud_normal_color->points[i].g=0;
            //     cloud_normal_color->points[i].b=255;
            // }
            
        }
        if(!cloud_normal_color->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"normal_color.pcd",*cloud_normal_color);
        else cout<<"clout_temp(feature_extraction) is empty!!!"<<endl;
    }
    

    return ;
}


//根据点云各点法向量，提取边界(法向量突变)，同时提取非边界点，并对非边界点(默认进行)去除离群点处理
void boundary_normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                        pcl::PointCloud<pcl::Normal>::Ptr normals,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_boundary,
                        double boundary_search_radius,
                        double boundary_search_angel_degree,
                        bool cloud_no_boudary_OutlierFielter,
                        double outfilter_radius,
                        int outfilter_n_threshold)
{
    if(!cloud_no_boundary) cloud_no_boudary_OutlierFielter=0;

    cloud_boundary->clear();
    cloud_no_boundary->clear();
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundEst;

    pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
    boundEst.setInputCloud(cloud_input); //设置输入的点云
    boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
    boundEst.setRadiusSearch(boundary_search_radius); //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
    boundEst.setAngleThreshold(boundary_search_angel_degree / 180.0 * M_PI ); //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>)); //设置搜索方式KdTree

    //参数实验：这里的 boundEst 除了compute 都是覆盖前面的赋值的----------------------------------------------------------
    /*
    double r_b=0;
    double angle_threshold_degree = 10;
    for(int i_boundary=0;i_boundary<150;i_boundary++)
    {
        cloud_boundary->clear();
        if(cloud_no_boundary) cloud_no_boundary->clear();

        r_b += 0.001;
        // angle_threshold_degree +=1;

        boundEst.setRadiusSearch(r_b); //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
        // boundEst.setAngleThreshold(angle_threshold_degree/180*M_PI); //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
        boundEst.compute(boundaries); //将边界估计结果保存在boundaries



        //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
        for (int i = 0; i < cloud_input->points.size(); i++)
        {
            if (boundaries[i].boundary_point > 0)
            {
                cloud_input->points[i].r=0;
                cloud_input->points[i].g=255;
                cloud_input->points[i].b=0;
                cloud_boundary->push_back(cloud_input->points[i]);
            }
            else
            {
                if(cloud_no_boundary)
                    cloud_input->points[i].r=0;
                    cloud_input->points[i].g=0;
                    cloud_input->points[i].b=255;
                    cloud_no_boundary->push_back(cloud_input->points[i]);
            }
            
        }       
        pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd230223/2-output/temp/combined"+to_string((int)(1000*r_b))+".pcd",*cloud_boundary + *cloud_no_boundary);
        // pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd230223/2-6/output/boundary"+to_string((int)(1000*r_b))+".pcd",*cloud_boundary);
        // pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd230223/2-6/output/no_boundary"+to_string((int)angle_threshold_degree)+".pcd",*cloud_no_boundary);

    }
    */
    //-----------------------------------------------------------------------------------------------------------------
    

    boundEst.compute(boundaries); //将边界估计结果保存在boundaries
    //存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
    for (int i = 0; i < cloud_input->points.size(); i++)
    {
        if (boundaries[i].boundary_point > 0)
        {
            cloud_boundary->push_back(cloud_input->points[i]);
        }
        else
        {
            if(cloud_no_boundary)
                cloud_no_boundary->push_back(cloud_input->points[i]);
        }
        
    }       


    // if(cloud_no_boudary_OutlierFielter)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_boundary_OutF(new pcl::PointCloud<pcl::PointXYZRGB>);

        //参数尝试-------------------------------------------------------------------------------------------------------------
        /*
        for(double r_of = 0.002; r_of<0.01; r_of += 0.002)
        {
            for(int n_of=10;n_of<51;n_of+=2)
            {
                cloud_no_boundary_OutF->clear();
                OutlierFilter(cloud_no_boundary,cloud_no_boundary_OutF,r_of,n_of);
                if(!cloud_no_boundary_OutF->empty())    pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd230223/2-output/temp/no_boundary_OutF_"+to_string((int)(r_of*1000))+'_'+to_string(n_of)+".pcd",*cloud_no_boundary_OutF);
                else cout<<"("<<r_of<<", "<<n_of<<')' <<"\t";
            }
            cout<<endl;
        }
        */
        //-------------------------------------------------------------------------------------------------------------
        cloud_no_boundary_OutF->clear();
        OutlierFilter(cloud_no_boundary,cloud_no_boundary_OutF,outfilter_radius,outfilter_n_threshold);
        *cloud_no_boundary = *cloud_no_boundary_OutF;
    }
}


//提取点云最高(z轴)的一团点
void top_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_points,
                    double z_range,
                    double radius)
{
    if(!cloud_input->size()) 
    {
        cout<<"top points: cloud in size 0!!!!"<<endl;
        return;
    }
    cloud_top_points -> clear();
    int cloud_size = cloud_input->size();
    double  top_point_x = cloud_input->points[0].x,
            top_point_y = cloud_input->points[0].y,
            top_point_z = cloud_input->points[0].z,
            top_point_num=0;

    double xmin=10000, ymin=10000;
    //筛选一下一角范围
    for(int i=1; i<cloud_size; ++i)
    {
        if(cloud_input->points[i].x < xmin)  xmin = cloud_input->points[i].x;
        if(cloud_input->points[i].y < ymin)  ymin = cloud_input->points[i].y;
        
    }


    for(int i=1; i<cloud_size; ++i)
    {
        if(cloud_input->points[i].z > top_point_z && cloud_input->points[i].x-xmin > 0.06 && cloud_input->points[i].y-ymin > 0.06)
        {
            top_point_x = cloud_input->points[i].x;
            top_point_y = cloud_input->points[i].y;
            top_point_z = cloud_input->points[i].z;
            top_point_num=i;
        }
    }
    cloud_top_points->push_back(cloud_input->points[top_point_num]);

    for(int i=1; i<cloud_size; ++i)
    {
        if(cloud_input->points[i].z > top_point_z - z_range  
            &&  sqrt(   pow(cloud_input->points[i].x-top_point_x,2)+
                        pow(cloud_input->points[i].y-top_point_y,2)+
                        pow(cloud_input->points[i].z-top_point_z,2)     ) < radius) 
            cloud_top_points->push_back(cloud_input->points[i]);
    }
    cout<<"top points size: "<<cloud_top_points->size()<<endl;
    return;
}



//RANSAC法提取平面
void plane_extracte_RANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
                                Eigen::VectorXf & coefficient,
                                double distance_threshold,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_input));	//选择拟合点云与几何模型
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_plane);	//创建随机采样一致性对象
	ransac.setDistanceThreshold(distance_threshold);	//设置距离阈值，与平面距离小于  的点作为内点
	ransac.computeModel();				//执行模型估计



	//---------- 根据索引提取内点 ----------
    if(cloud_plane)
    {
        vector<int> inliers;				//存储内点索引的向量
        ransac.getInliers(inliers);			//提取内点对应的索引
        pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_input, inliers, *cloud_plane);
    }


	ransac.getModelCoefficients(coefficient);
	cout << "平面方程为：\n"
		<< coefficient[0] << "x + "
		<< coefficient[1] << "y + "
		<< coefficient[2] << "z + "
		<< coefficient[3] << " = 0"
		<< endl;
    

}



//根据mask提取标记圆点
struct point_marker_struct
{
    int contour_index;
    double area;
    double x;
    double y;
};
bool compare_point_markers(const point_marker_struct&  a,const point_marker_struct& b )//用于 points_marker_extract_mask 函数中排序
{
    return a.area<b.area;
}
void points_marker_extract_mask(cv::Mat & mask_in, cv::Mat &mask_point_centers, cv::Mat &draw_img )
{
    cv::Mat one_contour_mat;
    vector<vector<cv::Point>> contours;
    cv::findContours(mask_in,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    mask_point_centers = cv::Mat::zeros(mask_in.size(),CV_8U);

    // for(int i=0;i<contours.size();++i)//遍历，所有边缘都显示
    // {
    //     one_contour_mat = draw_img.clone();
    //     cv::drawContours(one_contour_mat,contours,i,cv::Scalar(255,255,0));
    //     imshow("edge_binary_"+to_string(i),one_contour_mat);
    // }

    vector <uint8_t> point_index; //用于存储marker点对应的contour的序号
    vector <point_marker_struct> point_markers; //对应的contour序号、面积、重心x、y 
    point_marker_struct temp_point_marker;
    int j=0;
    for(int i=0;i<contours.size();++i)
    {
        cv::Moments m = cv::moments(contours[i]); //获取轮廓的距
        //计算轮廓质心 
        double cx = m.m10 / m.m00;
        double cy = m.m01 / m.m00;
        temp_point_marker.contour_index = i;
        temp_point_marker.area = cv::contourArea(contours[i]);
        temp_point_marker.x = cx;
        temp_point_marker.y = cy;
        point_markers.push_back(temp_point_marker);
        
        if(temp_point_marker.area<40 || temp_point_marker.area>100)
        {
            cv::circle(draw_img, cv::Point2d(cx,cy), 0, cv::Scalar(0, 255, 120,255), -1);
            cv::circle(mask_point_centers, cv::Point2d(cx,cy), 0, cv::Scalar(255), -1);
        }
        // cv::drawContours(draw_img,contours,i,cv::Scalar(255,255,0,255));
        // cv::drawContours(mask_edge,contours,j,cv::Scalar(255));
    }
    std::sort(point_markers.begin(),point_markers.end(), compare_point_markers);
    for(int i=0; i<point_markers.size(); ++i)
    {
        cout<<"point:"<<i<<"  contour:"<< point_markers[i].contour_index <<"  area:"<<point_markers[i].area <<"\t(x,y):("<<point_markers[i].x<<','<<point_markers[i].y<<')'<<endl;
    }

    if(!draw_img.empty())	cv::imwrite(path_output_cloud+"marker_"+to_string(step_n)+".png",draw_img);
	else cout<<"draw_img is empty!!!"<<endl;

    if(!mask_point_centers.empty())	cv::imwrite(path_output_cloud+"point_centers"+to_string(step_n)+".png",mask_point_centers);
	else cout<<"mask_point_centers is empty!!!"<<endl;
    // cv::imshow("edge_binary_"+to_string(j),draw_img);
}



//从mask提取标记线条区域
void line_extract_mask(cv::Mat & mask_in, cv::Mat &mask_line, cv::Mat &draw_img)
{
    mask_line = cv::Mat::zeros(cv::Size(mask_in.cols,mask_in.rows),CV_8U);
	cv::Mat lables, stats, centroid;
	cv::connectedComponentsWithStats(mask_in, lables, stats, centroid, 8, CV_16U);

	// cout<<"mask_in size:"<<mask_in.size()<<"\n"
	// 	<<"mask_out size:"<<mask_out.size()<<'\n'
	// 	<<"lables size:"<<lables.size()<<endl;
    int line_lable_num = 0;
    for(int i=0; i<stats.rows; ++i)
    {
        cout<<"stats:"<< i << "\tarea:"<< stats.at<int>(i,cv::CC_STAT_AREA)<<endl;
        if  (
                (step_n == 0 && stats.at<int>(i,cv::CC_STAT_AREA) > 100 && stats.at<int>(i,cv::CC_STAT_AREA) < 125)
                ||(step_n == 1 && stats.at<int>(i,cv::CC_STAT_AREA) > 120 && stats.at<int>(i,cv::CC_STAT_AREA) < 200)
                ||(step_n == 2 && stats.at<int>(i,cv::CC_STAT_AREA) > 120 && stats.at<int>(i,cv::CC_STAT_AREA) < 200)

            )
        {
            if(line_lable_num) cout<<"line lables: "<< i <<endl;
            else
            {
                line_lable_num = i;
            }
        }
    }
    cout << "line lable: "<<line_lable_num<<endl;


	int rows=mask_in.rows, cols=mask_in.cols;
	int i,j,lable_num;
	for(i=0;i<rows;++i)
	{
		for(j=0;j<cols;++j)
		{
			if(lables.at<uint16_t>(i,j) == line_lable_num)   mask_line.at<uchar>(i,j)=255;
            else    mask_line.at<uchar>(i,j)=0;
		}
	}

    //=============== 开闭运算、骨架提取 ================================
    cv::Mat mask_inv, line_opened, line_closed, line_skeleton;
    if(step_n == 0)
    {
        cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::Mat element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        

        cv::morphologyEx(mask_line, line_closed, cv::MORPH_CLOSE, element_close);
        
        cv::morphologyEx(line_closed, line_opened, cv::MORPH_OPEN, element_open);
        // cv::morphologyEx(mask_color, mask_closed, cv::MORPH_CLOSE, element_close);

        
        
        maks_img_invert(line_opened, mask_inv);
        // maks_img_invert(line_closed, mask_inv);

        skeleton_extract(mask_inv,line_skeleton);
        maks_img_invert(line_skeleton, mask_line);
    }
    else if(step_n == 1)
    {
        cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        

        cv::morphologyEx(mask_line, line_closed, cv::MORPH_CLOSE, element_close);
        
        cv::morphologyEx(mask_line, line_opened, cv::MORPH_OPEN, element_open);
        cv::morphologyEx(line_opened, line_closed, cv::MORPH_CLOSE, element_close);

        // cv::morphologyEx(mask_color, mask_closed, cv::MORPH_CLOSE, element_close);

        
        
        // maks_img_invert(line_opened, mask_inv);
        maks_img_invert(line_closed, mask_inv);
        skeleton_extract(mask_inv,line_skeleton);
        maks_img_invert(line_skeleton, mask_line);
    }
    else
    {
        cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        

        cv::morphologyEx(mask_line, line_closed, cv::MORPH_CLOSE, element_close);
        
        cv::morphologyEx(mask_line, line_opened, cv::MORPH_OPEN, element_open);
        cv::morphologyEx(line_opened, line_closed, cv::MORPH_CLOSE, element_close);

        // cv::morphologyEx(mask_color, mask_closed, cv::MORPH_CLOSE, element_close);

        
        
        // maks_img_invert(line_opened, mask_inv);
        maks_img_invert(line_closed, mask_inv);
        skeleton_extract(mask_inv,line_skeleton);
        maks_img_invert(line_skeleton, mask_line);
    }

	for(i=0; i< draw_img.rows; ++i)
        {
            for(j=0; j<draw_img.cols; ++j)
            {
                if(line_skeleton.at<uchar>(i,j)==0)     cv::circle(draw_img, cv::Point2d(j,i), 0, cv::Scalar(0, 255, 120, 255), -1);
            }
        }
    //================================================================


    

    if(!line_closed.empty())	cv::imwrite(path_output_cloud+"line_closed"+to_string(step_n)+".png",line_closed);
	else cout<<"line_closed is empty!!!"<<endl;

    if(!line_opened.empty())	cv::imwrite(path_output_cloud+"line_opened"+to_string(step_n)+".png",line_opened);
	else cout<<"line_opened is empty!!!"<<endl;

    if(!mask_line.empty())	cv::imwrite(path_output_cloud+"line_mask"+to_string(step_n)+".png",mask_line);
	else cout<<"mask_line is empty!!!"<<endl;

    if(!draw_img.empty())	cv::imwrite(path_output_cloud+"line_draw"+to_string(step_n)+".png",draw_img);
	else cout<<"draw_img is empty!!!"<<endl;

    if(!line_skeleton.empty())	cv::imwrite(path_output_cloud+"line_skeleton"+to_string(step_n)+".png",line_skeleton);
	else cout<<"line_skeleton is empty!!!"<<endl;
}