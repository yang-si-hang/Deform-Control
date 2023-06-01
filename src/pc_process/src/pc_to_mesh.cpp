#include "pc_to_mesh.hpp"



using namespace std;

extern string path_read_clod;
extern string path_output_cloud;
extern int step_n;



const int pcd_num = 2;//同一步同一组，同状态下拍摄的图像号
// const int pcd_num = 10;

// COLOR_NAME color_name = RED;
COLOR_NAME color_name = BLACK;
Eigen::Matrix<double,4,4> T_world_rob = Eigen::Matrix<double,4,4>::Identity();
Eigen::Matrix<double,4,4> T_rob_camera = Eigen::Matrix<double,4,4>::Identity();
Eigen::Matrix<double,4,4> T_world_camera = Eigen::Matrix<double,4,4>::Identity();



void pose_init(Eigen::Matrix<double,4,4> & T_world_camera)
{
	//以前
	// T_world_rob << 	-0.07688595, 		-0.79200033,  		0.60566,		0.26238245,
	// 			-0.99637645,  		0.08319054, 		-0.01770039,  	0.06448822,
	// 			-0.03636647, 		-0.60482627, 		-0.79552666,  	0.35980569,
	// 			0,          		0,          		0,          	1;

	//2023.02.11_1
	// T_world_rob << 	 9.95686421e-01, -9.27822696e-02, -3.06787633e-05,  4.50303092e-01,
 	// 				-8.36882506e-02, -8.98237261e-01,  4.31469696e-01,  1.33135824e-01,
	// 				-4.00602945e-02, -4.29605950e-01, -9.02127430e-01,  5.18843730e-01,
	// 				0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;

	//2023.02.16
	// T_world_rob <<  0.99564112, -0.06684545,  0.06504185,  0.35558543,
 	// 				-0.08830488, -0.90005899,  0.42672716,  0.06313956,
 	// 				0.03001673, -0.43061062, -0.90203852,  0.62359642,
 	// 				0,          0,          0,          1;

	// //2023.02.22-1
	// T_world_rob << 	0.93983698, -0.0664278,   0.33510268,  0.32125818,
 	// 				-0.14628443, -0.96468731,  0.21904169,  0.21218634,
 	// 				0.30871884, -0.25488378, -0.91636616,  0.48364463,
 	// 				0,          0,          0,          1;

	//2023.02.23-2
	// T_world_rob << 	0.88519444,  0.04542094,  0.46299865,  0.0861932,
 	// 				-0.15689401, -0.90777271,  0.38901539,  0.18602135,
	// 				0.43796699, -0.41699597, -0.79642908,  0.59800451,
	// 				0,          0,          0,          1;

	//2023.04.03
	// T_world_rob <<  0.9944662,  -0.06188597, -0.08489465,  0.58626384,
 	// 				-0.01163011, -0.86796586,  0.49648767,  0.04625349,
 	// 				-0.10441128, -0.49275288, -0.86388245,  0.45555011,
	// 				0,          0,          0,         1;

	// //2023.04.14
	// T_world_rob <<  0.98526469, -0.02899796,  0.1685604,   0.42804851,
 	// 				-0.13195386, -0.75589806,  0.64125369,  0.07099997,
	// 				0.10881943, -0.65404682, -0.74858606,  0.45880861,
	// 				0,          0,          0,          1;

	//2023.04.19
	T_world_rob << 0.99971324 , 0.00692338 , 0.02292378 , 0.45247851,
 					-0.00290766 ,-0.91511412,  0.40318445,  0.08451252,
 					0.02376927, -0.40313549, -0.91483157,  0.36585093,
 					0 ,        0  ,        0  ,        1;

	// 2023.04.27
	T_world_rob << 	0.90406469,  0.12011674,  0.41016948,  0.29032426,
 					-0.02362219, -0.944185,    0.32856761,  0.17865167,
					0.42674234, -0.30673547, -0.8507669,   0.28717248,
					0.  ,        0. ,         0.  ,        1. ;       

	// T_rob_camera << 	0.99836954,  		0.00497973, 		-0.05686347,  		0.05397719,
 	// 					-0.00314286,  		0.99947175,  		0.03234715,  		0.17311164,
 	// 					0.05699451, 		-0.03211569,  		0.99785781,  		0.12613958,
 	// 					0,          		0,          		0,          		1;

	//2023.04.19
	T_rob_camera << 	0.98393069,  0.00224741,  0.17853667, -0.05401309,
 						-0.00572052,  0.99980424,  0.01894083,  0.00092234,
 						-0.17845915, -0.01965778,  0.98375094,  0.12583942,
						0,          0,          0,          1;
	
	
	T_world_camera = T_world_rob * T_rob_camera;
	// cout<< "T_world_camera:\n" << T_world_camera.matrix() <<endl;
}



//点云下采样
void DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double length)
{
	//down sample
	// std::cout << "begin downSample cloud_in size: " << cloud_in->size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //创建滤波对象
	downSampled.setInputCloud(cloud_in);            //设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(length, length, length);  //设置滤波时创建的体素体积为1cm的立方体（1为米，0.01就是1cm）
	downSampled.filter(*cloud_out);  //执行滤波处理，存储输出
		
	std::cout << "DownSample, size: " << cloud_in->size()<<"->"<< cloud_out->size() << std::endl;
	
}

void DownSample_FPS(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, 
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, 
					int size_out)  //farthest point sampling
{
	if(!cloud_in->size())
	{
		cout<<"DownSample FPS cloud in size: 0!!!!!!"<<endl;
		return;
	}
	int size_in = cloud_in->size();
	int center_index = (int)(size_in/2);
	vector<double> L={};
	vector<int> select_index={};
	double d;
	cloud_out->clear();

	int i;
	for(i=0; i<size_in; ++i)
	{
		L.push_back(pcl::squaredEuclideanDistance(cloud_in->points[i], cloud_in->points[center_index]));
	}
	select_index.push_back(max_element(L.begin(),L.end()) - L.begin());

	L.clear();
	for(i=0; i<size_in; ++i)
	{
		L.push_back(pcl::squaredEuclideanDistance(cloud_in->points[i], cloud_in->points[select_index[0]]));
	}
	select_index.push_back(max_element(L.begin(),L.end()) - L.begin());

	int temp;
	for(i=0; i<size_out -2; ++i)
	{
		for(int j=0; j<size_in; ++j)
		{
			d = pcl::squaredEuclideanDistance(cloud_in->points[j], cloud_in->points[select_index.back()]);
			if( d < L[j]) L[j]=d;
			temp = select_index.back();
		}
		select_index.push_back(max_element(L.begin(),L.end()) - L.begin());
	}


	for(i=0; i<select_index.size(); ++i)
	{
		cloud_out->push_back(cloud_in->points[select_index[i]]);
	}
	cout<<"下采样 FPS: "<<size_in<<"->"<<cloud_out->size()<<endl;
}

//去除离群点
void OutlierFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out, double radius, int num_of_surround_points)
{
	// std::cout << "begin outlierFilter cloud_in size: " << cloud_in->size() << std::endl;
	if(!cloud_in->size())
	{
		cout<<"OutlierFilter输入点云 "<<cloud_in->header.frame_id<<" size为0 !!!"<<endl;
		return;
	}
	//统计，有点慢
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	//sor.setInputCloud(cloud_in);                           //设置待滤波的点云
	//sor.setMeanK(50);                               //设置在进行统计时考虑的临近点个数
	//sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值，用来倍乘标准差
	//sor.filter(*cloud_out);                    //滤波结果存储到cloud_filtered
	//
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> pcFilter;  //创建滤波器对象
	pcFilter.setInputCloud(cloud_in);             //设置待滤波的点云
	pcFilter.setRadiusSearch(radius);               // 设置搜索半径
	pcFilter.setMinNeighborsInRadius(num_of_surround_points);      // 设置一个内点最少的邻居数目
	pcFilter.filter(*cloud_out);        //滤波结果存储到cloud_filtered

	std::cout << "OutlierFilter, size: " <<cloud_in->size()<<"->"<< cloud_out ->size() <<std::endl;

}



//对点云的预处理，就是点云下采样，去离群点滤波，
void PrePorcessingOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, double l_DownSample, double r_OutlierFilter, int n_OF_threshold) 
{
    
    uint64 time_temp = CurrentTime_ms();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());

	DownSample(cloud_in, cloud_temp, l_DownSample);

	OutlierFilter(cloud_temp, cloud_out,r_OutlierFilter,n_OF_threshold);
	
	std::cout << "point cloud pre processing time(s): " << CurrentTime_ms()-time_temp << std::endl;
}


//根据二值图像去除离群点。参数：输入、输出图像，要保留的连通域最小像素数，4连通/8连通
void remove_Outlier(cv::Mat &mask_in, cv::Mat &mask_out, int min_connect_pixcels, int connectivity)
{
	mask_out = cv::Mat::zeros(cv::Size(mask_in.cols,mask_in.rows),CV_8U);
	cv::Mat lables, stats, centroid;
	cv::connectedComponentsWithStats(mask_in, lables, stats, centroid, connectivity, CV_16U);

	// cout<<"mask_in size:"<<mask_in.size()<<"\n"
	// 	<<"mask_out size:"<<mask_out.size()<<'\n'
	// 	<<"lables size:"<<lables.size()<<endl;

	int rows=mask_in.rows, cols=mask_in.cols;
	int i,j,lable_num;
	for(i=0;i<rows;++i)
	{
		for(j=0;j<cols;++j)
		{
			lable_num = lables.at<uint16_t>(i,j);
			if(lable_num == 0) continue;
			else
			{
				if (stats.at<int>(lable_num,cv::CC_STAT_AREA) < min_connect_pixcels)
					mask_out.at<uchar>(i,j)=0;
				else 
					mask_out.at<uchar>(i,j)=255;
			}

		}
	}
	// cv::imshow("mask_with_outlier",mask_in);
	// cv::imshow("mask_remove_outlier",mask_out);
}

//从rgb点云进行颜色切割
void color_depend_extract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out,enum COLOR_NAME color_name) 
{
	pcl::ConstCloudIterator<pcl::PointXYZRGB> cloud_iterator(*cloud_in);
	cloud_iterator.reset();
	cloud_out->clear();
	bool color_detect_result=0;
	uint8_t rgb_max=0,rgb_min=0,rgb_r,rgb_b,rgb_g;
	float HSV_H=0;


	while(cloud_iterator.isValid())
	{
		rgb_r=(int)cloud_iterator->r;
		rgb_b=(int)cloud_iterator->b;
		rgb_g=(int)cloud_iterator->g;
		rgb_max = max(max(rgb_r,rgb_g),rgb_b);
		rgb_min = min(min(rgb_r,rgb_g),rgb_b);
		if(rgb_max==rgb_min) HSV_H=0;
		else if(rgb_max==rgb_r)
			{
				if(rgb_g < rgb_b) HSV_H = 60.0*(float)(rgb_g-rgb_b)/(float)(rgb_max-rgb_min)+360.0;
				else HSV_H = 60.0*(float)(rgb_g-rgb_b)/(float)(rgb_max-rgb_min);
			}
		else if(rgb_max==rgb_g) HSV_H = 60.0*(float)(rgb_b-rgb_r)/(float)(rgb_max-rgb_min)+120.0;
		else if(rgb_max==rgb_b) HSV_H = 60.0*(float)(rgb_r-rgb_g)/(float)(rgb_max-rgb_min)+240.0;
		else HSV_H=0;

		switch (color_name)
		{
		case RED:		color_detect_result=((int)cloud_iterator->r - (int)cloud_iterator->b > 25 );break;
		case BLACK:		color_detect_result=((int)cloud_iterator->b + (int)cloud_iterator->g +(int)cloud_iterator->r <200);break;
		case BLACK_RED:	color_detect_result=((int)cloud_iterator->r - (int)cloud_iterator->b > 25 || (int)cloud_iterator->b + (int)cloud_iterator->g +(int)cloud_iterator->r <200);break;
		case BLUE:	color_detect_result=(HSV_H>200 && HSV_H<250);break;
		default:
			color_detect_result=0;
			cout<<"Color Name Error!!!"<<endl;
			break;
		}
		// if((int)cloud_iterator->r - (int)cloud_iterator->b > 25 )  //红色
		// if((int)cloud_iterator->b + (int)cloud_iterator->g +(int)cloud_iterator->r <200) //黑色
		// if((int)cloud_iterator->r - (int)cloud_iterator->b > 25 || (int)cloud_iterator->b + (int)cloud_iterator->g +(int)cloud_iterator->r <200) //红+黑
		if(color_detect_result)
		{
			cloud_out->points.push_back(*cloud_iterator);
		}
		++cloud_iterator;
	}
	// cout<< "r:" << (int)cloud_iterator->r <<'\n'
	// 	<< "g:" << (int)cloud_iterator->g <<'\n'
	// 	<< "b:" << (int)cloud_iterator->b << endl;
	cloud_out->height=1;
    cloud_out->width=cloud_out->points.size();
    cloud_out->is_dense=false;
}

//从png图像，依据颜色进行提取。比较乱，待整理，现有一些零碎的针对各颜色部分的处理，包括对于特定图像坐标的切割
void png_color_extract(const cv::Mat & img_input,cv::Mat & img_output,cv::Mat & mask,enum COLOR_NAME color_name)
{
	cv::Mat hsv_img, bgr_img, bgr_32f, bgr_result,temp_img;
	cv::cvtColor(img_input, bgr_img, cv::COLOR_BGRA2BGR);
	bgr_img.convertTo(bgr_32f,CV_32F);
	cv::cvtColor(bgr_32f, hsv_img, cv::COLOR_BGR2HSV);

	cv::Scalar lower,upper,lower_r1,lower_r2,upper_r1,upper_r2;//红色单独用两套
	switch (color_name)
	{
	case BLUE:
		// lower=cv::Scalar(190,0,0);
		lower=cv::Scalar(	200,		6.0/255.0,		0			);//h:0~360,  s:0~1.0,  v:0~255
		upper=cv::Scalar(	250,		1.0,			0.69*255	);
		break; 

	case WHITE:
		lower=cv::Scalar(	0,		0.0,			0.8*255		);
		upper=cv::Scalar(	360,	20.0/255.0,		255.0		);
		break;

	case YELLOW:
		lower=cv::Scalar(	45,		35.0/255.0,		0.6*255		);
		upper=cv::Scalar(	70,		255.0,			255.0		);
		break;

	case BLACK:
		if(step_n == 0)
		{
			lower=cv::Scalar(	0,			0,					0		);
			upper=cv::Scalar(	360,		0.4,			0.55*255.0		);
		}
		else if(step_n ==1)
		{
			lower=cv::Scalar(	0,			0,					0		);
			upper=cv::Scalar(	360,		0.4,			0.4*255.0		);
		}
		else
		{
			lower=cv::Scalar(	0,			0,					0		);
			upper=cv::Scalar(	360,		0.4,			0.4*255.0		);
		}
		break;


	case RED:
		lower_r1=cv::Scalar(	0,		30.0/255.0,		0.4*255		);
		upper_r1=cv::Scalar(	25,		255.0,			255.0		);

		lower_r2=cv::Scalar(	320,	30.0/255.0,		0.4*255		);
		upper_r2=cv::Scalar(	360,	255.0,			255.0		);
		break;  

	case RED_WHITE:
		lower_r1=cv::Scalar(	0,		30.0/255.0,		0.4*255		);
		upper_r1=cv::Scalar(	25,		255.0,			255.0		);

		lower_r2=cv::Scalar(	320,	30.0/255.0,		0.4*255		);
		upper_r2=cv::Scalar(	360,	255.0,			255.0		);

		lower=cv::Scalar(	0,		0.0,			0.8*255		);
		upper=cv::Scalar(	360,	20.0/255.0,		255.0		);
		break;  

	case ALL:
		lower=cv::Scalar(	0,		0.0,		0		);
		upper=cv::Scalar(	360,	1.0,		255.0	);
		break;

	default:
		break;
	}

	//根据hsv阈值进行提取二值化图像
	if(color_name == RED)
	{
		cv::Mat mask_r1, mask_r2;
		cv::inRange(hsv_img, lower_r1, upper_r1, mask_r1);
		cv::inRange(hsv_img, lower_r2, upper_r2, mask_r2);
		mask = mask_r1 + mask_r2;
	}
	else if(color_name == RED_WHITE)
	{
		cv::Mat mask_r1, mask_r2;
		cv::inRange(hsv_img, lower_r1, upper_r1, mask_r1);
		cv::inRange(hsv_img, lower_r2, upper_r2, mask_r2);
		cv::inRange(hsv_img, lower, upper, mask);
		
		int height = mask.rows, width=mask.cols,i,j;
		for(i=0; i<height; ++i)
		{
			for(j=0; j<width; ++j)
			{
				if(i<360 || i>470 || j<500 ||j>800) mask.at<uchar>(i,j)=0;
			}
		}
		
		mask = mask + mask_r1 + mask_r2;
	}
	else if(color_name==BLUE || color_name==RED_WHITE) //针对蓝色边缘，通过坐标裁剪去除部分周围环境的蓝色
	{
		cv::inRange(hsv_img, lower, upper, mask);
		
		int height = mask.rows, width=mask.cols,i,j;
		for(i=0; i<height; ++i)
		{
			for(j=0; j<width; ++j)
			{
				if(i<360 || i>570 || j<500 ||j>800) mask.at<uchar>(i,j)=0;
			}
		}
	}
	else if(color_name == BLACK)//20230403
	{
		cv::inRange(hsv_img, lower, upper, mask);

		int height = mask.rows, width=mask.cols,i,j;
		for(i=0; i<height; ++i)
		{
			for(j=0; j<width; ++j)
			{
				if(step_n==0)	if(i<370 || i>470 || j<550 ||j>650) 	mask.at<uchar>(i,j)=0;
				if(step_n==1)	if(i<320 || i>470 || j<550 ||j>700) 	mask.at<uchar>(i,j)=0;
				if(step_n==2)	if(i<240 || i>470 || j<550 ||j>700) 	mask.at<uchar>(i,j)=0;
			}
		}
	}
	else
	{
		cv::inRange(hsv_img, lower, upper, mask);
		int height = mask.rows, width=mask.cols,i,j;
		for(i=0; i<height; ++i)
		{
			for(j=0; j<width; ++j)
			{
				if(i<360 || i>470 || j<500 ||j>800) mask.at<uchar>(i,j)=0;
			}
		}
	}



	


	cv::Mat mask_remove_outlier;
	remove_Outlier(mask, mask_remove_outlier, 20, 4);
	mask = mask_remove_outlier.clone();

	cv::bitwise_and(bgr_img,bgr_img,img_output,mask);
	
	// cout<<"mask.cols:"<<mask.cols
	// 	<<"\n"<<"mask.rows:"<<mask.rows
	// 	<<"\n"<<"mask.size:"<<mask.size
	// 	<<"\n"<<"hsv_mat_type:"<<hsv_img.type()
	// 	<<"\n"<<"hsv_mat_elemSize:"<<hsv_img.elemSize()
	// 	// <<"\n"<<"mask.end-begin"<<(int)(mask.end()-mask.begin())
	// 	<<endl;
}

//根据点云坐标切割点云
void cut_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out) //根据x值去除部分云
{
	pcl::ConstCloudIterator<pcl::PointXYZRGB> cloud_iterator(*cloud_in);
	cloud_iterator.reset();
	cloud_out->clear();

	while(cloud_iterator.isValid())
	{
		// if(cloud_iterator->x > 0.34  &&  cloud_iterator->x <0.6 )
		if(	
			// cloud_iterator->x > 0.30  &&  cloud_iterator->x <0.55 
			// && cloud_iterator->y >0.20 &&cloud_iterator->y < 0.40 

			cloud_iterator->x > 0.47  &&  cloud_iterator->x <0.57
			&& cloud_iterator->y >0.1 &&cloud_iterator->y < 0.22
		  )//没修正左臂时用的
		{
			cloud_out->points.push_back(*cloud_iterator);
		}
		++cloud_iterator;
	}

	cloud_out->height=1;
    cloud_out->width=cloud_out->points.size();
    cloud_out->is_dense=false;
}

//复制多层点云
void add_cloud_layers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out, double translation_x, double translation_y, double translation_z, int num_of_added_layers)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_out = *cloud_in;

	for(int i=0; i<num_of_added_layers; ++i)
	{
		Eigen::Affine3f T_translation = Eigen::Affine3f::Identity();
		T_translation.translation() << (i+1)*translation_x, (i+1)*translation_y, (i+1)*translation_z;	// 三个数分别对应X轴、Y轴、Z轴方向上的平移
		pcl::transformPointCloud(*cloud_in, *cloud_temp, T_translation);
		*cloud_out += *cloud_temp;
	}
    
} 

//二值化图像取反
void maks_img_invert(cv::Mat & src, cv::Mat & output)
{
	int height = src.rows, width=src.cols,i,j;
	output = src.clone();
	for(i=0; i<height; ++i)
		for(j=0; j<width; ++j)
		{
			if(src.at<uchar>(i,j)) output.at<uchar>(i,j)=0;
			else output.at<uchar>(i,j)=255;
		}
}


//根据二值图像、原始png图像提取点云。要求三者像素同，需要原始的png图形和点云。
void img_cloud_match(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_origin,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_output,
						cv::Mat png_origin,
						cv::Mat mask)
{
	int point_num, rows, cols, i, j;
	rows = mask.rows;
	cols = mask.cols;


	if(mask.size() != png_origin.size())	cout<<"png & mask 尺寸不同！！！"<<endl;
	

	cloud_output->clear();


	point_num = -1;
	for(i=0; i<rows; ++i)
	{
		for(j=0; j<cols; ++j)
		{
			if(png_origin.at<uchar>(i, j*4+3)==255)
			{
				++point_num;
				if(mask.at<uchar>(i,j)==255)
				{
					cloud_output->points.push_back(cloud_origin->points[point_num]);
				}

			}
			
		}
	}
	

	

	cloud_output->height=1;
    cloud_output->width=cloud_output->points.size();
    cloud_output->is_dense=false;
	// cout<<"rows * cols: "<<rows<<" * "<<cols<<'\n'
	// 	<<"cloud_read width:"<<cloud_origin->width<<'\n'
	// 	<<"cloud_output width:"<<cloud_output->width<<'\n'
	// 	<<"png_a255 pixels:"<<point_num+1<<'\n'
	// 	<<endl;
	// cv::imshow("img_cloud_match mask",mask);
}


//将几个点云放一起分别染色，用于观察
void combin_clouds(	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_1,
					const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_2,
					const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input_3,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_1 = *cloud_input_1;
	*cloud_2 = *cloud_input_2;
	*cloud_3 = *cloud_input_3;


	cloud_combined->clear();
	

	for(int i=0; i<cloud_1->size(); ++i)
	{
		cloud_1->points[i].r = 0;
		cloud_1->points[i].g = 255;
		cloud_1->points[i].b = 0;
	}

	for(int i=0; i<cloud_2->size(); ++i)
	{
		cloud_2->points[i].r = 255;
		cloud_2->points[i].g = 0;
		cloud_2->points[i].b = 0;
	}

	for(int i=0; i<cloud_3->size(); ++i)
	{
		cloud_3->points[i].r = 0;
		cloud_3->points[i].g = 0;
		cloud_3->points[i].b = 255;
	}

	*cloud_combined = *cloud_1 + *cloud_2  + *cloud_3;
}


//校正z轴
void z_axis_adjust(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_before_zCorrect)
{
	//提取平面、校正z轴
	Eigen::VectorXf coefficient;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Vector3d plane_normal, rotation_shaft;
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	double theta;

	plane_extracte_RANSAC(cloud_input, coefficient, 0.002, cloud_plane_before_zCorrect);
	plane_normal<< coefficient[0],coefficient[1], coefficient[2];
    plane_normal.normalize();
	if(plane_normal[2]<0) plane_normal = -plane_normal;
    cout<<"plane normal:\n"<<plane_normal<<endl;	

	rotation_shaft = Eigen::Vector3d::UnitZ().cross(plane_normal);
	theta = asin(rotation_shaft.norm());
	rotation_shaft.normalize();
	cout<<"rotation_shaft: \n"<<rotation_shaft
		<<"\ntheta: "<<theta<<endl;

	transform.rotate(Eigen::AngleAxisd(-theta, rotation_shaft));
	pcl::transformPointCloud(*cloud_input, *cloud_temp, transform);
	*cloud_input = *cloud_temp;
	cout<< "z_axis_adjust T:\n"<<transform.matrix()<<endl;

	return;
}


void smooth_cloud_MLS(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input,
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output)//未完成
{

	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> filter;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;  //定义搜索方法
	filter.setInputCloud(cloud_input);    //设置输入点云
	// filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);  //增加密度较小区域的密度对于holes的填补却无能为力，具体方法要结合参数使用
	// filter.setUpsamplingRadius(0.005);//上采样半径
	// filter.setUpsamplingStepSize(0.005);//上采样步长
	filter.setSearchRadius(0.01);// 用于拟合的K近邻半径。在这个半径里tconst进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
	filter.setPolynomialOrder(3);  //拟合曲线的阶数, <=1仅仅依靠切线。
	filter.setComputeNormals(false);  // 是否存储点云的法向量，true 为存储，false 不存储
	filter.setSearchMethod(kdtree); //设置搜索方法
	filter.process(*cloud_output); //处理点云并输出
	cout<<"smooth_cloud_MLS size: "<<cloud_input->size()<<"->"<<cloud_output->size()<<endl;
}




//2023.04.03、14、19实验数据，采用手动画线提取=================================================
int pc_line_manual()
{
    uint64 time_temp = CurrentTime_ms();

	pose_init(T_world_camera);


	Eigen::Matrix<double,4,4> T_z_adjust = Eigen::Matrix<double,4,4>::Identity();
	

	// cv::Mat line_img = cv::imread(path_output_cloud+"line"+to_string(step_n)+".jpg",cv::ImreadModes::IMREAD_UNCHANGED);
	cv::Mat line_img = cv::imread(path_output_cloud+to_string(step_n)+"line"+".jpg",cv::ImreadModes::IMREAD_UNCHANGED);
	cv::Mat bgra_img = cv::imread(path_read_clod+"img"+to_string(pcd_num)+".png",cv::ImreadModes::IMREAD_UNCHANGED);
	cv::Mat line_mask = cv::Mat::zeros(line_img.size(),CV_8U);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);


	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_read_clod+"pc"+to_string(pcd_num)+".pcd",*cloud_read)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}


    //点云坐标变换->世界坐标
	time_temp = CurrentTime_ms();
	pcl::transformPointCloud(*cloud_read,*cloud_output,T_world_camera);
	*cloud_read = *cloud_output;
	cout<<"坐标变换用时 ms: "<<CurrentTime_ms()-time_temp <<endl;


	//校准z轴
	time_temp = CurrentTime_ms();
	{
		//为了确保各步骤坐标系相同，先对step0进行平面拟合z轴校准（下面两行代码），然后旧采用该校准的变换矩阵校准所有step
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_before_zCorrect(new pcl::PointCloud<pcl::PointXYZRGB>);
		// z_axis_adjust(cloud_read, cloud_plane_before_zCorrect);
		//230414
		// T_z_adjust<<    0.999637, -0.000153749,   -0.0269463,            0,
		// 				-0.000153749,     0.999935,   -0.0114091 ,           0,
		// 				0.0269463 ,   0.0114091  ,   0.999572 ,           0,
		// 				0 ,           0  ,          0 ,           1;
		//2023.04.19
		// T_z_adjust<<   	0.985597 ,0.000786428 ,   0.169111     ,      0,
		// 				0.000786428 ,   0.999957, -0.00923354 ,          0,
  		// 				-0.169111,  0.00923354,    0.985554   ,        0,
        //   				0        ,   0      ,     0   ,        1;
		//230427
		T_z_adjust <<	0.983635, -0.00284484,    0.180149 ,          0,
						-0.00284484 ,   0.999505 ,   0.031317  ,      0,
  						-0.180149,   -0.031317,    0.983141 ,         0,
          				0 ,          0   ,        0     ,      1;
		pcl::transformPointCloud(*cloud_read,*cloud_output,T_z_adjust);
		*cloud_read = *cloud_output;
		cout<<"校准z轴用时 ms: "<<CurrentTime_ms()-time_temp <<endl;
	}

	for(int i=0; i < line_mask.rows; ++i)
	{
		for(int j=0; j<line_mask.cols; ++j)
		{
			if(line_img.at<uchar>(i,j*3) + line_img.at<uchar>(i,j*3+1) + line_img.at<uchar>(i,j*3+2) > 255)
				line_mask.at<uchar>(i,j) = 255;
		}
	}

	img_cloud_match(cloud_read, cloud_line, bgra_img, line_mask);

	if(!cloud_read->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(step_n)+"read.pcd",*cloud_read);
	else cout<<"cloud_read is empty!!!"<<endl;

	if(!cloud_line->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(step_n)+"line.pcd",*cloud_line);
	else cout<<"cloud_line is empty!!!"<<endl;





	if(!line_mask.empty())	cv::imwrite(path_output_cloud+"img"+to_string(step_n)+"_line_mask.jpg",line_mask);
	else cout<<"line_mask is empty!!!"<<endl;

	if(!line_img.empty())	cv::imwrite(path_output_cloud+"img"+to_string(step_n)+"_line_read.jpg",line_img);
	else cout<<"line_img is empty!!!"<<endl;

	return 0;
}

//2023.04.14实验数据，采用手动edge提取=================================================
int pc_edge_manual()
{
	
	cv::Mat edge_img = cv::imread(path_output_cloud+to_string(step_n)+"edge"+".jpg",cv::ImreadModes::IMREAD_UNCHANGED);
	cv::Mat bgra_img = cv::imread(path_read_clod+"img"+to_string(pcd_num)+".png",cv::ImreadModes::IMREAD_UNCHANGED);
	cv::Mat line_mask = cv::Mat::zeros(edge_img.size(),CV_8U);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>);


	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/wyh/Desktop/Kinect_data/pcd230427/output_manual_xymoved/"+to_string(step_n)+"read.pcd",*cloud_read)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}

	for(int i=0; i < line_mask.rows; ++i)
	{
		for(int j=0; j<line_mask.cols; ++j)
		{
			if(edge_img.at<uchar>(i,j*3) + edge_img.at<uchar>(i,j*3+1) + edge_img.at<uchar>(i,j*3+2) > 255)
				line_mask.at<uchar>(i,j) = 255;
		}
	}

	img_cloud_match(cloud_read, cloud_edge, bgra_img, line_mask);


	if(!cloud_edge->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(step_n)+"edge.pcd",*cloud_edge);
	else cout<<"cloud_edge is empty!!!"<<endl;


	return 0;
}

//手动修正xy轴
int xy_axis_adjust()
{
	string path_cloud_read = "/home/wyh/Desktop/Kinect_data/pcd230403/output_manual/";
	string path_output = "/home/wyh/Desktop/Kinect_data/pcd230403/output_manual_xyadjusted/";
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.rotate(Eigen::AngleAxisd(-4.28915332882/180.0*M_PI, Eigen::Vector3d::UnitZ()));
	cout<<transform.matrix()<<endl;

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_cloud_read+to_string(step_n)+"read.pcd",*cloud_read)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file read.pcd\n");
		return(-1);
	}
	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_cloud_read+to_string(step_n)+"line.pcd",*cloud_line)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file line.pcd\n");
		return(-1);
	}


	//点云坐标变换->世界坐标
	pcl::transformPointCloud(*cloud_read,*cloud_output,transform);
	*cloud_read = *cloud_output;

	pcl::transformPointCloud(*cloud_line,*cloud_output,transform);
	*cloud_line = *cloud_output;




	if(!cloud_read->empty())    pcl::io::savePCDFile(path_output+to_string(step_n)+"read.pcd",*cloud_read);
	else cout<<"cloud_read is empty!!!"<<endl;

	if(!cloud_line->empty())    pcl::io::savePCDFile(path_output+to_string(step_n)+"line.pcd",*cloud_line);
	else cout<<"cloud_line is empty!!!"<<endl;

	return 0;
}


//手动更改各轴排序及原点位置，2023.04.03、14数据  
int axis_move()
{
	// string path_cloud_read = "/home/wyh/Desktop/Kinect_data/pcd230403/output_manual_xyadjusted/";
	// string path_output = "/home/wyh/Desktop/Kinect_data/pcd230403/output_manual_xymoved/";
	string path_cloud_read = "/home/wyh/Desktop/Kinect_data/pcd230427/line_manual/";
	string path_output = "/home/wyh/Desktop/Kinect_data/pcd230427/output_manual_xymoved/";
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);

	Eigen::Matrix<double,4,4> Ttrans = Eigen::Matrix<double,4,4>::Identity();
	Eigen::Affine3d Trot_z = Eigen::Affine3d::Identity();
	Eigen::Matrix<double,4,4> Trot_xyz = Eigen::Matrix<double,4,4>::Identity();
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
	//2023.04.04
	// Ttrans << 	1,	0,	0,	0.046 - 0.5706845686436,
	// 			0,	1,	0,	0.084 - 0.1147939979061,
	// 			0,	0,	1,	0.004 - (-0.10581),
	// 			0,	0,	0,	1;
	
	// Trot << 	0,	1,	0,	0,
	// 			0,	0,	1,	0,
	// 			1,	0,	0,	0,
	// 			0,	0,	0,	1;
	// T = Trot * Ttrans;

	//2023.04.14
	// Ttrans << 	1,	0,	0,	-0.4936315,
	// 			0,	1,	0,	-0.2975675,
	// 			0,	0,	1,	0.1549385,
	// 			0,	0,	0,	1;

	// Trot_z.rotate(Eigen::AngleAxisd(-0.8821237663499399/180.0*M_PI, Eigen::Vector3d::UnitZ()));
	
	
	// Trot_xyz << 	0,	1,	0,	0,
	// 				0,	0,	1,	0,
	// 				1,	0,	0,	0,
	// 				0,	0,	0,	1;

	// T = Trot_xyz * Trot_z * Ttrans;

	//2023.04.19
	// Ttrans << 	1,	0,	0,	-0.4027095,
	// 			0,	1,	0,	-0.219482,
	// 			0,	0,	1,	0.11486,
	// 			0,	0,	0,	1;

	// Trot_z.rotate(Eigen::AngleAxisd(4.05499559387/180.0*M_PI, Eigen::Vector3d::UnitZ()));
	
	
	// Trot_xyz << 	0,	1,	0,	0,
	// 				0,	0,	1,	0,
	// 				1,	0,	0,	0,
	// 				0,	0,	0,	1;

	// T = Trot_xyz * Trot_z * Ttrans;

	//2023.04.27
	Ttrans << 	1,	0,	0,	-0.408468,
				0,	1,	0,	-0.2122645,
				0,	0,	1,	0.1368625,
				0,	0,	0,	1;

	Trot_z.rotate(Eigen::AngleAxisd(4.69334679077/180.0*M_PI, Eigen::Vector3d::UnitZ()));
	
	
	Trot_xyz << 	0,	1,	0,	0,
					0,	0,	1,	0,
					1,	0,	0,	0,
					0,	0,	0,	1;

	T = Trot_xyz * Trot_z * Ttrans;
	
	cout<<"T:\n"<<T<<endl;

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_cloud_read+to_string(step_n)+"read.pcd",*cloud_read)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file read.pcd\n");
		return(-1);
	}
	// if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_cloud_read+to_string(step_n)+"line.pcd",*cloud_line)==-1)//*打开点云文件
	// {
	// 	PCL_ERROR("Couldn't read file line.pcd\n");
	// 	return(-1);
	// }


	//点云坐标变换->世界坐标
	pcl::transformPointCloud(*cloud_read,*cloud_output,T);
	*cloud_read = *cloud_output;

	pcl::transformPointCloud(*cloud_line,*cloud_output,T);
	*cloud_line = *cloud_output;




	if(!cloud_read->empty())    pcl::io::savePCDFile(path_output+to_string(step_n)+"read.pcd",*cloud_read);
	else cout<<"cloud_read is empty!!!"<<endl;

	if(!cloud_line->empty())    pcl::io::savePCDFile(path_output+to_string(step_n)+"line.pcd",*cloud_line);
	else cout<<"cloud_line is empty!!!"<<endl;

	return 0;
}

//pc_to_mesh.cpp 的“主函数”
int pc_to_mesh()
{
    uint64 time_temp = CurrentTime_ms();
    cout<<"========================== Point cloud to mesh start ========================="<<endl;

	cv::Mat bgr_extracted, mask_color, skeleton_img, mask_inv, mask_closed, mask_opened, mask_edge;
	cv::Mat bgra_img = cv::imread(path_read_clod+"img"+to_string(pcd_num)+".png",cv::ImreadModes::IMREAD_UNCHANGED);
	
	
	pose_init(T_world_camera);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_extracted(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_read_clod+"pc"+to_string(pcd_num)+".pcd",*cloud_read)==-1)//*打开点云文件
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	cout<<"open pcd ms: "<<CurrentTime_ms()-time_temp <<endl;

	time_temp = CurrentTime_ms();

	// 预处理
	// PrePorcessingOfPointCloud(cloud_read,cloud_output,0.005,0.01,27);
	// *cloud_read = *cloud_output;
	// double lenth_pre=0.005;
	// for(int i=0;i<50;i++)
	// {cloud_color_extracted
	// 	PrePorcessingOfPointCloud(cloud_read,cloud_output,0.005,0.01,27);
    // 	pcl::io::savePCDFile(path_output_cloud+"pc"+to_string(pcd_num)+"_read.pcd",*cloud_read);

	// }
	
    
	//点云坐标变换->世界坐标
	time_temp = CurrentTime_ms();
	pcl::transformPointCloud(*cloud_read,*cloud_output,T_world_camera);
	*cloud_read = *cloud_output;
	cout<<"坐标变换用时 ms: "<<CurrentTime_ms()-time_temp <<endl;


	//校准z轴
	time_temp = CurrentTime_ms();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_before_zCorrect(new pcl::PointCloud<pcl::PointXYZRGB>);
	z_axis_adjust(cloud_read, cloud_plane_before_zCorrect);
	cout<<"校准z轴用时 ms: "<<CurrentTime_ms()-time_temp <<endl;


	//从png提取特定颜色点云，得到提取后的 bgr和二值化图像(伴随着二维坐标切除)
	time_temp = CurrentTime_ms();
	png_color_extract(bgra_img,bgr_extracted,mask_color,color_name);
	img_cloud_match(cloud_read, cloud_color_extracted, bgra_img, mask_color);
	cout<<"提取颜色 ms: "<<CurrentTime_ms()-time_temp <<endl;

	// //提取颜色后下采样
	// time_temp = CurrentTime_ms();
	// DownSample(cloud_color_extracted,cloud_output, 0.002);
	// *cloud_color_extracted = *cloud_output;
	// cout<<"提取颜色后下采样 ms: "<<CurrentTime_ms()-time_temp <<endl;


	//切除 cloud_color_extracted 外围点
	time_temp = CurrentTime_ms();
	cut_cloud(cloud_color_extracted,cloud_output);
	*cloud_color_extracted = *cloud_output;
	// OutlierFilter(cloud_color_extracted, cloud_output, 0.005, 5);
	OutlierFilter(cloud_color_extracted, cloud_output, 0.005, 15);//前面没有下采样时的参数
	// OutlierFilter(cloud_color_extracted, cloud_output, 0.011, 50);
	*cloud_color_extracted = *cloud_output;
	cout<<"切除外围及离群点 ms: "<<CurrentTime_ms()-time_temp <<endl;
	

	//2023.04.03实验---------------------------------------------------------------------------------------------------------------------------------------
	// //提取marker point
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marker_centers(new pcl::PointCloud<pcl::PointXYZRGB>);
	// cv::Mat mask_marker_centers;
	// points_marker_extract_mask(mask_color,mask_marker_centers,bgra_img);
	// img_cloud_match(cloud_read, cloud_marker_centers, bgra_img, mask_marker_centers);
	// cout<<"marker centers:"<<endl;
	// for(int i=0; i<cloud_marker_centers->size(); ++i)
	// {
	// 	cout<<cloud_marker_centers->points[i].x<<'\t'<<cloud_marker_centers->points[i].y<<'\t'<<cloud_marker_centers->points[i].z<<endl;
	// }
	// if(!cloud_marker_centers->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(step_n)+"centers.pcd",*cloud_marker_centers);
	// else cout<<"cloud_marker_centers is empty!!!"<<endl;

	//提取标记线
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Mat mask_line;
	line_extract_mask(mask_color,mask_line,bgra_img);
	img_cloud_match(cloud_read, cloud_line, bgra_img, mask_line);
	if(!cloud_line->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(step_n)+"line.pcd",*cloud_line);
	else cout<<"cloud_line is empty!!!"<<endl;
	//--------------------------------------------------------------------------------------------------------------------------------------------


	//取切干净的软组织点云最高点附近点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(color_name == RED)
	{
		time_temp = CurrentTime_ms();
		top_points(cloud_color_extracted, cloud_top_points, 0.004, 0.001);
		cout<<"抓取点提取 ms: "<<CurrentTime_ms()-time_temp <<endl;
	}


	//从点云根据RGB信息提取
	// color_depend_extract(cloud_read,cloud_output,COLOR_NAME(BLUE));
	// *cloud_read = *cloud_output;
	

    
		
	

	// cut_cloud(cloud_read, cloud_output);
	// *cloud_read = *cloud_output;

	// add_cloud_layers(cloud_read,cloud_output,0,0,-0.005,3);

	// vector<pcl::ModelCoefficients> LinesCoefficients;
	// fitMultipleLines(cloud_output, LinesCoefficients);

	
	//=============== 开闭运算、骨架提取 ================================
	// cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
	// cv::Mat element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	// cv::morphologyEx(mask_color, mask_opened, cv::MORPH_OPEN, element_open);
	// // cv::morphologyEx(mask_opened, mask_closed, cv::MORPH_CLOSE, element_close);
	// cv::morphologyEx(mask_color, mask_closed, cv::MORPH_CLOSE, element_close);
	
	// maks_img_invert(mask_closed, mask_inv);
	// // maks_img_invert(mask_opened, mask_inv);
	// skeleton_extract(mask_inv,skeleton_img);
	//================================================================
	
	//显示图像
	// cv::imshow("bgr_mask", bgr_extracted);
    // cv::imshow("mask_color", mask_color);
    // // cv::imshow("mask_inv", mask_inv);
    // cv::imshow("mask_closed", mask_closed);
    // cv::imshow("mask_opened", mask_opened);
    // cv::imshow("skeleton", skeleton_img);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_of_part(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	if(color_name == RED)
	{
		//法向量计算  test
		time_temp = CurrentTime_ms();
		// *cloud_color_extracted = *cloud_read;//realsense
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		normal_estimation(cloud_color_extracted,normals,0.01, 1, path_output_cloud, pcd_num);
		cout<<"法向量计算 ms: "<<CurrentTime_ms()-time_temp <<endl;


		// pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);
		// pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> filter;
		// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree;  //定义搜索方法
		// filter.setInputCloud(cloud_color_extracted);    //设置输入点云
		// // filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);  //增加密度较小区域的密度对于holes的填补却无能为力，具体方法要结合参数使用
		// // filter.setUpsamplingRadius(0.005);//上采样半径
		// // filter.setUpsamplingStepSize(0.005);//上采样步长
		// filter.setSearchRadius(0.01);// 用于拟合的K近邻半径。在这个半径里tconst进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
		// filter.setPolynomialOrder(3);  //拟合曲线的阶数, <=1仅仅依靠切线。
		// filter.setComputeNormals(true);  // 是否存储点云的法向量，true 为存储，false 不存储
		// filter.setSearchMethod(kdtree); //设置搜索方法
		// filter.process(*smoothedCloud); //处理点云并输出
		
		// pcl::Normal normal_temp;
		// normals->clear();
		// for(int i=0; i<smoothedCloud->size(); ++i)
		// {
		// 	normal_temp.normal_x = smoothedCloud->points[i].normal_x;
		// 	normal_temp.normal_y = smoothedCloud->points[i].normal_y;
		// 	normal_temp.normal_z = smoothedCloud->points[i].normal_z;
		// 	normals->points.push_back(normal_temp);
		// }
		// normals->height=1;
		// normals->width=normals->points.size();
		// normals->is_dense=false;
		// cout<<"Sizes: "<< cloud_color_extracted->size()<<"\t"<<smoothedCloud->size()<<"\t"<<normals->size()<<endl;

		//根据法向量提取点云边界点和非边界点
		time_temp = CurrentTime_ms();
		
		boundary_normals(cloud_color_extracted, normals, cloud_boundary, cloud_no_boundary, 0.005, 20.0, 1, 0.011, 50);
		cout<<"边界提取 ms: "<<CurrentTime_ms()-time_temp <<endl;



		// //边缘提取 RGB
		// cv::Mat edge_binary = bgr_extracted.clone();
		// edge_extract_binary(mask_color, mask_edge, edge_binary);
		// img_cloud_match(cloud_read, cloud_edge, bgra_img, mask_edge);


		//将几个点云染色放一起观察
		time_temp = CurrentTime_ms();
		combin_clouds(cloud_boundary, cloud_top_points, cloud_no_boundary, cloud_combined);
		cout<<"Combine ms: "<<CurrentTime_ms()-time_temp <<endl;


		//提取内圈的最高点
		time_temp = CurrentTime_ms();
		top_points(cloud_no_boundary, cloud_top_of_part);
		cout<<"提取内圈最高点 ms: "<<CurrentTime_ms()-time_temp <<endl;

		//将内圈点染色加入combine
		*cloud_output = *cloud_top_of_part;
		for(int i=0; i<cloud_output->size(); ++i)
		{
			cloud_output->points[i].r=255;
			cloud_output->points[i].g=0;
			cloud_output->points[i].b=0;
		}
		*cloud_combined = *cloud_combined + *cloud_top_of_part;


		//保存下采样前的
		// if(!cloud_no_boundary->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"no_boundary_before_DownS.pcd",*cloud_no_boundary);
		// else cout<<"cloud_no_boundary is empty!!!"<<endl;


		// smooth_cloud_MLS(cloud_no_boundary,cloud_output);
		// *cloud_no_boundary = *cloud_output;

		// smooth_cloud_MLS(cloud_no_boundary,cloud_output);
		// *cloud_no_boundary = *cloud_output;


		//对将要保存的点云图进行下采样
		time_temp = CurrentTime_ms();
		DownSample(cloud_no_boundary,cloud_output, 0.003);
		*cloud_no_boundary = *cloud_output;
		// DownSample(cloud_no_boundary,cloud_output, 0.0045);
		// *cloud_no_boundary = *cloud_output;

		DownSample_FPS(cloud_no_boundary, cloud_output, 400);
		*cloud_no_boundary = *cloud_output;
		cout<<"下采样 no boundary ms: "<<CurrentTime_ms()-time_temp <<endl;

		
		



		//均匀采样（取球）
		// pcl::UniformSampling<pcl::PointXYZRGB> form;   // 创建均匀采样对象
		// form.setInputCloud(cloud_no_boundary);                  //设置输入点云
		// form.setRadiusSearch(0.005);                //设置半径大小,单位:m
		// form.filter(*cloud_output);                  //执行滤波处理
		// *cloud_no_boundary = *cloud_output;

		time_temp = CurrentTime_ms();
		double l_temp=0.001;
		do
		{
			DownSample(cloud_top_of_part,cloud_output, l_temp);
			l_temp += 0.0005;
		}
		while(cloud_output->size()>5);
		
		*cloud_top_of_part = *cloud_output;
		cout<<"下采样 top of part ms: "<<CurrentTime_ms()-time_temp <<endl;


		//将内圈点染色加入combine
		for(int i=0; i<cloud_top_of_part->size(); ++i)
		{
			cloud_top_of_part->points[i].r=0;
			cloud_top_of_part->points[i].g=0;
			cloud_top_of_part->points[i].b=255;
		}
		pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"combine_temp.pcd",*cloud_no_boundary + *cloud_top_of_part);

	}

	//====================== 保存图像 ==================================== 
	string color_name_str ="";
	switch (color_name)
	{
	case WHITE:	color_name_str="WHITE";	break;
	case RED: 	color_name_str="RED";	break;
	case BLUE: 	color_name_str="BLUE";	break;
	case BLACK: color_name_str="BLACK";	break;
	case RED_WHITE: color_name_str="R_W";	break;
	case YELLOW: color_name_str="Y";	break;
	case ALL: color_name_str="A"; break;
	default:	break;
	}

	//点云保存------------------------------
	// if(!cloud_read->empty()) 	pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"_read.pcd",*cloud_read);
	// else cout<<"cloud_read is empty!!!"<<endl;

	// if(!cloud_color_extracted->empty())   pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"_color.pcd",*cloud_color_extracted);
    // else cout<<"cloud_color_extracted is empty!!!"<<endl;

	// if(!cloud_plane_before_zCorrect->empty())   pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"_plane_before_z.pcd",*cloud_plane_before_zCorrect);
    // else cout<<"cloud_plane_before_zCorrect is empty!!!"<<endl;

	// // pcl::io::savePCDFile(path_output_cloud+"pc_to_mesh"+to_string(pcd_num)+".pcd",*cloud_output);
    // // if(!cloud_edge->empty())	pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"_egde.pcd",*cloud_edge);
	// // else cout<<"cloud_edge is empty!!!"<<endl;

	// if(!cloud_boundary->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"boundary.pcd",*cloud_boundary);
    // else cout<<"cloud_boundary is empty!!!"<<endl;

    // if(!cloud_no_boundary->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"no_boundary.pcd",*cloud_no_boundary);
    // else cout<<"cloud_no_boundary is empty!!!"<<endl;

	// if(!cloud_plane_before_zCorrect->empty())   pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"plane.pcd",*cloud_plane_before_zCorrect);
    // else cout<<"cloud_plane_before_zCorrect is empty!!!"<<endl;

	// if(!cloud_combined->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"combined.pcd",*cloud_combined);
	// else cout<<"cloud_combined is empty!!!"<<endl;

	// if(!cloud_top_points->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"top_of_all.pcd",*cloud_top_points);
	// else cout<<"cloud_top_points is empty!!!"<<endl;
	
	// if(!cloud_top_of_part->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(pcd_num)+"top_of_part.pcd",*cloud_top_of_part);
	// else cout<<"cloud_top_of_part is empty!!!"<<endl;
    

	// 另一种保存编号（用于不同拉伸步骤连续保存）结合 main 中的for循环开启
	{
		// string save_path_n_output = "/home/wyh/Desktop/Kinect_data/pcd230223/2-output/";
		string save_path_n_output = path_output_cloud;

		if(!cloud_read->empty()) 	pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"_read.pcd",*cloud_read);
		else cout<<"cloud_read is empty!!!"<<endl;

		if(!cloud_color_extracted->empty())   pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+color_name_str+".pcd",*cloud_color_extracted);
		else cout<<"cloud_color_extracted is empty!!!"<<endl;

		// pcl::io::savePCDFile(save_path_n_output+"pc_to_mesh"+to_string(step_n)+".pcd",*cloud_output);
		// if(!cloud_edge->empty())	pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"_egde.pcd",*cloud_edge);
		// else cout<<"cloud_edge is empty!!!"<<endl;

		// if(!cloud_boundary->empty())    pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"boundary.pcd",*cloud_boundary);
		// else cout<<"cloud_boundary is empty!!!"<<endl;

		


		if(!cloud_no_boundary->empty())    pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"no_boundary.pcd",*cloud_no_boundary);
		else cout<<"cloud_no_boundary is empty!!!"<<endl;

		if(!cloud_top_points->empty())    pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"top_of_all.pcd",*cloud_top_points);
		else cout<<"cloud_top_points is empty!!!"<<endl;
		

		if(!cloud_combined->empty())    pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"combined.pcd",*cloud_combined);
		else cout<<"cloud_combined is empty!!!"<<endl;


		
		if(!cloud_top_of_part->empty())    pcl::io::savePCDFile(save_path_n_output+to_string(step_n)+"top_of_part.pcd",*cloud_top_of_part);
		else cout<<"cloud_top_of_part is empty!!!"<<endl;





		if(!bgr_extracted.empty())	cv::imwrite(save_path_n_output+"img"+to_string(step_n)+"_"+color_name_str+".jpg",bgr_extracted);
		else cout<<"bgr_extracted is empty!!!"<<endl;

		if(!mask_color.empty())	cv::imwrite(save_path_n_output+"img"+to_string(step_n)+"_"+color_name_str+"_mask.jpg",mask_color);
		else cout<<"mask_color is empty!!!"<<endl;

	}
    


	
	//图像保存-------------------------- 
	// if(!bgr_extracted.empty())	cv::imwrite(path_output_cloud+"img"+to_string(pcd_num)+"_"+color_name_str+".jpg",bgr_extracted);
	// else cout<<"bgr_extracted is empty!!!"<<endl;

	// if(!mask_color.empty())			cv::imwrite(path_output_cloud+"img"+to_string(pcd_num)+"_"+color_name_str+"_mask.jpg",mask_color);
	// else cout<<"mask_color is empty!!!"<<endl;

	// if(!skeleton_img.empty())	cv::imwrite(path_output_cloud+"img"+to_string(pcd_num)+"_"+color_name_str+"_skeleton.jpg",skeleton_img);
	// else cout<<"skeleton_img is empty!!!"<<endl;

	// if(!edge_binary.empty())	cv::imwrite(path_output_cloud+"img"+to_string(pcd_num)+"_"+color_name_str+"_edge_binary.jpg",edge_binary);
	// else cout<<"edge_binary is empty!!!"<<endl;
	
	// if(!mask_edge.empty())	cv::imwrite(path_output_cloud+"img"+to_string(pcd_num)+"_"+color_name_str+"_mask_edge.jpg",mask_edge);
	// else cout<<"mask_edge is empty!!!"<<endl;

    cout<<"finish ms: "<<CurrentTime_ms()-time_temp<<endl;
	// pause();
	cv::waitKey(10);


    cout<<"=============================================================================="<<endl;

    return 0;
}


        


