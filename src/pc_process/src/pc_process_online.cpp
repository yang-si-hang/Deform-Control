#include "pc_process_online.hpp"


using namespace std;

//capture 的全局变量
extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;//读取的点云
extern cv::Mat image_cloud_mat;//depth_to_point_cloud 后，与点云对应的bgra图像，alpha用于表征像素点对应点有无有效深度信息（有没有对应的点云点）


//pc_to_mesh全局变量
extern Eigen::Matrix<double,4,4> T_world_camera ;
extern COLOR_NAME color_name ;

//用于从文件读取点云并处理
	//不同步数（不同变形程度）的路径
	string path_read_clod;// path_read_clod = temp_string + to_string(step_n)+'/';
	string path_output_cloud;//path_output_cloud = temp_string + to_string(step_n)+'/';
	int step_n;
	//各步的输出都存于一个路径时
	string path_save_online = "/home/wyh/Desktop/Kinect_data/pcdonline/";
	const int pcd_num = 2;//同一步同一组，同状态下拍摄的图像号
	// const int pcd_num = 10;


uint64 time_temp, time_loop_start;
uint16_t frame_num=0;

cv::Mat bgr_extracted, mask_color, skeleton_img, mask_inv, mask_closed, mask_opened, mask_edge, bgra_img;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_extracted(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);//用于各处点云处理的中转
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_before_zCorrect(new pcl::PointCloud<pcl::PointXYZRGB>);

vector<int64_t> save_times={}; 
int64_t save_times_sum=0;


void coordinate_init()
{
	cout<<"coordinate initing..........."<<endl;
	pose_init(T_world_camera);

	capture_once();

	//点云坐标变换->世界坐标
	time_temp = CurrentTime_ms();
	pcl::transformPointCloud(*cloud,*cloud_output,T_world_camera);
	*cloud = *cloud_output;
	cout<<"坐标变换用时 ms: "<<CurrentTime_ms()-time_temp <<endl;

	//提取平面、校正z轴
	Eigen::VectorXf coefficient;
	Eigen::Vector3d plane_normal, rotation_shaft;
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	double theta;

	plane_extracte_RANSAC(cloud, coefficient, 0.002, cloud_plane_before_zCorrect);
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
	T_world_camera = transform.matrix() * T_world_camera;
	// if(!cloud_plane_before_zCorrect->empty()) 	pcl::io::savePCDFile(path_save_online+"plane_before_zCorrect.pcd",*cloud_plane_before_zCorrect);
	// else cout<<"cloud_plane_before_zCorrect is empty!!!"<<endl;
	cout<<"..........coordinate finished"<<endl;

}


//pc_process_online.cpp 的“主函数”
int pc_process_online(bool online) 
{
    time_temp = CurrentTime_ms();
    cout<<"------- pc_process_online start ----------"<<endl;

	if(online) 
	{
		bgra_img=image_cloud_mat;
		cloud_read = cloud;
	}
	else
	{
		bgra_img = cv::imread(path_read_clod+"img"+to_string(pcd_num)+".png",cv::ImreadModes::IMREAD_UNCHANGED);
		

		if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(path_read_clod+"pc"+to_string(pcd_num)+".pcd",*cloud_read)==-1)//*打开点云文件
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd\n");
			return(-1);
		}
		cout<<"open pcd ms: "<<CurrentTime_ms()-time_temp <<endl;
	}
	
    
   

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



	//从png提取特定颜色点云，得到提取后的 bgr和二值化图像
	time_temp = CurrentTime_ms();
	png_color_extract(bgra_img,bgr_extracted,mask_color,color_name);
	img_cloud_match(cloud_read, cloud_color_extracted, bgra_img, mask_color);
	cout<<"提取颜色 ms: "<<CurrentTime_ms()-time_temp <<endl;

	//提取颜色后下采样
	time_temp = CurrentTime_ms();
	DownSample(cloud_color_extracted,cloud_output, 0.002);
	*cloud_color_extracted = *cloud_output;
	cout<<"提取颜色后下采样 ms: "<<CurrentTime_ms()-time_temp <<endl;


	//切除 cloud_color_extracted 外围点
	time_temp = CurrentTime_ms();
	cut_cloud(cloud_color_extracted,cloud_output);
	*cloud_color_extracted = *cloud_output;

	OutlierFilter(cloud_color_extracted, cloud_output, 0.005, 5);
	// OutlierFilter(cloud_color_extracted, cloud_output, 0.005, 15);//前面没有下采样时的参数
	// OutlierFilter(cloud_color_extracted, cloud_output, 0.011, 50);
	*cloud_color_extracted = *cloud_output;
	cout<<"切除外围及离群点 ms: "<<CurrentTime_ms()-time_temp <<endl;
	

	//取切干净的软组织点云最高点附近点云
	time_temp = CurrentTime_ms();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_points(new pcl::PointCloud<pcl::PointXYZRGB>);
	top_points(cloud_color_extracted, cloud_top_points, 0.004, 0.001);
	cout<<"抓取点提取 ms: "<<CurrentTime_ms()-time_temp <<endl;



	//从点云根据RGB信息提取
	// color_depend_extract(cloud_read,cloud_output,COLOR_NAME(BLUE));
	// *cloud_read = *cloud_output;
	

    
		
	


	

	//法向量计算  test
	time_temp = CurrentTime_ms();
	// *cloud_color_extracted = *cloud_read;//realsense
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	normal_estimation(cloud_color_extracted,normals,0.01, 0, path_save_online, frame_num);//save
	cout<<"法向量计算 ms: "<<CurrentTime_ms()-time_temp <<endl;



	//根据法向量提取点云边界点和非边界点
	time_temp = CurrentTime_ms();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
	boundary_normals(cloud_color_extracted, normals, cloud_boundary, cloud_no_boundary, 0.01, 20.0, 1, 0.006, 20);
	cout<<"边界提取 ms: "<<CurrentTime_ms()-time_temp <<endl;



	//将几个点云染色放一起观察
	time_temp = CurrentTime_ms();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>);
	combin_clouds(cloud_boundary, cloud_top_points, cloud_no_boundary, cloud_combined);
	cout<<"Combine ms: "<<CurrentTime_ms()-time_temp <<endl;


	//提取内圈的最高点
	time_temp = CurrentTime_ms();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_top_of_part(new pcl::PointCloud<pcl::PointXYZRGB>);
	top_points(cloud_no_boundary, cloud_top_of_part, 0.004, 0.01);
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


	//保存下采样前的 cloud_no_boundary
	// if(!cloud_no_boundary->empty())    pcl::io::savePCDFile(path_output_cloud+to_string(frame_num)+"no_boundary_before_DownS.pcd",*cloud_no_boundary);
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
	// if(!cloud_no_boundary->empty() || !cloud_top_of_part->empty())  pcl::io::savePCDFile(path_output_cloud+to_string(frame_num)+"combine_temp.pcd",*cloud_no_boundary + *cloud_top_of_part);
    // else cout<<"cloud_no_boundary & cloud_top_of_part are both empty!!!"<<endl;


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
	

	// 另一种保存编号（用于不同拉伸步骤连续保存）结合 main 中的for循环开启
	{
		time_temp = CurrentTime_ms();

		if(!cloud_read->empty()) 	pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"_read.pcd",*cloud_read);
		else cout<<"cloud_read is empty!!!"<<endl;

		// if(!cloud_color_extracted->empty())   pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"_color.pcd",*cloud_color_extracted);
		// else cout<<"cloud_color_extracted is empty!!!"<<endl;

		// pcl::io::savePCDFile(path_save_online+"pc_to_mesh"+to_string(frame_num)+".pcd",*cloud_output);
		// if(!cloud_edge->empty())	pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"_egde.pcd",*cloud_edge);
		// else cout<<"cloud_edge is empty!!!"<<endl;

		// if(!cloud_boundary->empty())    pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"boundary.pcd",*cloud_boundary);
		// else cout<<"cloud_boundary is empty!!!"<<endl;

		


		// if(!cloud_no_boundary->empty())    pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"no_boundary.pcd",*cloud_no_boundary);
		// else cout<<"cloud_no_boundary is empty!!!"<<endl;

		// if(!cloud_top_points->empty())    pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"top_of_all.pcd",*cloud_top_points);
		// else cout<<"cloud_top_points is empty!!!"<<endl;
		

		if(!cloud_combined->empty())    pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"combined.pcd",*cloud_combined);
		else cout<<"cloud_combined is empty!!!"<<endl;


		
		// if(!cloud_top_of_part->empty())    pcl::io::savePCDFile(path_save_online+to_string(frame_num)+"top_of_part.pcd",*cloud_top_of_part);
		// else cout<<"cloud_top_of_part is empty!!!"<<endl;

		save_times.push_back(CurrentTime_ms()-time_temp);
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

	// pause();
	// cv::waitKey(10);



    return 0;
}


int main(int argc, char  *argv[])
{
	cout<<"================================ pc_process_online START ==================================="<<endl;
	init_keyboard();
	camera_init();
	// pose_init(T_world_camera);
	coordinate_init();

	frame_num=0;
	while(true)
	{
		save_times.clear();
		save_times_sum=0;

		time_loop_start = CurrentTime_ms();
		if(kbhit()) break;
		cout<<"------------------------------------"<<endl;
		cout<<"FRAME: "<<frame_num<<endl;
		cout<<"------------------------------------"<<endl;
		
		capture_once();
		pc_process_online(true);



			std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);    // 无压缩png.
            compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
            compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);

            // cv::imwrite(path_save_online+"img"+to_string(frame_num)+".png",image_cloud_mat,compression_params);
            

		++frame_num;

		for(int i =0; i<save_times.size(); ++i)
		{
			save_times_sum += save_times[i];
		}
		cout<<"在线获取并处理点云 frame "<<frame_num<<" 用时："<<CurrentTime_ms()-time_loop_start - save_times_sum<<" + "<<save_times_sum<<"ms"<<endl;
		
	}

	camera_close();
	close_keyboard();

	cout<<"================================ pc_process_online FINISH ==================================="<<endl;
	return 0;
}