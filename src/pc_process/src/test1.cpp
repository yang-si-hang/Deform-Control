#include "test1.hpp"


//=============================================================================
//=============================================================================
#define DEBUG_std_cout 
#define KBHIT //非阻塞键盘检测，用于跳出while
#define DEPTH_TO_CLOUD_AND_SAVE
// #define SEGMENTATION_PCL
// #define EUCILIDEAN_CLUSTER


//=============================================================================
//=============================================================================

using namespace std;


static string pcd_save_path = "/home/wyh/Desktop/Kinect_data/pcd_temp/";
static string png_save_path = pcd_save_path;
//=================== KBIT 相关函数 ==================
#ifdef KBHIT
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard()
{ 
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}
char readch()
{
    char ch;
    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}
int kbhit()
{
    char ch;
    int nread;
    if(peek_character != -1)
        return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1) 
    {
      peek_character = ch;
      return 1;
    }
return 0;
}
#endif//KBHIT




void depth_to_point_cloud(k4a_image_t & image_color, k4a_image_t & image_depth,
      k4a::transformation & k4aTransformation,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
     cv::Mat & mat_output)//生成RGB点云，及对应的四通道（png）图像，是否要用RGB来筛选，在其中添加
{
    cloud->clear();
    int image_color_width_pixels = k4a_image_get_width_pixels(image_color);
    int image_color_height_pixels = k4a_image_get_height_pixels(image_color);
    k4a::image transformed_depth_image = NULL;
    
    
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 
        image_color_width_pixels, image_color_height_pixels, 
        image_color_width_pixels * (int)sizeof(uint16_t));
    k4a::image image_point_cloud = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, 
        image_color_width_pixels, image_color_height_pixels,
         image_color_width_pixels * 3 * (int)sizeof(int16_t));
    k4aTransformation.depth_image_to_color_camera(image_depth, &transformed_depth_image);
    k4aTransformation.depth_image_to_point_cloud(transformed_depth_image,
             K4A_CALIBRATION_TYPE_COLOR, &image_point_cloud);

    k4a::image image_color_temp(image_color);
    int16_t *image_point_cloud_data = (int16_t *)(void *)image_point_cloud.get_buffer();
    uint8_t *image_color_data = image_color_temp.get_buffer();
    vector<uint8_t> image_cloud_vec;
    
    

    for(int i = 0; i < image_color_width_pixels * image_color_height_pixels; ++i)
    {
        pcl::PointXYZRGB point;
        
    
        point.z = image_point_cloud_data[3 * i + 2]/ 1000.0f;
        if (point.z == 0)  
        {
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            continue;
        }
        point.x = image_point_cloud_data[3 * i + 0]/ 1000.0f;
        point.y = image_point_cloud_data[3 * i + 1]/ 1000.0f;
        
        
        point.b = image_color_data[4*i+0];
        point.g = image_color_data[4*i+1];
        point.r = image_color_data[4*i+2];
        uint8_t alpha = image_color_data[4*1+3];
        // if(point.r+point.b+point.g > 200 && point.r-point.b < 25) continue;
        // if(point.r-point.b < 25) continue;
        // if(point.g-point.r < 5 || point.g-point.b < 5) continue;
        // if(point.b-point.r < 25 || point.b-point.g < 25)   //只取蓝色
        // if(abs(point.r-point.g)*3 > point.r-point.b ||  point.b>110 || point.r<95 || abs(point.r-point.g)>30 || point.g<95 || point.r-point.b<20)//
        // if((point.r<50 || point.r>100) || point.b+point.g>130)
        // {
        //     image_cloud_vec.push_back(0);
        //     image_cloud_vec.push_back(0);
        //     image_cloud_vec.push_back(0);
        //     image_cloud_vec.push_back(0);
        //     continue;
        // }
        if(point.r==0 && point.g==0 && point.b==0 && alpha==0) 
        {
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            image_cloud_vec.push_back(0);
            
            continue;
        }
        else
        {
            image_cloud_vec.push_back(image_color_data[4*i+0]);
            image_cloud_vec.push_back(image_color_data[4*i+1]);
            image_cloud_vec.push_back(image_color_data[4*i+2]);
            image_cloud_vec.push_back(255);
            cloud->points.push_back(point); //放到点云中
        }
        
        
    } 
    cloud->height=1;
    cloud->width=cloud->points.size();
    cloud->is_dense=false;

    cv::Mat mat_temp(image_cloud_vec);
    mat_output = mat_temp.reshape(4,image_color_height_pixels).clone();//将 vector 转化为4通道mat
    cout<<"mat_size: "<<mat_temp.size()<<endl;    
    cout<<"mat_output: "<<mat_output.size()<<endl;    
}


#ifdef SEGMENTATION_PCL
void segmentation_region_growing_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    std::vector <pcl::PointIndices> & clusters)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud (*cloud, *indices);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    // reg.setDistanceThreshold (100);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (10);
    reg.setMinClusterSize (600);

    
    reg.extract (clusters);

    // seg_cloud = reg.getColoredCloud ();
    
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud (colored_cloud);
    // while (!viewer.wasStopped ())
    // {
    //     Delay_ms(0.1);
    // }
}
#endif //SEGMENTATION_PCL

// int main(int argc, char const *argv[])
int main(int argc, char  *argv[])
{
    int64_t time_ms_temp=0;
    cv::Mat cv_color_with_alpha;
    cv::Mat cv_color_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_ir;
    cv::Mat cv_ir_8U;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = 
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_keypoints = 
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
    (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector <pcl::PointIndices>  clusters;
    // pcl::PointCloud <pcl::PointXYZRGB>::Ptr seg_cloud;

    #ifdef KBHIT
        init_keyboard();
    #endif

    cout<<"==========  test1 start  =========="<<endl;
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count==0)
    {
        cout<<"There is no camera!!!\nPlease connect the camera!!!"<<endl;
        return EXIT_FAILURE;
    }
    
    cout<<"camera_num:"<<device_count<<endl;

   
    
    k4a_device_t device1 = NULL; 
    
    
    while( K4A_RESULT_SUCCEEDED != k4a_device_open(0,&device1))
    {
        cout<<"Waiting for opening the device!!!"<<endl;
    }
    cout<<device1<<endl;

    /* // Get the size of the serial number
   {
        size_t serial_size = 0;
        k4a_device_get_serialnum(device1, NULL, &serial_size);

        // Allocate memory for the serial, then acquire it
        char *serial = (char*)(malloc(serial_size));
        k4a_device_get_serialnum(device1, serial, &serial_size);
        printf("Opened device1: %s\n", serial);
        free(serial);
        
        k4a_device_close(device1);
    }*/

    

    // Configure a stream of 4096x3072 BRGA color data at 15 frames per second
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps       = K4A_FRAMES_PER_SECOND_30;
    config.color_format     = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    // config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;
    // Start the camera 
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device1, &config))
    {
        printf("Failed to start device1\n");
        return EXIT_FAILURE;
    }
    else printf("camera_started\n");

    k4a::device device(device1);
    k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
    k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);
    
    

    // Capture a depth frame
    k4a_capture_t capture;
    k4a_image_t image_color, image_depth, image_ir;


    int iAuto = 0;//用来稳定，类似自动曝光
	int iAutoError = 0;// 统计自动曝光的失败次数
	while (true)
	{
        // Delay_ms(100);
		if (K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_capture(device1, &capture, 100))
		{
			std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
 
			// 跳过前 n 个（成功的数据采集）循环，用来稳定
			if (iAuto != 30)
			{
				++iAuto;
			}
			else
			{
				std::cout << "Done: auto-exposure" << std::endl;
				break;// 跳出该循环，完成相机的稳定过程
			}
		}
		else
		{
			std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
			if (iAutoError != 30)
			{
				++iAutoError;
			}
			else
			{
				std::cout << "Error: failed to give auto-exposure. " << std::endl;
				return EXIT_FAILURE;
			}
		}
	}
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
	std::cout << "-----------------------------------" << std::endl;
    

    
    int frame_num = 0;
    while (true)
    {
        time_ms_temp = CurrentTime_ms();

        ++frame_num;
        cout<<"Frame: "<<frame_num<<endl;
        #ifdef KBHIT
            if(kbhit()) break;
        #endif //KBHIT
        switch (k4a_device_get_capture(device1, &capture, 100))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a capture\n");
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                break;
        }

        image_color = k4a_capture_get_color_image(capture);
        image_depth = k4a_capture_get_depth_image(capture);
        image_ir = k4a_capture_get_ir_image(capture);
        #ifdef DEBUG_std_cout
            cout    << "format: " << k4a_image_get_format(image_depth) << "\n"
                    << "device_timestamp: " << k4a_image_get_device_timestamp_usec(image_depth)/1000000.0 << "\n"
                    << "system_timestamp: " << k4a_image_get_system_timestamp_nsec(image_depth)/1000000000.0 << "\n"
                    << "height*width: " << k4a_image_get_height_pixels(image_depth) 
                    << " x " << k4a_image_get_width_pixels(image_depth)
                    <<endl;

        #endif//DEBUG_std_cout

        cv_color_with_alpha = cv::Mat(k4a_image_get_height_pixels(image_color), k4a_image_get_width_pixels(image_color), CV_8UC4, (void *)k4a_image_get_buffer(image_color));
		cv::cvtColor(cv_color_with_alpha, cv_color_no_alpha, cv::COLOR_BGRA2BGR);
        cv_depth = cv::Mat(k4a_image_get_height_pixels(image_depth), k4a_image_get_width_pixels(image_depth), CV_16U, (void *)k4a_image_get_buffer(image_depth), static_cast<size_t>(k4a_image_get_stride_bytes(image_depth)));
        cv_depth.convertTo(cv_depth_8U, CV_8U, 255.0/(*max_element(cv_depth.begin<short>(),cv_depth.end<short>())) );
        cv_ir = cv::Mat(k4a_image_get_height_pixels(image_ir), k4a_image_get_width_pixels(image_ir), CV_16U, (void *)k4a_image_get_buffer(image_ir), static_cast<size_t>(k4a_image_get_stride_bytes(image_ir)));
		cv_ir.convertTo(cv_ir_8U, CV_8U, 255.0/(*max_element(cv_ir.begin<short>(),cv_ir.end<short>())) );
        cout<<"ir value: "<<*max_element(cv_ir.begin<short>(),cv_ir.end<short>())<<endl;
        cout<<"depth value: "<<*max_element(cv_depth.begin<short>(),cv_depth.end<short>())<<endl;
        // cout<<"=================   ir   =====================\n"<<cv_ir_8U<<"\n======================================"<<endl;
        // cout<<"=================   depth   =====================\n"<<cv_depth<<"\n======================================"<<endl;
        //显示
		cv::imshow("color", cv_color_no_alpha);
        cv::imshow("depth", cv_depth_8U);
        cv::imshow("ir",cv_ir_8U);//ir 是二值化图像
        cv::waitKey(5);
        cout << "--- capturing ---" << endl;
        cout <<  "capturing_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;

        int row = cv_color_no_alpha.rows;
        int col = cv_color_no_alpha.cols;
        cout << "row x col: "<<row<<" x "<<col<<endl;

        #ifdef DEPTH_TO_CLOUD_AND_SAVE
            time_ms_temp = CurrentTime_ms();
            
            cv::Mat image_cloud_mat;

            depth_to_point_cloud(image_color,image_depth,k4aTransformation,cloud,image_cloud_mat);
            cout <<  "dep_to_pcloud_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;

            cv::imshow("png",image_cloud_mat);
            cv::waitKey(5);

            time_ms_temp = CurrentTime_ms();
            pcl::io::savePCDFile(pcd_save_path+"pc"+to_string(frame_num)+".pcd",*cloud);
            
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);    // 无压缩png.
            compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
            compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);

            std::vector<int> params_temp;
            compression_params.push_back(cv::IMWRITE_JPEG2000_COMPRESSION_X1000);
            compression_params.push_back(0);
             
            cv::imwrite(png_save_path+"img"+to_string(frame_num)+".png",image_cloud_mat,compression_params);
            cv::imwrite(png_save_path+"ir_8U"+to_string(frame_num)+".png",cv_ir_8U);
            cv::imwrite(png_save_path+"depth_8U"+to_string(frame_num)+".png",cv_depth_8U);
            // cv::imwrite(png_save_path+"ir_8U"+to_string(frame_num)+".png",cv_ir,params_temp);
            // cv::imwrite(png_save_path+"depth_8U"+to_string(frame_num)+".png",cv_depth,params_temp);
            
            cout <<  "savePCDFile_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;
        #endif//DEPTH_TO_CLOUD_AND_SAVE

        #ifdef SEGMENTATION_PCL
            time_ms_temp = CurrentTime_ms();
            segmentation_region_growing_rgb(cloud,clusters);
            cout <<  "segment_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;
            
            time_ms_temp = CurrentTime_ms();
            int cluster_num =1;
            for(std::vector<pcl::PointIndices>::const_iterator it=clusters.begin();
                it!=clusters.end();
                it++)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                {
                    cloud_cluster->points.push_back(cloud->points[*pit]);
                    cloud_cluster->width = cloud_cluster->points.size();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;
                }
                pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd/pc_seg_"+to_string(frame_num)+"_"+to_string(cluster_num)+".pcd",*cloud_cluster);
                ++cluster_num;
            }
            cout <<  "save_seg_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;
        
        #endif //SEGMENTATION_PCL

        #ifdef EUCILIDEAN_CLUSTER
            time_ms_temp = CurrentTime_ms();
            // 创建用于提取搜索方法的kdtree树对象
            pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);//创建一个指向kd树搜索对象的共享指针

            //被分割出来的点云团（标号队列）
            std::vector<pcl::PointIndices> clusters;
            // 欧式分割器
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        
            ec.setClusterTolerance(0.003); // 判定半径
            ec.setMinClusterSize(10);      // 聚类的最小点数
            ec.setMaxClusterSize(20000);      // 聚类的最大点数
            //搜索策略树
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(clusters);
            std::cout << "欧式分割结束。   " << "聚类数量：" << clusters.size() << std::endl;
            cout <<  "segment_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;

            time_ms_temp = CurrentTime_ms();
            int cluster_num =1;
            for(std::vector<pcl::PointIndices>::const_iterator it=clusters.begin();
                it!=clusters.end();
                it++)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

                for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                {
                    cloud_cluster->points.push_back(cloud->points[*pit]);
                    cloud_cluster->width = cloud_cluster->points.size();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;
                }
                pcl::io::savePCDFile("/home/wyh/Desktop/Kinect_data/pcd/pc_seg_"+to_string(frame_num)+"_"+to_string(cluster_num)+".pcd",*cloud_cluster);
                ++cluster_num;
            }
            cout <<  "save_seg_cost:"<< CurrentTime_ms()-time_ms_temp<<" ms"<<endl;
        #endif//EUCILIDEAN_CLUSTER

    }
    
    
    

    // cout<<"......输入随机键继续"<<endl;
    // cin.get();
    #ifdef KBHIT
        close_keyboard();
    #endif
    cv::destroyAllWindows();


    k4a_image_release(image_color);
    k4a_image_release(image_depth);
    k4a_image_release(image_ir);
    k4a_capture_release(capture);



    
    // Shut down the camera 
    k4a_device_stop_cameras(device1);
    k4a_device_close(device1);
    cout<<"==========  test1 stop  =========="<<endl;
    return 0;
}
