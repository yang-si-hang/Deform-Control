#pragma clang diagnostic ignored"-Wdangling-else"
 //C++
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
 //OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 //Kinect DK
#include <k4a/k4a.hpp>
//#include <k4a/k4a.h>
#include <math.h>
#include <sstream>

using namespace cv;
using namespace std;

 //宏
 //方便控制是否 std::cout 信息
#define DEBUG_std_cout 0


static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}

static void generate_point_cloud(const k4a::image depth_image,	const k4a_image_t xy_table,	k4a_image_t point_cloud,	int *point_count)
{
	int width = depth_image.get_width_pixels();
	int height = depth_image.get_height_pixels();
	//int height = k4a_image_get_height_pixels(depth_image);

	uint16_t *depth_data = (uint16_t *)(void *)depth_image.get_buffer();
	k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	*point_count = 0;
	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
			point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
			point_cloud_data[i].xyz.z = (float)depth_data[i];
			(*point_count)++;
		}
		else
		{
			point_cloud_data[i].xyz.x = nanf("");
			point_cloud_data[i].xyz.y = nanf("");
			point_cloud_data[i].xyz.z = nanf("");
		}
	}
}

static void write_point_cloud(const char *file_name, const k4a_image_t point_cloud, int point_count)
{
	int width = k4a_image_get_width_pixels(point_cloud);
	int height = k4a_image_get_height_pixels(point_cloud);

	k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

	 //save to the ply file
	std::ofstream ofs(file_name); // text mode first
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "element vertex"
		<< " " << point_count << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "end_header" << std::endl;
	ofs.close();

	std::stringstream ss;
	for (int i = 0; i < width * height; i++)
	{
		if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
		{
			continue;
		}

		ss << (float)point_cloud_data[i].xyz.x << " " << (float)point_cloud_data[i].xyz.y << " "
			<< (float)point_cloud_data[i].xyz.z << std::endl;
	}

	std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
	ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}



int main(int argc, char *argv[]) {
	/*

		找到并打开 Azure Kinect 设备
	*/
	//发现已连接的设备数

	const uint32_t device_count = k4a::device::get_installed_count();
	if (0 == device_count) {
		cout << "Error: no K4A devices found. " << endl;
		return -1;
	}
	else {
		std::cout << "Found " << device_count << " connected devices. " << std::endl;
		if (1 != device_count)// 超过1个设备，也输出错误信息。
		{
			std::cout << "Error: more than one K4A devices found. " << std::endl;
			return -1;
		}
		else// 该示例代码仅限对1个设备操作
		{
			std::cout << "Done: found 1 K4A device. " << std::endl;
		}
	}
	//打开（默认）设备
	k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
	std::cout << "Done: open device. " << std::endl;

	/*
		检索并保存 Azure Kinect 图像数据
	*/
	//配置并启动设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	//config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture
	device.start_cameras(&config);
	std::cout << "Done: start camera." << std::endl;

	//写入txt文件流
	ofstream rgb_out;
	ofstream d_out;
	ofstream ir_out;

	rgb_out.open("./rgb.txt");
	d_out.open("./depth.txt");
	ir_out.open("./ir.txt");

	rgb_out << "#  color images" << endl;
	rgb_out << "#  file: rgbd_dataset" << endl;
	rgb_out << "#  timestamp" << "    " << "filename" << endl;

	d_out << "#  depth images" << endl;
	d_out << "#  file: rgbd_dataset" << endl;
	d_out << "#  timestamp" << "    " << "filename" << endl;

	ir_out << "#  ir images" << endl;
	ir_out << "#  file: rgbd_dataset" << endl;
	ir_out << "#  timestamp" << "    " << "filename" << endl;

	rgb_out << flush;
	d_out << flush;
	//稳定化
	k4a::capture capture;
	int iAuto = 0;//用来稳定，类似自动曝光
	int iAutoError = 0;// 统计自动曝光的失败次数
	while (true) {
		if (device.get_capture(&capture)) {
			std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;

			//跳过前 n 个（成功的数据采集）循环，用来稳定
				if (iAuto != 30) {
					iAuto++;
					continue;
				}
				else {
					std::cout << "Done: auto-exposure" << std::endl;
					break;// 跳出该循环，完成相机的稳定过程
				}

		}
		else {
			std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
			if (iAutoError != 30) {
				iAutoError++;
				continue;
			}
			else {
				std::cout << "Error: failed to give auto-exposure. " << std::endl;
				return -1;
			}
		}
	}
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
	std::cout << "-----------------------------------" << std::endl;
	//从设备获取捕获
	k4a::image rgbImage;
	k4a::image depthImage;
	k4a::image irImage;
	k4a::image transformed_depthImage;

	cv::Mat cv_rgbImage_with_alpha;
	cv::Mat cv_rgbImage_no_alpha;
	cv::Mat cv_depth;
	cv::Mat cv_depth_8U;
	cv::Mat cv_irImage;
	cv::Mat cv_irImage_8U;

	while (true)
		for (size_t i = 0; i < 100; i++)
		{
			if (device.get_capture(&capture, std::chrono::milliseconds(0)))
				if (device.get_capture(&capture)) {
					//rgb
						//* Each pixel of BGRA32 data is four bytes.The first three bytes represent Blue, Green,
						//*and Red data.The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
						rgbImage = capture.get_color_image();
#if DEBUG_std_cout == 1
					std::cout << "[rgb] " << "\n"
						<< "format: " << rgbImage.get_format() << "\n"
						<< "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
						<< "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
						<< "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels()
						<< std::endl;
#endif

					//depth
						//* Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data.The unit of the data is in
						//* millimeters from the origin of the camera.
						depthImage = capture.get_depth_image();
#if DEBUG_std_cout == 1
					std::cout << "[depth] " << "\n"
						<< "format: " << depthImage.get_format() << "\n"
						<< "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
						<< "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
						<< "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels()
						<< std::endl;
#endif

					//ir
						//* Each pixel of IR16 data is two bytes of little endian unsigned depth data.The value of the data represents
						//* brightness.
						irImage = capture.get_ir_image();
#if DEBUG_std_cout == 1
					std::cout << "[ir] " << "\n"
						<< "format: " << irImage.get_format() << "\n"
						<< "device_timestamp: " << irImage.get_device_timestamp().count() << "\n"
						<< "system_timestamp: " << irImage.get_system_timestamp().count() << "\n"
						<< "height*width: " << irImage.get_height_pixels() << ", " << irImage.get_width_pixels()
						<< std::endl;
#endif

					//深度图和RGB图配准
						//Get the camera calibration for the entire K4A device, which is used for all transformation functions.
					k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);

					k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

					transformed_depthImage = k4aTransformation.depth_image_to_color_camera(depthImage);

					cv_rgbImage_with_alpha = cv::Mat(rgbImage.get_height_pixels(), rgbImage.get_width_pixels(), CV_8UC4,
						(void *)rgbImage.get_buffer());
					cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

					cv_depth = cv::Mat(transformed_depthImage.get_height_pixels(), transformed_depthImage.get_width_pixels(), CV_16U,
						(void *)transformed_depthImage.get_buffer(), static_cast<size_t>(transformed_depthImage.get_stride_bytes()));

					normalize(cv_depth, cv_depth_8U, 0, 256 * 256, NORM_MINMAX);
					cv_depth_8U.convertTo(cv_depth, CV_8U, 1);

					cv_irImage = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16U,
						(void *)irImage.get_buffer(), static_cast<size_t>(irImage.get_stride_bytes()));
					normalize(cv_irImage, cv_irImage_8U, 0, 256 * 256, NORM_MINMAX);
					cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1);

				
					//k4a::image xyzImage;
					//cv::Mat cv_xyzImage;// 16位有符号
					//cv::Mat cv_xyzImage_32F;// 32位float
					//点云================================================================================================================================================
					//	/*
					//		Each pixel of the xyz_image consists of three int16_t values, totaling 6 bytes. The three int16_t values are the X, Y, and Z values of the point.
					//		我们将为每个像素存储三个带符号的 16 位坐标值（以毫米为单位）。 因此，XYZ 图像步幅设置为 width * 3 * sizeof(int16_t)。
					//		数据顺序为像素交错式，即，X 坐标 – 像素 0，Y 坐标 – 像素 0，Z 坐标 – 像素 0，X 坐标 – 像素 1，依此类推。
					//		如果无法将某个像素转换为 3D，该函数将为该像素分配值 [0,0,0]。
					//	*/

					//xyzImage = k4aTransformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH);
					//cv_xyzImage = cv::Mat(xyzImage.get_height_pixels(), xyzImage.get_width_pixels(), CV_16SC3, (void *)xyzImage.get_buffer(), static_cast<size_t>(xyzImage.get_stride_bytes()));
					//cv_xyzImage.convertTo(cv_xyzImage_32F, CV_32FC3, 1.0 / 1000, 0);// 转为float，同时将单位从 mm 转换为 m.
					//cv::imshow("xyzimage", cv_xyzImage_32F);
				
			
					//show image
					cv::imshow("color", cv_rgbImage_no_alpha);
					cv::imshow("depth", cv_depth_8U);
					cv::imshow("ir", cv_irImage_8U);

					//save image
					double time_rgb = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
							rgbImage.get_device_timestamp()).count());

					std::string filename_rgb = std::to_string(time_rgb / 1000000) + ".png";

					double time_d = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
						depthImage.get_device_timestamp()).count());

					std::string filename_d = std::to_string(time_d / 1000000) + ".png";

					double time_ir = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
						irImage.get_device_timestamp()).count());
					std::string filename_ir = std::to_string(time_ir / 1000000) + ".png";
					imwrite("./rgb/" + filename_rgb, cv_rgbImage_no_alpha);
					imwrite("./depth/" + filename_d, cv_depth_8U);
					imwrite("./ir/" + filename_ir, cv_irImage_8U);

					
					//const int32_t TIMEOUT_IN_MS = 1000;
					//std::string file_name;
					//uint32_t device_count = 0;

					//k4a_device_t device1 = NULL;
					//k4a_device_configuration_t config1 = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
					//k4a_capture_t capture1 = NULL;
					//k4a_image_t depth_image = NULL;
					//k4a_calibration_t calibration1;

					k4a_image_t xy_table = NULL;
					k4a_image_t point_cloud = NULL;
					int point_count = 0;

					double time_point = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
						rgbImage.get_device_timestamp()).count());
					std::string filename_point = std::to_string(time_point / 1000000) + ".ply";
					//file_name = "./pointcloud.ply";

		/*			device_count1 = k4a_device_get_installed_count();

					if (device_count1 == 0)
					{
						printf("No K4A devices found\n");
						return 0;
					}*/

					/*if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
					{
						printf("Failed to open device\n");
					}*/


				/*	config1.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
					config1.camera_fps = K4A_FRAMES_PER_SECOND_30;*/
					
				/*	k4a_device_get_calibration(device1, config1.depth_mode, config1.color_resolution, &calibration1);*/

					k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						k4aCalibration.depth_camera_calibration.resolution_width,
						k4aCalibration.depth_camera_calibration.resolution_height,
						k4aCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
						&xy_table);

					create_xy_table(&k4aCalibration, xy_table);

					k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
						k4aCalibration.depth_camera_calibration.resolution_width,
						k4aCalibration.depth_camera_calibration.resolution_height,
						k4aCalibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
						&point_cloud);
				/*	k4a_device_start_cameras(device, &config);
					k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS);*/

					//depth_image = k4a_capture_get_depth_image(capture1);
					if (depthImage.get_size() == 0)
					{
						printf("Failed to get depth image from capture\n");
					}

					generate_point_cloud(depthImage, xy_table, point_cloud, &point_count);
					
					write_point_cloud(filename_point.c_str(), point_cloud, point_count);

				/*	k4a_image_release(depthImage);
					k4a_capture_release(capture);*/
					k4a_image_release(xy_table);
					k4a_image_release(point_cloud);
					//returnCode = 0;
					//k4a_device_close(device1);


					std::cout << "Acquiring!" << endl;

					//写入depth.txt, rgb.txt文件
					rgb_out << std::to_string(time_rgb / 1000000) << "    " << "rgb/" << filename_rgb << endl;
					d_out << std::to_string(time_d / 1000000) << "    " << "depth/" << filename_d << endl;
					ir_out << std::to_string(time_ir / 1000000) << "    " << "ir/" << filename_ir << endl;

					rgb_out << flush;
					d_out << flush;
					ir_out << flush;

					k4aTransformation.destroy();

					cv_rgbImage_with_alpha.release();
					cv_rgbImage_no_alpha.release();
					cv_depth.release();
					cv_depth_8U.release();
					cv_irImage.release();
					cv_irImage_8U.release();
					capture.reset();

					if (cv::waitKey() == 'q')
					{//按键采集，用户按下'q',跳出循环,结束采集
						std::cout << "----------------------------------" << std::endl;
						std::cout << "------------- closed -------------" << std::endl;
						std::cout << "----------------------------------" << std::endl;
						break;
					}
				}
				else
                {
					std::cout << "false: K4A_WAIT_RESULT_TIMEOUT." << std::endl;
				}
		}
	cv::destroyAllWindows();
	rgb_out << flush;
	d_out << flush;
	ir_out << flush;
	rgb_out.close();
	d_out.close();
	ir_out.close();


	// 释放，关闭设备
	rgbImage.reset();
	depthImage.reset();
	irImage.reset();
	capture.reset();
	device.close();
	

	return 1;
}
