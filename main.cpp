#include <time.h>
#include <thread>
#include <cstdint>
#include <vector>
#include <sys/stat.h>
#include <direct.h>
#include <io.h>
#include <unordered_map>
#include <condition_variable>

#include "main.h"
#include "hv_cam_dahua.h"
#include "routes_solver.h"
#include "sigma_controller.h"
#include "convert_image.h"
#include "detection.h"

#define _CRT_SECURE_NO_WARNINGS

using namespace std;
using namespace cv;

// Multithread mutex and condition variable.
mutex detect_nms_mutex;
condition_variable detect_nms_cv;
bool is_detect_done = false;
bool is_nms_over = false;

// Global shared data.
CSigmaController m_Sigmal;
HV_CAM_DAHUA midcamera = HV_CAM_DAHUA();
vector<vector<float>> gen_scan_routes;
ObjectDetector detector = ObjectDetector();
int scan_samples;
vector<vector<float>> detected_objs_voltages;
string dir_name;
bool is_save_img = false;
bool is_save_results = true;
float pixel_error = 0.14; // The nms threshold for filter nearest samples.
int scan_time = 1;

// The detected objs;
vector<ResampleCenters> total_objs;

// Thread functions for processing and helper functions for nms.
vector<vector<float>> SolveScanRoutes(int sample_nums, int max_range, int min_range);
//void SolveScanRoutes(int sample_nums, int max_range, int min_range, vector<vector<float>>& gen_scan_routes);
void SendXYSignal();
//bool SendSolvedXYSignal(vector<vector<float>>& solved_scan_voltages);
void SendSolvedXYSignal();
void InitializeComPort();
bool ConnectSettingCamera();
bool CloseCamera();
void InitializeDetector(String cfg_file, String weights_file);
int GenerateResample(vector<vector<float>>& detected_objs_voltages, DetectedResults& detected_results, vector<ResampleCenters>& resample_centers);
void NMSResamples(vector<ResampleCenters>& resample_centers, vector<ResampleCenters>& filtered_centers, int total_samples);
void CreateFolder(string dir_name);
void threadProcessImage();
void threadFilterResamples();
vector<vector<float>> RemoveNearstPoints(vector<vector<float>>& gen_scan_routes);
void SendSave(vector<ResampleCenters>& final_objs);

int main()
{
	auto beg_t = chrono::system_clock::now();
	auto sys_time_begin = chrono::high_resolution_clock::now();
	cv::String cfg_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov4.cfg";
	cv::String weights_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov4.weights";
	scan_samples = 133;
	dir_name = "1208_final_133_random_scan" + to_string(scan_samples);
	CreateFolder(dir_name);

	/* 1. Initialize and warm up the detector.*/
	InitializeDetector(cfg_file, weights_file);

	/* 2. Initialize the COM_DA contorller.*/
	InitializeComPort();

	/* 3. Connecting and setting the camera.*/
	bool is_camera_open = ConnectSettingCamera();

	/* 4. Compute the scanning path given the samples.Using the Route Planning algorithms.*/

	//vector<vector<float>> gen_scan_routes = SolveScanRoutes(scan_samples);

	gen_scan_routes = SolveScanRoutes(scan_samples, 500, -500);
	//SolveScanRoutes(scan_samples, 500, -500, gen_scan_routes);

	/* 5. Processing here. Create ConvertImage object. Grab and convert the image.
		Create 1) send (x,y) thread and 2) image convert therad. */
	ConvertImage image_convertor = ConvertImage();
	image_convertor.num_samples = scan_samples;
	chrono::steady_clock::time_point begin_time2 = chrono::steady_clock::now();



	//thread send_com_thread(SendXYSignal);
	thread send_com_thread(SendSolvedXYSignal);
	thread convert_image_thread(&ConvertImage::process_image, image_convertor);
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	
	thread nms_filter_thread(threadFilterResamples);
	thread img_process_thread(threadProcessImage);

	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}

	chrono::steady_clock::time_point send_end_time3 = chrono::steady_clock::now();
	cout << "The whole process is  get clock (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(send_end_time3 - begin_time2).count() / 1000 << endl;

	send_com_thread.join();
	convert_image_thread.join();
	nms_filter_thread.join();
	img_process_thread.join();

	while (is_nms_over == false)
	{
		this_thread::sleep_for(chrono::milliseconds(2));
	}

	//pixel_error = 0.12;
	scan_time += 1;
	cout << ">>>>>>>>>>>>>>begin second part."<<scan_samples << endl;
	cout << "The total img list: " << img_list1.size() << " total nums:" << scan_samples << " , " << img_mat_list.empty() << endl;
	image_convertor.num_samples = scan_samples;
	//thread send_com_thread(SendXYSignal);
	thread send_com_thread1(SendSolvedXYSignal);
	thread convert_image_thread1(&ConvertImage::process_image, image_convertor);
	//this_thread::sleep_for(chrono::milliseconds(5));
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread nms_filter_thread1(threadFilterResamples);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread img_process_thread1(threadProcessImage);




	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}
	send_com_thread1.detach();
	convert_image_thread1.detach();
	nms_filter_thread1.join();
	img_process_thread1.join();


	while (is_nms_over == false)
	{
		this_thread::sleep_for(chrono::milliseconds(10));
	}

	//pixel_error = 0.20;
	scan_time += 1;
	cout << ">>>>>>>>>>>>>>begin second part." << scan_samples << endl;
	cout << "The total img list: " << img_list1.size() << " total nums:" << scan_samples << " , " << img_mat_list.empty() << endl;
	image_convertor.num_samples = scan_samples;
	//thread send_com_thread(SendXYSignal);
	thread send_com_thread2(SendSolvedXYSignal);
	thread convert_image_thread2(&ConvertImage::process_image, image_convertor);
	//this_thread::sleep_for(chrono::milliseconds(5));
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	//thread send_com_thread2(SendSolvedXYSignal);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread nms_filter_thread2(threadFilterResamples);
	//this_thread::sleep_for(chrono::milliseconds(5));
	thread img_process_thread2(threadProcessImage);

	while (img_list1.size() < scan_samples)
	{
		this_thread::sleep_for(chrono::microseconds(2000));
	}
	send_com_thread2.detach();
	convert_image_thread2.detach();
	nms_filter_thread2.join();
	img_process_thread2.join();




	auto end_t = chrono::system_clock::now();
	chrono::steady_clock::time_point end_detect = chrono::steady_clock::now();
	auto sys_time_end = chrono::high_resolution_clock::now();

	std::cout << "Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_detect - begin_time2).count() / 1000.0 << endl;
	std::cout << "Sys Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_t - beg_t).count() / 1000.0 << endl;
	std::cout << "Highresol Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;


	cout << img_list1.size() << img_mat_list.size() << endl;

	cout << "Before filtered: " << endl;

	for (auto i : total_objs)
	{
		cout << "before filtered: " << i.num_samples << ",  " << i.center_x << ", " << i.center_y << ", " << i.confidence << endl;
	}
	vector<ResampleCenters> final_objs;
	NMSResamples(total_objs, final_objs, 100);
	for (auto j : final_objs)
	{
		cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;
	}

	if (is_save_results)
	{
		vector<vector<float>> results_voltage(final_objs.size(), vector<float>(2, 0));
		for (int index = 0; index < results_voltage.size(); index++)
		{
			//std::cout << "[" << scan_samples[scan_index[index]][0] << ", " << scan_samples[scan_index[index]][1] << "] ";
			results_voltage[index][0] = final_objs[index].center_x;
			results_voltage[index][1] = final_objs[index].center_y;
		}

		image_convertor.num_samples = final_objs.size();
		thread convert_image_thread3(&ConvertImage::process_image, image_convertor);

		gen_scan_routes = results_voltage;
		this_thread::sleep_for(chrono::milliseconds(5));
		//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
		thread send_com_thread3(SendSolvedXYSignal);
		thread send_save_thread3(SendSave, ref(final_objs));
		convert_image_thread3.join();
		send_com_thread3.join();
		send_save_thread3.join();
	}

	/* 6. Disconnect and close the camera.*/
	img_list1.clear();
	bool is_close = CloseCamera();



	return 0;
}

// NMS filter out the same samples in different frames, using abs error. (Chebyshv distance.)
// filter_out abs_error(sample_i, sample_j)<0.06 .
void NMSResamples(vector<ResampleCenters>& resample_centers, vector<ResampleCenters>& filtered_centers, int total_samples)
{
	int len = resample_centers.size();
	vector<bool> is_checked(len, false);
	int remain_nums = 0;
	for (int i = 0; i < len; i++)
	{
		bool is_filtered = false;
		if (i < len - 1 && is_checked[i] == false) // i=len-1, j=len, the vector will access failed. Segment fault.
		{
			for (int j = i + 1; j < len; j++)
			{
				float abs_x = abs(resample_centers[i].center_x - resample_centers[j].center_x);
				float abs_y = abs(resample_centers[i].center_y - resample_centers[j].center_y);
				double abs_error = double(abs_x) + double(abs_y);
				if (abs_error < pixel_error)
				{
					//if (resample_centers[i].num_samples > resample_centers[j].num_samples)
					if (resample_centers[i].confidence > resample_centers[j].confidence)
					{
						is_checked[j] = true;
					}
					else
					{
						is_checked[i] = true;
						is_filtered = true;
						break;
					}
				}
			}
		}
		if (is_checked[i] == false && !is_filtered)
		{
			filtered_centers.push_back(resample_centers[i]);
			remain_nums += resample_centers[i].num_samples;
		}
	}
	int len_filtered = filtered_centers.size();
	int avg_sample = (total_samples - remain_nums) / len_filtered;
	int last_samples = (total_samples - remain_nums) - avg_sample * (len_filtered - 1);
	for (int i = 0; i < len_filtered; i++)
	{
		if (i < len_filtered - 1)
		{
			filtered_centers[i].num_samples += avg_sample;
		}
		else
		{
			filtered_centers[i].num_samples += last_samples;
		}
	}
}

// Generate Resample points (x,y) around the center of detected objs.
// num_resample_i = total_samples* conf_i / total_conf.
int GenerateResample(vector<vector<float>>& detected_objs_voltages, DetectedResults& detected_results, vector<ResampleCenters>& resample_centers)
{
	//cout << ">>>>>>>>In the Generate resamples center " << endl;
	int nums_detected = detected_results.detected_box.size();
	int newly_scan_samples = 0.8 * scan_samples;
	int global_scan_samples = scan_samples - newly_scan_samples;
	float total_confs = 0.001;
	for (auto i : detected_results.detected_conf)
	{
		total_confs += i;
	}

	int previous_id = detected_results.detected_ids[0];
	int voltage_index = 0;
	ResampleCenters temp_centers;
	for (int i = 0; i < nums_detected; i++)
	{
		int scan_id = detected_results.detected_ids[i];
		if (scan_id != previous_id)
		{
			voltage_index += 1;
			previous_id = scan_id;
		}
		float samples_i_x = detected_objs_voltages[voltage_index][0], samples_i_y = detected_objs_voltages[voltage_index][1];
		float sample_conf_i = detected_results.detected_conf[i];
		Rect sample_box_i = detected_results.detected_box[i];
		float complement_i_x = ((sample_box_i.x + sample_box_i.width / 2.0) - 112.0) * 0.002; // 224 pixel <--> 0.45
		float complement_i_y = ((sample_box_i.y + sample_box_i.height / 2.0) - 112.0) * 0.002;
		float real_x = samples_i_x + complement_i_x, real_y = samples_i_y + complement_i_y;
		int newly_samples_i = newly_scan_samples * sample_conf_i / total_confs;
		temp_centers.center_x = real_x, temp_centers.center_y = real_y;
		temp_centers.confidence = sample_conf_i, temp_centers.num_samples = newly_samples_i;
		//cout << "box information: " << sample_box_i.x << " " << sample_box_i.y << endl;
		//cout << "id: " << scan_id << "sample i x: " << samples_i_x << ", y: " << samples_i_y << ", complement_i_x: " << complement_i_x << ", complement_i_y:" << complement_i_y << endl;

		resample_centers.emplace_back(temp_centers);
	}
	return newly_scan_samples;
}

// Send the simualtion (x, y) signals to COM port.
void SendXYSignal()
{
	std::cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	while (1)
	{
		if (count == 100)
		{
			std::cout << "break the send thread" << endl;
			break;
		}
		if (count % 4 == 0)
		{
			m_Sigmal.on_rotate1_usb(-2.6, 1.7);
		}
		else if (count % 4 == 1)
		{
			m_Sigmal.on_rotate1_usb(-2.8, 2.0);
		}
		else if (count % 4 == 2)
		{
			m_Sigmal.on_rotate1_usb(-2.93, 1.8);
		}
		else
		{
			m_Sigmal.on_rotate1_usb(-3.25, 1.71);
		}

		count++;
		cout << "send signals: " << count << endl;
		this_thread::sleep_for(chrono::microseconds(1000));
	}
	std::cout << "cout the x y is overed!" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	std::cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
}

// Send the given solved (x, y) signals to COM port.
void SendSolvedXYSignal()
{
	std::cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	for (int i = 0; i < gen_scan_routes.size(); i++)
	{
		count++;
		//cout << "sent " << count << " :[" << solved_scan_voltages[i][0] << "," << solved_scan_voltages[i][1] << "]" << endl;
		m_Sigmal.on_rotate1_usb(gen_scan_routes[i][0], gen_scan_routes[i][1]);
		this_thread::sleep_for(chrono::microseconds(3000));
		//while (!is_callback_ok)
		//{
		//	this_thread::sleep_for(chrono::microseconds(100));
		//}
		//is_callback_ok = false;
	}
	std::cout << ">>>> End the send com thread" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	std::cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
	//return true;
}

// Given the smaples data, solve the scanning routes.
vector<vector<float>> SolveScanRoutes(int sample_nums, int max_range, int min_range)
{
	//chrono::steady_clock::time_point begin_time_ortools = chrono::steady_clock::now();
	int num_samples = sample_nums;
	//// 1. Generate samples;
	vector<vector<int>> scan_samples = operations_research::GenerateSamples(num_samples, max_range, min_range);
	// compute the start and end points of each scan.
	int* assign_points = operations_research::CalculateStartEndPoints(scan_samples);
	//std::cout << "original samples" << endl;
	//for (auto temp_sample : scan_samples)
	//{

	//	std::cout << "[" << temp_sample[0] << ", " << temp_sample[1] << "] ";
	//}
	//std::cout << "Scan begin points: " << scan_samples[assign_points[0]][0] << ", " << scan_samples[assign_points[0]][1] << endl;
	//std::cout << "Scan end points: " << scan_samples[assign_points[1]][0] << ", " << scan_samples[assign_points[1]][1] << endl;
	//// 2. Compute distance matrix;
	vector<vector<int64_t>> chebyshev_dist = operations_research::ComputeChebyshevDistanceMatrix(num_samples, scan_samples);
	//// 3. Initial the Datamodel;

	operations_research::DefineStartDataModel StartEndData;
	StartEndData.distance_matrix = chebyshev_dist;
	StartEndData.num_vehicles = 1;
	StartEndData.starts.emplace_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[0] });
	StartEndData.ends.emplace_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[1] });
	vector<int> scan_index = operations_research::Tsp(StartEndData);

	vector<vector<float>> scan_voltages(num_samples, vector<float>(2, 0));
	for (int index = 0; index < scan_index.size(); index++)
	{
		//std::cout << "[" << scan_samples[scan_index[index]][0] << ", " << scan_samples[scan_index[index]][1] << "] ";
		scan_voltages[index][0] = scan_samples[scan_index[index]][0] / 100.0;
		scan_voltages[index][1] = scan_samples[scan_index[index]][1] / 100.0;
	}
	//for (auto i : scan_index)
	//{
	//	std::cout << i << "-> ";
	//}
	//std::cout << " " << endl;
	//chrono::steady_clock::time_point end_time_ortools = chrono::steady_clock::now();
	//std::cout << "Waitting for save (ms): " << chrono::duration_cast<chrono::microseconds>(end_time_ortools - begin_time_ortools).count() / 1000.0 << endl;
	//std::cout << "====>>>> 4. Solve the scanning routes succeed." << endl;
	return scan_voltages;
}

// Initilize and connect the COM port.
void InitializeComPort()
{
	bool sigma1_status = m_Sigmal.OpenController(COM_PORT);//COM1对应第一个控制器
	Sleep(50);
	if (sigma1_status == TRUE)
	{
		// 1%,0.1ms,0.1us
		m_Sigmal.InitialSystem_STEP(DA_STEP_PCT, DA_STEP_TIME, DA_STEP_DLY);//步长初始化
																			// 5000 us
		m_Sigmal.InitialSystem_EXP(DA_EXP);//曝光初始化
										   // 10000 us
		m_Sigmal.InitialSystem_CYT(DA_CYCLE);//周期初始化
											 // 100 us
		m_Sigmal.InitialSystem_PW(DA_PULSE_W);//脉宽初始化

		m_Sigmal.on_rotate1_usb(0, 0);
		std::cout << "====>>>> 2. Initialize the COM port succeed!" << endl;
	}
	else
	{
		std::cout << "====>>>> 2. Initialize the COM port failed !!!" << endl;
	}
	Sleep(50);
	m_Sigmal.on_rotate1_usb(-4.51, -4.51);
	Sleep(50);
}

// Connecting and setting the camera.
bool ConnectSettingCamera()
{
	bool is_ok = midcamera.scanCameraDevice();
	is_ok = midcamera.linkCamera(MID_CAM_SERIAL_NUMBER);
	is_ok = midcamera.openCamera();
	is_ok = midcamera.setCameraAcquisitionFrameRate(MID_ACQUISITION_FRAME_RATE);
	is_ok = midcamera.setCameraROI(MID_OFFSET_X, MID_OFFSET_Y, MID_WIDTH, MID_HEIGHT);
	is_ok = midcamera.setCameraExposureMode(MID_EXPOSURE_MODE);
	is_ok = midcamera.setCameraExposureTime(MID_EXPOSURE_TIME);
	is_ok = midcamera.setCameraBalanceWihteAuto(MID_BALANCEWHITE_MODE);
	is_ok = midcamera.setCameraReverseXY(MID_REVERSE_X, MID_REVERSE_Y);
	is_ok = midcamera.setCameraExposureGain(MID_GAIN_RAW);
	is_ok = midcamera.setCameraBrightness(MID_BRIGHTNESS);
	is_ok = midcamera.setCameraTriggerMode(MID_TRIGGE_MODE);
	is_ok = midcamera.setCameraImageType(MID_PIXEL_FORMAT);
	// create stream -> register callback -> start grabbing;
	is_ok = midcamera.createStream();
	is_ok = midcamera.registerCallback();
	is_ok = midcamera.cameraStartGrabbing();
	if (is_ok)
	{
		std::cout << "====>>>> 3. Connecting and setting the camera succeed." << endl;
	}
	else
	{
		std::cout << "====>>>> 3. Connecting and setting the camera failed !!!" << endl;
	}
	return is_ok;
}

// Close the camera.
bool CloseCamera()
{
	bool is_ok;
	is_ok = midcamera.cameraStopGrabbing();
	is_ok = midcamera.unregisterCallback();
	is_ok = midcamera.closeCamera();
	if (is_ok)
	{
		std::cout << "====>>>> 6. Close the camera succeed." << endl;
	}
	else
	{
		std::cout << "====>>>> 6. Close the camera failed !!!" << endl;
	}
	return is_ok;
}

// Initilize the detector.
void InitializeDetector(String cfg_file, String weights_file)
{
	detector.is_save_img = is_save_img;
	detector.initialization(cfg_file, weights_file, 224, 224);
	// Warm up the detector.
	cv::Mat temp_img = cv::imread("1122all_new44.jpg");
	bool temp_result;
	for (int i = 0; i < 1; i++)
	{
		temp_result = detector.inference(temp_img, 0);
	}
	std::cout << "====>>>> 1. Warm up the detector end!" << endl;

	// When warm up the detector, we need to clear the output of the confidence vector.
	detector.detected_results.detected_box.clear();
	detector.detected_results.detected_conf.clear();
	detector.detected_results.detected_ids.clear();
}

// Create folder to save image if no exist.
void CreateFolder(string dir_name)
{
	string folderPath = "C:\\CODEREPO\\DahuaGal\\Image\\" + dir_name;
	if (_access(folderPath.c_str(), 0) == -1)	//If file is not exist.
	{
		cout << "Dir is not exist, make dir: " << dir_name << endl;
		int no_use = _mkdir(folderPath.c_str());
	}
}

void threadProcessImage()
{
	lock_guard<mutex> lck(detect_nms_mutex);
	auto sys_time_begin = chrono::high_resolution_clock::now();
	cout << "!!!!!!!! begin the Image processing thread!!!!!!!!" << endl;
	Mat grab_img2;
	int real_img_count = 0;
	//chrono::steady_clock::time_point thread_begin = chrono::steady_clock::now();
	while (1)
	{
		while (!img_mat_list.empty())
		{
			//cout << " beggin to detect: " << real_img_count << " The image is remain: " << img_mat_list.size() << endl;
			grab_img2 = img_mat_list.wait_and_pop();
			//vector<vector<float>> detected_objs_voltages;
			grab_img2 = grab_img2(cv::Rect(20, 0, 224, 224));
			if (!grab_img2.empty())
			{
				bool is_detected = detector.inference(grab_img2, real_img_count);
				if (is_detected)
				{
					detected_objs_voltages.emplace_back(gen_scan_routes[real_img_count]);
					if (is_save_img)
					{
						std::stringstream filename2;
						time_t currenttime = time(0);
						char tmp[64];

						strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&currenttime));
						string time_str = tmp;

						filename2 << "./Image/" << dir_name << "/"<< scan_time <<"_" << time_str << "_" << real_img_count << "_" << gen_scan_routes[real_img_count][0] << "_" << gen_scan_routes[real_img_count][1] << ".jpg";
						cv::imwrite(filename2.str(), grab_img2);
						cout <<"frame id: "<<real_img_count<<"x, y: " << gen_scan_routes[real_img_count][0] <<", " << gen_scan_routes[real_img_count][1] << endl;

					}
				}
				real_img_count += 1;
			}
		}
		this_thread::sleep_for(chrono::microseconds(1000));
		//cout << "img list in the queue: " << img_list1.size() << endl;
		if (img_mat_list.empty() && real_img_count == scan_samples)
		{
			cout << "Image queue is empty" << endl;
			break;
		}
		else
		{
			cout << "Image mat queue is: " << img_mat_list.size() << ", total processing: " << real_img_count<<" total list: "<<scan_samples << endl;
		}
	}


	cout << "End detected thread!!!!!!" << endl;
	auto sys_time_end = chrono::high_resolution_clock::now();
	std::cout << "Thread Image process cost: Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;
	cout << "Total processing Img: " << scan_samples << ", FPS is: " << (scan_samples) * 1000000.0 / float(chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count()) << endl;
	
	is_detect_done = true;
	detect_nms_cv.notify_one();
	//cout << "Processing speed: " << real_img * 1000000.0 / float(chrono::duration_cast<chrono::microseconds>(end_detect - begin_detect).count()) << " FPS!" << endl;
}

void threadFilterResamples()
{
	cout << ">>>>>>>>>>>>Begin filter thread!!!!" << endl;
	unique_lock<mutex> lck(detect_nms_mutex);
	detect_nms_cv.wait(lck, [] {return is_detect_done; });

	auto sys_time_begin = chrono::high_resolution_clock::now();


	is_detect_done = false;
	vector<ResampleCenters> resample_centers;
	vector<ResampleCenters> filtered_centers;
	int total_sample_nums = 0;

	if (detected_objs_voltages.size() > 0)
	{
		total_sample_nums = GenerateResample(detected_objs_voltages, detector.detected_results, resample_centers);
		NMSResamples(resample_centers, filtered_centers, total_sample_nums);
		// Add the filtered sample into the final results.
		total_objs.insert(total_objs.end(), filtered_centers.begin(), filtered_centers.end());
	}

	//for (auto i : resample_centers)
	//{
	//	cout << "before filtered: " << i.num_samples << ",  " << i.center_x << ", " << i.center_y << ", " << i.confidence << endl;
	//}
	//for (auto j : filtered_centers)
	//{
	//	cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;
	//}
	lck.unlock();

	detector.detected_results.detected_box.clear();
	detector.detected_results.detected_conf.clear();
	detector.detected_results.detected_ids.clear();
	detected_objs_voltages.clear();
	img_list1.clear();
	gen_scan_routes.clear();

	int temp_i = 0;
	for (auto j : filtered_centers)
	{
		//cout << " begin new scan: " << endl;
		//cout << "after filtered: " << j.num_samples << ",  " << j.center_x << ", " << j.center_y << ", " << j.confidence << endl;

		vector<vector<float>> scan_routes_j = SolveScanRoutes(j.num_samples, int(50.0 * 1.0 / j.confidence), int(-50.0 * 1.0 / j.confidence));
		//SolveScanRoutes(j.num_samples, int(50.0 * 1.0 / j.confidence), int(-50.0 * 1.0 / j.confidence), gen_scan_routes);
		for (int i = 0; i < scan_routes_j.size(); i++)
		{
			scan_routes_j[i][0] += j.center_x;
			scan_routes_j[i][1] += j.center_y;
			//cout << gen_scan_routes[i][0] << ", " << gen_scan_routes[i][1] << endl;
		}
		gen_scan_routes.insert(gen_scan_routes.end(), scan_routes_j.begin(), scan_routes_j.end());
		temp_i += scan_routes_j.size();
		//cout <<"scan routes: " << scan_routes_j[scan_routes_j.size() - 1][0] << ", total_list: " << gen_scan_routes[temp_i - 1][0] << endl;
		//cout << "Gen scan routes: " << gen_scan_routes.size() << endl;
	}
	//cout << "End the NMS thread!!!!!!!!" << endl;

	//cout << ">>>>>>Before remove nearst points: " << endl;
	//for (auto i : gen_scan_routes)
	//{
	//	cout << i[0] << " " << i[1] << endl;
	//}

	vector<vector<float>> sample_removed = RemoveNearstPoints(gen_scan_routes);
	//cout << ">>>>>>After remove nearst points: " << endl;
	//for (auto j : sample_removed)
	//{
	//	cout << j[0] << " " << j[1] << endl;
	//}

	gen_scan_routes.clear();
	//copy(sample_removed.begin(), sample_removed.end(), gen_scan_routes.begin());
	//gen_scan_routes.insert(gen_scan_routes.end(), sample_removed.begin(), sample_removed.end());
	gen_scan_routes = sample_removed;
	//cout << "Copy scan after: " << gen_scan_routes.size() << sample_removed.size() << endl;
	scan_samples = gen_scan_routes.size();
	is_nms_over = true;

	resample_centers.clear();
	filtered_centers.clear();

	auto sys_time_end = chrono::high_resolution_clock::now();
	std::cout << "Thread NMS and filter cost: Total time (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(sys_time_end - sys_time_begin).count() / 1000.0 << endl;
}

vector<vector<float>> RemoveNearstPoints(vector<vector<float>>& gen_scan_routes)
{
	int len = gen_scan_routes.size();
	vector<bool> is_checked(len, false);
	vector<vector<float>> sparse_samples;
	//cout << "Begin to remove the samples: " << len << endl;
	for (int i = 0; i < len; i++)
	{
		bool is_filtered = false;
		if (i < len - 1 && is_checked[i] == false) // i=len-1, j=len, the vector will access failed. Segment fault.
		{
			for (int j = i + 1; j < len; j++)
			{
				float abs_x = abs(gen_scan_routes[i][0] - gen_scan_routes[j][0]);
				float abs_y = abs(gen_scan_routes[i][1] - gen_scan_routes[j][1]);
				double abs_error = double(abs_x) + double(abs_y);
				if (abs_error < 0.20)
				{
						is_checked[j] = true;
				}
			}
		}
		if (is_checked[i] == false && !is_filtered)
		{
			sparse_samples.emplace_back(gen_scan_routes[i]);
		}
	}
	//cout << "After to remove the samples: " << sparse_samples.size() << endl;
	return sparse_samples;
}

void SendSave(vector<ResampleCenters>& final_objs)
{
	cout << ">>>> Begin the send and save thread" << endl;
	Mat save_img;
	int i = 0;
	int img_save = 0;
	while (1)
	{
		while (!img_mat_list.empty())
		{

			save_img = img_mat_list.wait_and_pop();
			save_img = save_img(cv::Rect(20, 0, 224, 224));
			if (!save_img.empty())
			{
				std::stringstream filename2;
				time_t currenttime = time(0);
				char tmp[64];
				strftime(tmp, sizeof(tmp), "%Y%m%d_%H%M%S", localtime(&currenttime));
				string time_str = tmp;
				filename2 << "./Image/" << dir_name << "/" << "result" << "_" << time_str << "_" << final_objs[i].center_x << "_" << final_objs[i].center_y << ".jpg";
				cv::imwrite(filename2.str(), save_img);
				img_save += 1;
				i += 1;
			}
		}
		this_thread::sleep_for(chrono::microseconds(5000));
		//cout << "img list in the queue: " << img_list1.size() << endl;
		if (img_mat_list.empty() && img_save == final_objs.size())
		{
			break;
		}
	}
	cout << ">>>> End save img!!!" << endl;

}