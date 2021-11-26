#include <time.h>
#include <thread>
#include <cstdint>
#include <vector>

#include "routes_solver.h"
#include "hv_cam_dahua.h"
#include "sigma_controller.h"
#include "convert_image.h"
#include "detection.h"
using namespace std;
using namespace cv;

// DA Contorller Setting.
#define COM_PORT			3
#define DA_STEP_PCT			1		// %
#define DA_STEP_TIME		0.2		// ms
#define DA_STEP_DLY			0.1		// us
#define DA_EXP				800	// us    should be same with HC_EXP
#define DA_CYCLE			2500	// us
#define DA_PULSE_W			100		// us

// Camera Setting.
#define MID_CAM_SERIAL_NUMBER "6G03CFBPAK00001"
#define MID_OFFSET_X 228   // 00
#define MID_OFFSET_Y 160   // 00
#define MID_WIDTH 264   //720
#define MID_HEIGHT 224  //540
#define MID_EXPOSURE_MODE HV_CAM_DAHUA::EXPOSURE_AUTO_MODE_OFF
#define MID_BALANCEWHITE_MODE HV_CAM_DAHUA::BALANCEWHITE_AUTO_OFF
#define MID_EXPOSURE_TIME 800
#define MID_REVERSE_X false
#define MID_REVERSE_Y false
#define MID_GAIN_RAW 10
#define MID_BRIGHTNESS 90
#define MID_TRIGGE_MODE HV_CAM_DAHUA::TRIGGER_MODE_LINE1
#define MID_PIXEL_FORMAT HV_CAM_DAHUA::BayerRG8
#define MID_ACQUISITION_FRAME_RATE 800

//extern vector<Mat> img_list1;
CSigmaController m_Sigmal;
HV_CAM_DAHUA midcamera=HV_CAM_DAHUA();
vector<vector<float>> gen_scan_routes;

vector<vector<float>> SolveScanRoutes(int sample_nums);
void SendXYSignal();
bool SendSolvedXYSignal(vector<vector<float>>& solved_scan_voltages);
void InitializeComPort();
bool ConnectSettingCamera();
bool CloseCamera();

int main()
{
	cv::String cfg_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov4.cfg";
	cv::String weights_file = "C:\\CODEREPO\\DahuaGal\\model\\yolov4.weights";


	cv::Mat grab_img1 = cv::imread("equipment.png");
	cv::namedWindow("here");
	cv::imshow("here", grab_img1);
	cv::waitKey(1000);
	cv::destroyWindow("here");

	/* 1. Initialize the COM_DA contorller.*/
	InitializeComPort();

	/* 2. Connect and setting the camera. */
	bool is_open = ConnectSettingCamera();

	/* 3. Compute the scanning path given the samples.Using the route planning algorithms. */
	int scan_samples = 100;
	//vector<vector<float>> gen_scan_routes = SolveScanRoutes(scan_samples);
	gen_scan_routes = SolveScanRoutes(scan_samples);

	/* 4. Processing here. Create ConvertImage object. Grab and convert the image. 
		Create 1) send (x,y) thread and 2) image convert therad.
	*/
	ConvertImage image_convertor = ConvertImage();
	image_convertor.num_samples = scan_samples;
	chrono::steady_clock::time_point begin_time2 = chrono::steady_clock::now();

	thread send_com_thread(SendXYSignal);
	//thread send_com_thread(SendSolvedXYSignal, ref(gen_scan_routes));
	thread convert_image_thread(&ConvertImage::process_image, image_convertor);

	//cout << "send xy func thread is join!!!!!!!!!!!!!!" << endl;
	//cout << "convert func thread is join!!!!!!!!!!!!!!" << endl;
	cout << "begin to save!" << endl;

	while (img_list1.size() < scan_samples)
		//while (midcamera.img_list.size() < scan_samples)
	{
		cout << "mid camera image list: " << midcamera.img_list.size() << endl;
		cout <<"get img: "<< img_list1.size() << endl;
		//cout << "call back convert imgage: " << midcamera.img_list.size() << endl;
		this_thread::sleep_for(chrono::microseconds(2000));
	}

	chrono::steady_clock::time_point send_end_time3 = chrono::steady_clock::now();
	cout << "The whole process is  get clock (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(send_end_time3 - begin_time2).count() / 1000 << endl;

	//isFrameOK = true;

	//send_com_thread.join();
	//convert_image_thread.join();
	send_com_thread.detach();
	convert_image_thread.detach();
	 
	chrono::steady_clock::time_point begin_detect1 = chrono::steady_clock::now();


	// 11-22
	cout << "The image list remain is: " << img_list1.size() << endl;

	ObjectDetector detector = ObjectDetector();
	detector.initialization(cfg_file, weights_file, 224, 224);

	// warm up.
	cv::Mat grab_img = cv::imread("1122all_new44.jpg");
	bool is_detected = detector.inference(grab_img);
	cout << "detect obj: " << is_detected << endl;
	cv::Mat grab_img3 = cv::imread("1122all_new86.jpg");
	is_detected = detector.inference(grab_img3);
	cout<<"detect obj: "<<is_detected<<endl;

	//waitKey(5000);
	chrono::steady_clock::time_point begin_detect = chrono::steady_clock::now();

	//cv::Mat grab_img1 = cv::imread("1111all_new32.jpg");
	int real_img = 0;
	int detected_img = 0;
	for (int i = 0; i < img_list1.size(); i++)
	{
		grab_img1 = img_list1[i];
		grab_img1 = grab_img1(cv::Rect(20, 0, 224, 224));

		//cout << grab_img1.size << endl;
		//cout << grab_img1.channels() << endl;
		//cout << grab_img1.total() << endl;

		// save 
		//std::stringstream filename;
		//filename << "./Image/" << "1122all_new" << i << ".jpg";
		//cv::imwrite(filename.str(), img_list1[i]);
		if (!grab_img1.empty())
		{
			real_img += 1;


			//chrono::steady_clock::time_point begin_infer = chrono::steady_clock::now();
			bool is_detected = detector.inference(grab_img1);


			//chrono::steady_clock::time_point end_infer = chrono::steady_clock::now();
			//cout << "The single infer  is  get clock (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_infer - begin_infer).count() / 1000.0 << endl;
			//cout << "Processing speed: " << 1000000.0 / float(chrono::duration_cast<chrono::microseconds>(end_infer - begin_infer).count()) << " FPS!" << endl;


			if (false)
			{
				detected_img += 1;
				static const string kWinName = "Deep learning object detection in OpenCV";
				cv::namedWindow(kWinName, cv::WINDOW_AUTOSIZE);

				////显示s延时信息并绘制
				vector<double> layersTimes;
				double freq = cv::getTickFrequency() / 1000;
				double t = detector.net.getPerfProfile(layersTimes) / freq;
				//double t = net.getPerfProfile(layersTimes) / freq;
				string label = cv::format("Infercence time for a frame:%.2f ms", t);
				cout << "Total inference time ms: " << t << endl;
				//cv::putText(grab_img1, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
				////绘制识别框
				//cv::Mat detecteFrame;
				//grab_img1.convertTo(detecteFrame, CV_8U);
				cv::imshow(kWinName, grab_img1);

				cv::waitKey(200);
			}

			//namedWindow("1", WINDOW_AUTOSIZE);
			//imshow("1", grab_img1);
			//waitKey(300);



		}
	}

	

	chrono::steady_clock::time_point end_detect = chrono::steady_clock::now();
	cout << "The detetcion process is  get clock (ms): !!!!!!!!!!!!!" << chrono::duration_cast<chrono::microseconds>(end_detect - begin_detect).count() / 1000 << endl;
	cout << "Processing speed: " << 100000000.0 / float(chrono::duration_cast<chrono::microseconds>(end_detect - begin_detect).count()) << " FPS!" << endl;
	

	/* 5. Disconnect and close the camera.*/
	img_list1.clear();
	bool is_close = CloseCamera();
	

	return 0;
}

void SendXYSignal()
{
	cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	while (1)
	{
		//cout << "触发模式flag:" << isGrabbingFlag << endl;
		/*float x, y;
		cout << "Please input x,y volt:";
		cin >> x >> y;*/

		if (count == 100)
		{
			cout << "break the send thread" << endl;
			break;
		}
		//m_Sigmal.on_rotate1_usb(gen_scan_routes[count][0], gen_scan_routes[count][1]);
		//int sleep_time = 5000;
		//m_Sigmal.on_rotate1_usb(-4.59, -0.45);
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-3.37, 1.79);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-3.26, 1.8);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-4.06, 3.2);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-2.72, 4.59);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-1.25, 4.23);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(-0.5, 4.28);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(0.57, 4.4);			
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(1.53, 3.32);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));
		//m_Sigmal.on_rotate1_usb(2.03, 3.12);		
		//this_thread::sleep_for(chrono::microseconds(sleep_time));

		//if (count % 2 == 0)
		//{
		// m_Sigmal.on_rotate1_usb(-3.8, -1.15);
		//}
		//else
		//{
		//	m_Sigmal.on_rotate1_usb(-3.9, -1.35);
		//}

		//m_Sigmal.on_rotate1_usb(-4.5 + 0.05 * count, 4.5 - 0.05 * count);
		// 
		if (count % 4 == 0)
		{
			m_Sigmal.on_rotate1_usb(-2.6, 1.7);
			//m_Sigmal.on_rotate1_usb(gen_scan_routes[count][0], gen_scan_routes[count][1]);
		}
		else if(count %4 == 1)
		{
			m_Sigmal.on_rotate1_usb(-2.8, 2.0);
			//m_Sigmal.on_rotate1_usb(gen_scan_routes[count][0], gen_scan_routes[count][1]);
		}
		else if(count % 4==2)
		{
			m_Sigmal.on_rotate1_usb(-2.93, 1.8);
			//m_Sigmal.on_rotate1_usb(gen_scan_routes[count][0], gen_scan_routes[count][1]);
		}
		else
		{
			m_Sigmal.on_rotate1_usb(-3.25, 1.71);
			//m_Sigmal.on_rotate1_usb(gen_scan_routes[count][0], gen_scan_routes[count][1]);
		}

		count++;
		//count = count + 10;
		cout << "send signals: " << count << endl;
		//cout << "send signals: " << count << "; data: ["<< gen_scan_routes[count-1][0] <<"," << gen_scan_routes[count-1][1] << "]" << endl;
		//Sleep(2);
		this_thread::sleep_for(chrono::microseconds(1000));
	}
	cout << "cout the x y is overed!" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
}

bool SendSolvedXYSignal(vector<vector<float>>& solved_scan_voltages)
{
	cout << "!!!!!!!!! begin to send com thread" << endl;
	int count = 0;
	chrono::steady_clock::time_point begin_time_thre = chrono::steady_clock::now();
	for (int i = 0; i < solved_scan_voltages.size(); i++)
	{
		count++;
		cout << "sent " << count << " :[" << solved_scan_voltages[i][0] << "," << solved_scan_voltages[i][1] << "]" << endl;
		m_Sigmal.on_rotate1_usb(solved_scan_voltages[i][0], solved_scan_voltages[i][1]);
		this_thread::sleep_for(chrono::microseconds(4000));
		//while (!is_callback_ok)
		//{
		//	this_thread::sleep_for(chrono::microseconds(100));
		//}
		//is_callback_ok = false;
	}
	cout << "Total send: " << count << endl;

	cout << "cout the x y is overed!" << endl;
	chrono::steady_clock::time_point send_end_time2_thre = chrono::steady_clock::now();
	cout << "Send clock (ms): " << chrono::duration_cast<chrono::microseconds>(send_end_time2_thre - begin_time_thre).count() / 1000 << endl;
	return true;
}

vector<vector<float>> SolveScanRoutes(int sample_nums)
{
	chrono::steady_clock::time_point begin_time_ortools = chrono::steady_clock::now();

	int num_samples = sample_nums;
	//// 1. Generate samples;
	vector<vector<int>> scan_samples = operations_research::GenerateSamples(num_samples);
	// compute the start and end points of each scan.
	int* assign_points = operations_research::CalculateStartEndPoints(scan_samples);
	cout << "original samples" << endl;
	for (auto temp_sample : scan_samples)
	{

		cout << "[" << temp_sample[0] << ", " << temp_sample[1] << "] ";
	}
	cout << "Scan begin points: " << scan_samples[assign_points[0]][0] << ", " << scan_samples[assign_points[0]][1] << endl;
	cout << "Scan end points: " << scan_samples[assign_points[1]][0] << ", " << scan_samples[assign_points[1]][1] << endl;
	//// 2. Compute distance matrix;
	vector<vector<int64_t>> chebyshev_dist = operations_research::ComputeChebyshevDistanceMatrix(num_samples, scan_samples);
	//// 3. Initial the Datamodel;

	operations_research::DefineStartDataModel StartEndData;
	StartEndData.distance_matrix = chebyshev_dist;
	StartEndData.num_vehicles = 1;
	StartEndData.starts.push_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[0] });
	StartEndData.ends.push_back(operations_research::RoutingIndexManager::NodeIndex{ assign_points[1] });
	vector<int> scan_index = operations_research::Tsp(StartEndData);

	vector<vector<float>> scan_voltages(num_samples, vector<float>(2, 0));
	for (int index = 0; index < scan_index.size(); index++)
	{
		cout << "[" << scan_samples[scan_index[index]][0] << ", " << scan_samples[scan_index[index]][1] << "] ";
		scan_voltages[index][0] = scan_samples[scan_index[index]][0] / 100.0;
		scan_voltages[index][1] = scan_samples[scan_index[index]][1] / 100.0;
	}
	for (auto i : scan_index)
	{
		cout << i << "-> ";
	}
	cout << " " << endl;
	chrono::steady_clock::time_point end_time_ortools = chrono::steady_clock::now();
	cout << "Waitting for save (ms): " << chrono::duration_cast<chrono::microseconds>(end_time_ortools - begin_time_ortools).count() / 1000.0 << endl;
	return scan_voltages;
}


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
		cout << "Initialize the COM port succeed!" << endl;
	}
	else
	{
		cout << "Initialize the COM port failed !!!" << endl;
	}
	Sleep(10);
	m_Sigmal.on_rotate1_usb(-4.99, -4.99);

}

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
	return is_ok;
}

bool CloseCamera()
{
	bool is_ok;
	is_ok = midcamera.cameraStopGrabbing();
	is_ok = midcamera.unregisterCallback();
	is_ok = midcamera.closeCamera();
	return is_ok;
}