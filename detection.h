#pragma once
#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <opencv2/opencv.hpp>
#include<opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/dnn.hpp>
#include <vector>
#include<fstream>
using namespace std;
using namespace cv;


struct DetectedResults
{
	vector<float> detected_conf;
	vector<Rect> detected_box;
	vector<int> detected_ids;
};


struct ResampleCenters
{
	int num_samples;
	float center_x;
	float center_y;
};

class ObjectDetector {
public:
	ObjectDetector();
	~ObjectDetector();
public:
	// Initialize the parameters.
	float confThreshold = 0.5;//���Ŷ���ֵ
	float nmsThreshold = 0.4;//�����������ֵ
	int inpWidth = 416;//��������ͼƬ���
	int inpHeight = 416;//��������ͼƬ�߶�
	vector<string> classes;//�������ֵ�����
	cv::dnn::Net net;

	//vector<float> detected_conf;
	//vector<Rect> detected_box;
	//vector<int> detected_ids;
	DetectedResults detected_results;
	//detected_results.


public:
	void initialization(cv::String cfg, cv::String weight, int input_width, int input_height);
	bool inference(cv::Mat& frame, int frame_id);
	vector<String> getOutputsNames(const cv::dnn::Net& net);
	bool postprocess(cv::Mat& frame, const vector<cv::Mat>& outs, int frame_id);
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
};

#endif
