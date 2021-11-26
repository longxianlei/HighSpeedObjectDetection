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

class ObjectDetector {
public:
	ObjectDetector();
	~ObjectDetector();
public:
	// Initialize the parameters.
	float confThreshold = 0.5;//置信度阈值
	float nmsThreshold = 0.4;//非最大抑制阈值
	int inpWidth = 416;//网络输入图片宽度
	int inpHeight = 416;//网络输入图片高度
	vector<string> classes;//储存名字的容器
	cv::dnn::Net net;
public:
	void initialization(cv::String cfg, cv::String weight, int input_width, int input_height);
	bool inference(cv::Mat& frame);
	vector<String> getOutputsNames(const cv::dnn::Net& net);
	bool postprocess(cv::Mat& frame, const vector<cv::Mat>& outs);
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
};

#endif