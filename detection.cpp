#include "detection.h"

ObjectDetector::ObjectDetector()
{

}

ObjectDetector::~ObjectDetector()
{

}

void ObjectDetector::initialization(cv::String cfg, cv::String weight, int input_width, int input_height)
{
    net = cv::dnn::readNetFromDarknet(cfg, weight);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    // CPU
    //net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    inpWidth = input_width, inpHeight = input_height;
    //�������������
    string classesFile = "C:\\CODEREPO\\DahuaGal\\model\\coco.names";//coco.names����80�ֲ�ͬ������
    ifstream ifs(classesFile.c_str());
    string line;
    //vector<string> classes;
    while (getline(ifs, line))classes.emplace_back(line);
}

bool  ObjectDetector::inference(cv::Mat& frame, int frame_id)
{
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, { inpWidth, inpHeight }, 0.00392, true); //1.0 / 255.0  0.00392
    net.setInput(blob);
    vector<Mat> detectionMat;
    try {
        net.forward(detectionMat, getOutputsNames(net));
    }
    catch (cv::Exception& e) {
        cout << "EXCEPTION!!!" << endl;
        return false;
    }
    bool is_detected = postprocess(frame, detectionMat, frame_id);
    return is_detected;
}

// Get the names of the output layers.
vector<String>  ObjectDetector::getOutputsNames(const cv::dnn::Net& net)
{
    static vector<String> names;

    if (names.empty()) {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        //Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }

    return names;
}

// Remove the bounding boxes with low confidence using non-maxima suppression.
bool ObjectDetector::postprocess(cv::Mat& frame, const vector<cv::Mat>& outs, int frame_id)
{
    vector<int> classIds;//����ʶ���������
    vector<float> confidences;//�������Ŷ�
    vector<cv::Rect> boxes;//����߿�
    for (size_t i = 0; i < outs.size(); i++) {
        //�����������ɨ�����б߽��
        //���������Ŷ�ѡ��
        //Ŀ������data:x,y,w,hΪ�ٷֱȣ�x,yΪĿ�����ĵ�����
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; j++, data += outs[i].cols) {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;//���Ŷ�
            //ȡ��������ֵ������
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold) {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                classIds.emplace_back(classIdPoint.x);
                confidences.emplace_back((float)confidence);
                boxes.emplace_back(cv::Rect(left, top, width, height));
            }
        }
    }
    //�����Ŷ�
    vector<int> indices;//����û���ص��߿������
    //�ú������������ص��߿�
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Is the target object detected?
    bool is_target = false;
    bool is_target_frame = false;
    for (size_t i = 0; i < indices.size(); i++) {

        is_target = false;
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        //cout << "class: " << classIds[idx] << ", conf: " << confidences[idx] << endl;
        is_target = classIds[idx] == 2;

        drawPred(classIds[idx], confidences[idx], box.x, box.y,
            box.x + box.width, box.y + box.height, frame);

        //cout << is_target << endl;
        if (is_target)
        {
            is_target_frame = true;
            detected_results.detected_conf.emplace_back(confidences[idx]);
            detected_results.detected_box.emplace_back(box);
            detected_results.detected_ids.emplace_back(frame_id);
        }

    }
    return is_target_frame;
    //return indices.size() > 0 ? true : false;
}

void ObjectDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame) {
    //���Ʊ߽��
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    string label = cv::format("%.2f", conf);
    if (!classes.empty()) {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;//�߿��ϵ�����ǩ�����Ŷ�
    }
    //���Ʊ߽���ϵı�ǩ
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}