#include "convert_image.h"
#include "hv_cam_dahua.h"


vector<Mat> img_list1;

ConvertImage::ConvertImage()
{
	is_frame_ok = false;
}

ConvertImage::~ConvertImage()
{

}

void ConvertImage::process_image()
{
	cout << "!!!!!!!! begin the image convert thread!!!!!!!!" << endl;
	cout << "the setting size of images: " << num_samples << endl;
	//chrono::steady_clock::time_point thread_begin = chrono::steady_clock::now();
	while (1)
	{
		while (!image_safe_queue.empty())
		{
			CFrame pFrame1 = image_safe_queue.wait_and_pop();

			openParam1.width = pFrame1.getImageWidth();
			openParam1.height = pFrame1.getImageHeight();
			openParam1.paddingX = pFrame1.getImagePadddingX();
			openParam1.paddingY = pFrame1.getImagePadddingY();
			openParam1.dataSize = pFrame1.getImageSize();
			openParam1.pixelForamt = pFrame1.getImagePixelFormat();
			//cout <<"Get img width: "<< openParam1.width << ", height: " << openParam1.height << endl;

			pImage1 = pFrame1.getImage();
			nBGRBufferSize1 = pFrame1.getImageWidth() * pFrame1.getImageHeight() * 3;
			uint8_t* pBGRbuffer = new uint8_t[nBGRBufferSize1];
			status1 = IMGCNV_ConvertToBGR24((unsigned char*)pImage1, &openParam1, pBGRbuffer, &nBGRBufferSize1);
			image1 = Mat(pFrame1.getImageHeight(), pFrame1.getImageWidth(), CV_8UC3, (uint8_t*)pBGRbuffer);

			img_list1.emplace_back(image1.clone());
			delete pBGRbuffer;
		}
		this_thread::sleep_for(chrono::microseconds(1000));
		//cout << "img list in the queue: " << img_list1.size() << endl;
		if (image_safe_queue.empty() && img_list1.size() == num_samples)
		{
			cout << "Image queue is empty" << endl;
			break;
		}
		else
		{
			cout << "Image queue is: " << image_safe_queue.size() << "img list number is not full: " << img_list1.size() << endl;
		}
	}
}


