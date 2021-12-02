#pragma once
#ifndef __MAIN_H__
#define __MAIN_H__


// DA Contorller Setting.
#define COM_PORT			3
#define DA_STEP_PCT			1		// %
#define DA_STEP_TIME		0.25		// ms
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


#endif // !__MAIN_H__
