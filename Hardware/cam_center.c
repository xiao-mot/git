#include "cam_center.h"

CamCenterCtrl cam_ctrl;
float Location_distance =0;
extern uint8_t Location_effective;   //放置和抓取定位有效次数

// 初始化默认参数
void CamCtrl_Init(CamCenterCtrl* ctrl)
{
    // 默认比例系数（需校准）// 1像素=0.1cm
    ctrl->pixel_to_cm_x = 0.003f;    //3/16.56   0.1/22.3    圆环0.00418f  物料 
    ctrl->pixel_to_cm_y = 0.005f;    //3/11.56   0.1/10.9    圆环0.006f    物料
    ctrl->is_calibrated = 0;
    ctrl->move_sequence = 0;
}


// 校准函数（实际移动距离已知时调用）
    /* 使用示例：
    1. 让小车向X+方向移动actual_move_cm厘米
    2. 记录移动前后的像素差值Δx
    3. pixel_to_cm_x = actual_move_cm / Δx 
    // 需人工测量后输入参数
*/
void CamCtrl_Calibrate(CamCenterCtrl* ctrl, float actual_move_cm)
{
	ctrl->pixel_to_cm_x = actual_move_cm / (ctrl->CAM_CENTER_X - LX); 
	ctrl->pixel_to_cm_y = actual_move_cm / (ctrl->CAM_CENTER_Y - LY); 	
	
    ctrl->is_calibrated = 1;
}

// 核心居中控制函数
void CamCtrl_AutoCenter(CamCenterCtrl* ctrl, int LX, int LY)
{
    // 计算像素偏差
    float dx = ctrl->CAM_CENTER_X - LX;
    float dy = ctrl->CAM_CENTER_Y - LY;
/*
    // 检查是否已完成校准
    if(!ctrl->is_calibrated) {
        // 此处可添加蜂鸣器报警等提示
        return;
    }
*/	
	
    // 误差容限检查
    if(abs(dx) <= ctrl->MOVE_TOLERANCE && abs(dy) <= ctrl->MOVE_TOLERANCE) {
		Location_effective++;//定位完成有效次数
		Location_distance=0;
        return;             // 已居中定位完毕
    }
    
    // 运动序列控制
    if(ctrl->move_sequence == 0) 
	{
		// X轴调整
        if(abs(dx) > ctrl->MOVE_TOLERANCE) 
		{
            float move_cm = dx * ctrl->pixel_to_cm_x;
            uint8_t direction = (dx > 0) ? 1 : 0; // 1:正方向 0:负方向
			if(direction == 1)     {Mode_Flag = FOR_MODE_LOW; }
			else if(direction ==0) {Mode_Flag = BACK_MODE_Low;}		
            //Get_wheel_circle(fabsf(move_cm)*100); // 转换为0.01cm单位
			Location_distance=fabsf(move_cm);//赋给全局变量
        }
		ctrl->move_sequence = 1;  // 切换Y轴
    } 
    else 
	{  
		// Y轴调整
        if(abs(dy) > ctrl->MOVE_TOLERANCE) 
		{
            float move_cm = dy * ctrl->pixel_to_cm_y;
            uint8_t direction = (dy > 0) ? 1 : 0;// 1:正方向 0:负方向
			if(direction == 1)     {Mode_Flag = TRANS_MODE;     }
			else if(direction ==0) {Mode_Flag = TRANS_LEFT_MODE;}	
			Location_distance=fabsf(move_cm);//赋给全局变量
        }
        ctrl->move_sequence = 0;  // 切换X轴
    }
}



