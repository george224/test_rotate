#include "top.h"

//Top Level Function
void video_rotate(AXI_STREAM& s_axis_video,AXI_STREAM& m_axis_video, int hsize_in, int vsize_in,
				float angle,ap_int<32> *frame_buf)
{
#pragma HLS INTERFACE m_axi depth=8388608 port=frame_buf offset=slave
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=angle offset=0x24 bundle=CONTROL_BUS
#pragma HLS INTERFACE s_axilite port=vsize_in offset=0x18 bundle=CONTROL_BUS
#pragma HLS INTERFACE s_axilite port=hsize_in offset=0x10 bundle=CONTROL_BUS
#pragma HLS INTERFACE axis register port=s_axis_video bundle=INPUT_STREAM
#pragma HLS INTERFACE axis register port=m_axis_video bundle=OUTPUT_STREAM

	ap_axiu<24, 1, 1, 1> video;
	ap_axiu<24, 1, 1, 1> video_out;

//	assert(hsize_in <= MAX_WIDTH);
//	assert(vsize_in <= MAX_HEIGHT);
//	assert(hsize_in >= 0);
//	assert(vsize_in >= 0);

	float sina = hls::sinf(angle);
	float cosa = hls::cosf(angle);
	int h_half=hsize_in/2;
	int v_half=vsize_in/2;
	int x,y=0;//原始点坐标
	int s,t=0;//旋转后坐标

//	memset(frame_buf,0,1920*1080*sizeof(ap_int<32>));
//	memset(&video_out,0,sizeof(ap_axiu<24, 1, 1, 1>));
//	for(int i = 0; i < vsize_in ; i ++)
//	{
//		for(int j = 0; j < hsize_in ; j ++)
//		{
//			s_axis_video >> video;
//			frame_buf[j+i*hsize_in] = video.data/2;
//			video_out = video;
//			video_out.data = frame_buf[j+i*hsize_in];
//
//			m_axis_video << video_out;
//		}
//	}

	for(int i = 0; i < vsize_in ; i ++)
	{
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1080
		for(int j = 0; j < hsize_in ; j ++)
		{
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1920
#pragma HLS PIPELINE II=1
			s_axis_video >> video;
			x = j-h_half;
			y = i-v_half;
			s = (int)(x*cosa - y*sina + 0.5);
			t = (int)(x*sina + y*cosa + 0.5);

			if((s>=-h_half)&&(s<h_half)&&(t>=-v_half)&&(t<v_half))
			{
				int index = s+h_half+(t+v_half)*hsize_in;
//				printf("index=%d\n",index);
//				if(index>=512*512)
//				{
//					printf("index error.\n");
//				}
				frame_buf[index] = video.data;
			}
		}
	}

	for(int i = 0; i < vsize_in ; i ++)
	{
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1080
		for(int j = 0; j < hsize_in ; j ++)
		{
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1920
#pragma HLS PIPELINE II=1
			video_out.data = frame_buf[j+i*hsize_in];
			video_out.keep = 15;
			if((i==0)&&(j==0))
			{
				video_out.user = 1;
				video_out.last = 0;
			}
			else if(j==hsize_in-1)
			{
				video_out.user = 0;
				video_out.last = 1;
			}
			else
			{
				video_out.user = 0;
				video_out.last = 0;
			}
			m_axis_video << video_out;
		}
	}
}

void video_rotate_bilinear(AXI_STREAM& s_axis_video,AXI_STREAM& m_axis_video, int13 hsize_in, int13 vsize_in,
		float angle,ap_int<32> *frame_buf)
{
#pragma HLS INTERFACE m_axi depth=67108864 port=frame_buf offset=slave
#pragma HLS INTERFACE s_axilite port=return
#pragma HLS INTERFACE s_axilite port=angle offset=0x24 bundle=CONTROL_BUS
#pragma HLS INTERFACE s_axilite port=vsize_in offset=0x18 bundle=CONTROL_BUS
#pragma HLS INTERFACE s_axilite port=hsize_in offset=0x10 bundle=CONTROL_BUS
#pragma HLS INTERFACE axis register port=s_axis_video bundle=INPUT_STREAM
#pragma HLS INTERFACE axis register port=m_axis_video bundle=OUTPUT_STREAM

	ap_axiu<24, 1, 1, 1> video;
	ap_axiu<24, 1, 1, 1> video_out;

//	assert(hsize_in <= MAX_WIDTH);
//	assert(vsize_in <= MAX_HEIGHT);
//	assert(hsize_in >= 0);
//	assert(vsize_in >= 0);

	int13 oldWidth = hsize_in;
	int13 oldHeight = vsize_in;
	int13 oldCenterRow = (oldHeight - 1) / 2;
	int13 oldCenterCol = (oldWidth - 1) / 2;

	int13 newWidth = hsize_in;
	int13 newHeight = vsize_in;
	int13 newCenterRow = (newHeight - 1) / 2;
	int13 newCenterCol = (newWidth - 1) / 2;

	int14 paddRow = -newCenterRow * hls::cosf(angle) + newCenterCol * hls::sinf(angle) + oldCenterRow;
	int14 paddCol = -newCenterRow * hls::cosf(angle) - newCenterCol * hls::sinf(angle) + oldCenterCol;

    int index=0;

    for (int13 row = 0; row < newHeight; row++)
    {
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1080
        for (int13 col = 0; col < newWidth; col++)
        {
#pragma HLS PIPELINE II=1 rewind
#pragma HLS loop_flatten off
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1920
        	s_axis_video >> video;
        	index = col+row*hsize_in;
        	frame_buf[index] = video.data;
        }
    }

    for (int13 row = 0; row < newHeight; row++)
    {
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1080
        for (int13 col = 0; col < newWidth; col++)
        {
#pragma HLS PIPELINE II=1
#pragma HLS loop_flatten off
#pragma HLS LOOP_TRIPCOUNT min=0 max=2048 avg=1920
            //先旋轉再偏移
        	int14 oldRow = row * hls::cos(angle) - col * hls::sinf(angle) + 0.5 + paddRow;
        	int14 oldCol = row * hls::cos(angle) + col * hls::sinf(angle) + 0.5 + paddCol;
            index = oldCol+oldRow*hsize_in;
            //防出界
            if (oldRow >= 0 && oldRow < oldHeight &&
                oldCol >= 0 && oldCol < oldWidth)
            {
//                output.at<cv::Vec3b>(row, col) = src.at<cv::Vec3b>(oldRow, oldCol);
            	video_out.data= frame_buf[index];
            }
            else
            {
//                output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
				video_out.data = 0;
            }
//            //先旋轉再偏移
//            float tempRow = row * cos(angle) - col * sin(angle) + paddRow + 0.5;
//            float tempCol = row * cos(angle) + col * sin(angle) + paddCol + 0.5;
//            int index = col+row*hsize_in;
//            //防出界
//            if (tempRow < 0 || tempRow >= oldHeight || tempCol < 0 || tempCol >= oldWidth)
//            {
////                output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
//				if(index>=512*512)
//				{
//					printf("index error.\n");
//				}
//                frame_buf[index] = 0;
//                continue;
//            }
//
//            cv::Point a = cv::Point((int)tempRow, (int)tempCol);  //a離p點最近
//
//            float a_to_row = tempRow - a.x;    //p點到a點的垂直距離
//            float a_to_col = tempCol - a.y;    //p點到a點的水平距離
//            //設定其他三個點
//            cv::Point b = cv::Point(a.x, a.y + 1);
//            cv::Point c = cv::Point(a.x + 1, a.y);
//            cv::Point d = cv::Point(a.x + 1, a.y + 1);
//            //防邊邊
//            if (a.x == oldHeight - 1)
//            {
//                c = a;
//                d = b;
//            }
//
//            if (a.y == oldWidth - 1)
//            {
//                b = a;
//                d = c;
//            }
//
//            output.at<cv::Vec3b>(row, col) = (1 - a_to_row) * (1 - a_to_col) * src.at<cv::Vec3b>(a.x, a.y)
//                + (1 - a_to_row)* a_to_col * src.at<cv::Vec3b>(b.x, b.y)
//                + a_to_row * (1 - a_to_col) * src.at<cv::Vec3b>(c.x, c.y)
//                + a_to_row * a_to_col * src.at<cv::Vec3b>(d.x, d.y);
			video_out.keep = 15;
			if((row==0)&&(col==0))
			{
				video_out.user = 1;
				video_out.last = 0;
			}
			else if(col==hsize_in-1)
			{
				video_out.user = 0;
				video_out.last = 1;
			}
			else
			{
				video_out.user = 0;
				video_out.last = 0;
			}
			m_axis_video << video_out;
        }
    }
}
