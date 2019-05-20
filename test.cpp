#include "top.h"
#include <hls_opencv.h>

void ImageZoom_rot(uchar *src, int src_width, int src_height,
		int src_center_row, int src_center_col,
		int cut_width, int cut_height,
		uchar *dst, int dst_width, int dst_height,
		double angle)
{
	float zoomWidth = cut_width / (float)dst_width;
	float zoomHeight = cut_height / (float)dst_height;

	float row,col;
	int a,a1,b,b1;
	float fab,fab1,fa1b,fa1b1;

	float rowdiff,rowdiff_1;
	float coldiff,coldiff_1;

	int index1, index2;

	float cost = cos(angle);
	float sint = sin(angle);
	float start_r = src_center_row + cut_width * sint / 2.0 - cut_height * cost / 2.0;
	float start_c = src_center_col - cut_width * cost / 2.0 - cut_height * sint / 2.0;
	float zoomWidthSin = -zoomWidth * sint;
	float zoomWidthCos = zoomWidth * cost;
	float zoomHeightSin = zoomHeight * sint;
	float zoomHeightCos = zoomHeight *cost;

	int i,j;

	for(i = 0 ; i < dst_height; i++)
	{
		row = start_r;
		col = start_c;
		for(j = 0 ; j < dst_width; j++)
		{
			a = (int)row;
			a1 = a+1;
			b = (int)col;
			b1 = b+1;

			if(a < 0 || b < 0 || a1 >= src_height || b1 >= src_width)
			{
				dst[i * dst_width + j] = 0;
				row += zoomWidthSin;
				col += zoomWidthCos;

				continue;
			}

			rowdiff = row - a;
			rowdiff_1 = 1.f - rowdiff;
			coldiff = col - b;
			coldiff_1 = 1.f - coldiff;

			index1 = a * src_width + b;
			index2 = a1 * src_width + b;

			fab = src[index1] * rowdiff_1 * coldiff_1;
			fab1 = src[index1 + 1] * rowdiff_1 * coldiff;
			fa1b = src[index2] * rowdiff * coldiff_1;
			fa1b1 = src[index2 + 1] * rowdiff * coldiff;

			dst[i * dst_width + j]  = (unsigned char)(fab + fa1b + fa1b1 + fab1);

			row += zoomWidthSin;
			col += zoomWidthCos;
		}
		start_r += zoomHeightCos;
		start_c += zoomHeightSin;
	}
}

//計算邊界 返回寬長
cv::Point2i FindBoundary(cv::Mat src, double angle)
{
    cv::Point2i vertices[4] = { cv::Point2i(0, 0),
        cv::Point2i(0, src.cols - 1),
        cv::Point2i(src.rows - 1, 0),
        cv::Point2i(src.rows - 1, src.cols - 1) };

    cv::Point2f newVertices[4];
    for (int i = 0; i < 4; i++)
    {
        newVertices[i].x = (vertices[i].x + vertices[i].y * sin(angle) ) / cos(angle);
        newVertices[i].y = ( vertices[i].x - vertices[i].y ) / (2 * sin(angle));
    }

    float min_x = std::min(newVertices[0].x, std::min(newVertices[1].x, std::min(newVertices[2].x, newVertices[3].x)));
    float max_x = std::max(newVertices[0].x, std::max(newVertices[1].x, std::max(newVertices[2].x, newVertices[3].x)));
    float min_y = std::min(newVertices[0].y, std::min(newVertices[1].y, std::min(newVertices[2].y, newVertices[3].y)));
    float max_y = std::max(newVertices[0].y, std::max(newVertices[1].y, std::max(newVertices[2].y, newVertices[3].y)));


    cv::Point2i boundary = cv::Point2i(max_x - min_x + 1, max_y - min_y + 1);
//
    boundary.y = src.cols;
    boundary.x = src.rows;
    std::cout << "new height : " << boundary.x << " new width : " << boundary.y << std::endl;
    return boundary;
}

void NearestNeighbor(cv::Mat src, double angle, cv::Point2i boundary, std::string name)
{
    int oldWidth = src.cols;
    int oldHeight = src.rows;
    int oldCenterRow = (oldHeight - 1) / 2;
    int oldCenterCol = (oldWidth - 1) / 2;

    int newWidth = boundary.y;
    int newHeight = boundary.x;
    int newCenterRow = (newHeight - 1) / 2;
    int newCenterCol = (newWidth - 1) / 2;

    cv::Mat output = cv::Mat(newHeight, newWidth, src.type());

    int paddRow = -newCenterRow * cos(angle) + newCenterCol * sin(angle) + oldCenterRow;
    int paddCol = -newCenterRow * cos(angle) - newCenterCol * sin(angle) + oldCenterCol;

    for (int row = 0; row < newHeight; row++)
    {
        for (int col = 0; col < newWidth; col++)
        {
            //先旋轉再偏移
            int oldRow = row * cos(angle) - col * sin(angle) + 0.5 + paddRow;
            int oldCol = row * cos(angle) + col * sin(angle) + 0.5 + paddCol;

            //防出界
            if (oldRow >= 0 && oldRow < oldHeight &&
                oldCol >= 0 && oldCol < oldWidth)
            {
                output.at<cv::Vec3b>(row, col) = src.at<cv::Vec3b>(oldRow, oldCol);
            }
            else
                output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);

        }
    }

    cv::imwrite("NearestNeighbor_" + name + ".jpg", output);
}

void Bilinear(cv::Mat src, double angle, cv::Point2i boundary, std::string name)
{
    int oldWidth = src.cols;
    int oldHeight = src.rows;
    int oldCenterRow = (oldHeight - 1) / 2;
    int oldCenterCol = (oldWidth - 1) / 2;

    int newWidth = boundary.y;
    int newHeight = boundary.x;
    int newCenterRow = (newHeight - 1) / 2;
    int newCenterCol = (newWidth - 1) / 2;

    cv::Mat output = cv::Mat(newHeight, newWidth, src.type());

    int paddRow = -newCenterRow * cos(angle) + newCenterCol * sin(angle) + oldCenterRow;
    int paddCol = -newCenterRow * cos(angle) - newCenterCol * sin(angle) + oldCenterCol;

    for (int row = 0; row < newHeight; row++)
    {
        for (int col = 0; col < newWidth; col++)
        {
            //先旋轉再偏移
            double tempRow = row * cos(angle) - col * sin(angle) + paddRow + 0.5;
            double tempCol = row * cos(angle) + col * sin(angle) + paddCol + 0.5;

            //防出界
            if (tempRow < 0 || tempRow >= oldHeight || tempCol < 0 || tempCol >= oldWidth)
            {
                output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                continue;
            }

            cv::Point a = cv::Point((int)tempRow, (int)tempCol);  //a離p點最近

                                                                  /***
                                                                  a     b
                                                                  p
                                                                  c     d
                                                                  ****/

            double a_to_row = tempRow - a.x;    //p點到a點的垂直距離
            double a_to_col = tempCol - a.y;    //p點到a點的水平距離
            //設定其他三個點
            cv::Point b = cv::Point(a.x, a.y + 1);
            cv::Point c = cv::Point(a.x + 1, a.y);
            cv::Point d = cv::Point(a.x + 1, a.y + 1);
            //防邊邊
            if (a.x == oldHeight - 1)
            {
                c = a;
                d = b;
            }

            if (a.y == oldWidth - 1)
            {
                b = a;
                d = c;
            }


            output.at<cv::Vec3b>(row, col) = (1 - a_to_row) * (1 - a_to_col) * src.at<cv::Vec3b>(a.x, a.y)
                + (1 - a_to_row)* a_to_col * src.at<cv::Vec3b>(b.x, b.y)
                + a_to_row * (1 - a_to_col) * src.at<cv::Vec3b>(c.x, c.y)
                + a_to_row * a_to_col * src.at<cv::Vec3b>(d.x, d.y);

        }
    }

    cv::imwrite("Bilinear_" + name + ".jpg", output);
}

double CalculateW(double x)
{
    double a = -0.1, value, X = std::abs(x);

    if (X == 0)
    {
        value = 1;
    }
    else if (X <= 1)
    {
        value = (a + 2) * std::pow(X, 3) - (a + 3) * std::pow(X, 2) + 1;
    }
    else if (X < 2)
    {
        value = a * std::pow(X, 3) - 5 * a * std::pow(X, 2) + 8 * a * X - 4 * a;
    }
    else {
        value = 0.0;
    }

    return value;
}

void Bicubic(cv::Mat src, double angle, cv::Point2i boundary, std::string name)
{
    int oldWidth = src.cols;
    int oldHeight = src.rows;
    int oldCenterRow = (oldHeight - 1) / 2;
    int oldCenterCol = (oldWidth - 1) / 2;

    int newWidth = boundary.y;
    int newHeight = boundary.x;
    int newCenterRow = (newHeight - 1) / 2;
    int newCenterCol = (newWidth - 1) / 2;

    cv::Mat output = cv::Mat(newHeight, newWidth, src.type());

    int paddRow = -newCenterRow * cos(angle) + newCenterCol * sin(angle) + oldCenterRow;
    int paddCol = -newCenterRow * cos(angle) - newCenterCol * sin(angle) + oldCenterCol;

    for (int row = 0; row < newHeight; row++)
    {
        for (int col = 0; col < newWidth; col++)
        {
            //先旋轉再偏移
            double tempRow = row * cos(angle) - col * sin(angle) + paddRow;
            double tempCol = row * cos(angle) + col * sin(angle) + paddCol;

            int tempX = tempRow + 0.5, tempY = tempCol + 0.5;

            int arrX[4] = { tempX - 1, tempX, tempX + 1, tempX + 2 };
            int arrY[4] = { tempY - 1, tempY, tempY + 1, tempY + 2 };



            //防出界
            if (arrX[0] >= 0 && arrX[3] < oldHeight && arrY[0] >= 0 && arrY[3] < oldWidth)
            {
                cv::Vec3b value = cv::Vec3b(0, 0, 0);
                //套公式
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        value += (src.at<cv::Vec3b>(arrX[i], arrY[j]) * CalculateW(tempRow - arrX[i]) * CalculateW(tempCol - arrY[j]));
                    }
                }

                output.at<cv::Vec3b>(row, col) = value;
            }
            else
                output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
        }
    }

    cv::imwrite("Bicubic_" + name + ".jpg", output);
}

int main (void) {

	// Load data in OpenCV image format
	IplImage* src = cvLoadImage("lenna.png");//test_1080p.bmp

	//Get input Image size
	CvSize size_in;
	size_in = cvGetSize(src);
	float angle=45/180.0*CV_PI;
	ap_int<32> frame_buf[512*512];
//	frame_buf = (ap_int<24> *)0x8000000;
	memset(frame_buf,0,512*512*sizeof(ap_int<32>));
	//Create Destination image
	IplImage* dst = cvCreateImage(size_in, src->depth, src->nChannels);

	//Create the AXI4-Stream
	AXI_STREAM src_axi, dst_axi;

	// Convert OpenCV format to AXI4 Stream format
	IplImage2AXIvideo(src, src_axi);

	// Call the function to be synthesized
	video_rotate_bilinear(src_axi, dst_axi,size_in.width,size_in.height,angle,frame_buf);

	// Convert the AXI4 Stream data to OpenCV format
	AXIvideo2IplImage(dst_axi, dst);

	// Standard OpenCV image functions
	cvSaveImage("out.png", dst);
//
//	video_rotate(src_axi, dst_axi,size_in.width,size_in.height,angle,frame_buf);
//	AXIvideo2IplImage(dst_axi, dst);
//	cvSaveImage("out_bilinear.png", dst);
//
//	cv::Mat _src(src);
//    cv::Point2i boundary = FindBoundary(_src, angle);
//
//    NearestNeighbor(_src, angle, boundary, "lena");
//    Bilinear(_src, angle, boundary, "lena");
//    Bicubic(_src, angle, boundary, "lena");
//    cv::waitKey(0);
	cvReleaseImage(&src);
	cvReleaseImage(&dst);

	return 0;
}
