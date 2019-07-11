#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/image_encodings.h>
#include "qr_code/imageimage.h"
#include "qr_code/distance.h"
#include "qr_code/barcode.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include <zbar/Exception.h>
//0720動作確認
//透視変換のお試し用のプログラム
using namespace cv;
using namespace std;
using namespace zbar;

cv::Mat image;

ros::Publisher barcode_pub_;
ros::Publisher location_pub;
ros::Publisher depth_pub_;
ros::Publisher marker_pub;
image_transport::Publisher bw_image;
image_transport::Publisher qrimage;

int cnt=0;
double temp[100];


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		//cv_bridgeを使って、opencvの形式に変換。BGR8なので、カラー画像
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}


	cv::Mat cv_image;
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
	cv::Mat img_copy = image.clone();//元画像のコピー

	cv::threshold(cv_image, cv_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//2値変換

	//画像確認用
	//imshow("2way",cv_image);

	sensor_msgs::ImagePtr bwmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_image).toImageMsg();

	//rvizで確認
	//http://demura.net/lecture/14060.html

	bw_image.publish(bwmsg);

	ImageScanner scanner;
	int width  = cv_image.cols;
	int height = cv_image.rows;


	// wrap image data
    Image qr_image(width, height, "Y800", (unsigned char*)cv_image.data, width * height);

    // scan the image for barcodes
    int n = scanner.scan(qr_image);

	std::vector<cv::Point> corners;

    // extract results
    for(Image::SymbolIterator symbol = qr_image.symbol_begin();
        symbol != qr_image.symbol_end();
        ++symbol) {
        // do something useful with results

        cout << "decoded " << symbol->get_type_name()
             << " symbol \"" << symbol->get_data() << '"' << endl;
             //get_type_nameの方が、どの種類のバーコードか、get_dataがバーコードの内容を指している
            qr_code::barcode barcode_string;
			barcode_string.bar = symbol->get_data();
			barcode_string.header.stamp = ros::Time::now();
			barcode_pub_.publish(barcode_string);


			//画像内でのqrコードの位置が分かる
			if (symbol->get_location_size() == 4) {

				//symbolの中に、QRコードの4点が記録されていて、それに合わせて線を引いている
				//線の色は赤
                line(img_copy, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(255, 0, 0), 2, 8, 0);

				 qr_code::imageimagePtr msgm(new qr_code::imageimage);

				msgm->header.stamp = ros::Time::now();
				msgm->location_x_0 = symbol->get_location_x(0);
				msgm->location_y_0 = symbol->get_location_y(0);
				msgm->location_x_1 = symbol->get_location_x(1);
				msgm->location_y_1 = symbol->get_location_y(1);
				msgm->location_x_2 = symbol->get_location_x(2);
				msgm->location_y_2 = symbol->get_location_y(2);
				msgm->location_x_3 = symbol->get_location_x(3);
				msgm->location_y_3 = symbol->get_location_y(3);

				location_pub.publish(msgm);


            }

            //qrコードが複数ある場合に、前のやつと別だと分かるようにする必要がある
    }

    sensor_msgs::ImagePtr qrmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_copy).toImageMsg();
    qrimage.publish(qrmsg);

    imshow("image",img_copy);

	cv::waitKey(1);
}

void depthcallback(const std_msgs::Float64::ConstPtr& msg)
{
	//距離
	qr_code::distance dist;

	double sum;
	double average;


	dist.header.stamp = ros::Time::now();

	//移動平均を導出
	if (cnt == 100) {
				cnt = 0;
			}

	temp[cnt] = msg->data;
	cnt++;

	for (int i = 0; i < 100;i++) {
		sum += temp[i];
		}

		average = sum / 100;

	dist.dis = average;

	depth_pub_.publish(dist);
}


int main(int argc, char** argv)
{
	ros::init (argc, argv, "img_subscriber");
	ros::NodeHandle nh("~");

	image_transport::ImageTransport it(nh);

	bw_image= it.advertise("/bw_image", 1);
	qrimage= it.advertise("/qr_image", 1);

	//topic名は適宜変更する
	image_transport::Subscriber image_sub = it.subscribe("/image_raw", 10, imageCallback);

	ros::Subscriber dis_sub = nh.subscribe("/distance", 10, depthcallback);

	location_pub = nh.advertise<qr_code::imageimage>("location", 10);
	barcode_pub_ = nh.advertise<qr_code::barcode>("barcode", 10);

	 depth_pub_ = nh.advertise<qr_code::distance>("qr_code/distance", 10);

	ROS_INFO("SPINNING...");


	ros::spin();

	return 0;
}
