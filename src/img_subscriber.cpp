#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>

//#include <zbar/Exception.h>

using namespace cv;
using namespace std;
using namespace zbar;
 
cv::Mat image;

ros::Publisher barcode_pub_;

 
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
	
	
	//閾値処理
	//cv::threshold(cv_image, cv_image, 200, 255, CV_THRESH_TOZERO_INV );//どれくらいの明るさ以上で白にするかという指標を閾値として0~255で与える
	//CV_THRESH_TOZERO_INVで、いったん閾値よりも上を飛ばす
	//光っていたらという場合は実際の現場では考慮しなくてよい？
	
	//予想以上に、光っているところが全て白く認識されてしまうので、ここをどうにかしないと使いづらい
	
	
	//cv::bitwise_not(cv_image, cv_image); // 白黒の反転
	
	//QRコードが黒くなるようになれば、読み取ってくれる
	//背景が明るい場合は、反転させると逆に良くない
	
	//反転画像としていない画像の両方が必要？
	
	//輪郭をよりはっきりさせるために2値変換
	cv::threshold(cv_image, cv_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//2値変換
	
	//画像確認用
	imshow("2way",cv_image);
	 
    int width  = image.cols;
	int height = image.rows;
	
	//ImageScanner scanner;
	// wrap image data
    //Image qr_image(width, height, "Y800", (unsigned char*)cv_image.data, width * height);
    //unsigned char型　保存できる値は　0-255です
    //恐らくグレースケールなので0-255までの値しか入らないということだと思う。
    //なので、0-255以外の値にならないようにunsigned charをつけているのではないかと考えている
    
    //"Y800"の後の4つ目の変数がqrコードのscanに使っているデータのようだ
	
    // scan the image for barcodes
    //int n = scanner.scan(qr_image);
    //読み取れている場合はn=1,読み取れていないとn=0
    //printf("%d",n);
   
    
    //この時点で読み取れていれば、それを出力させる
    
	cv::Mat cv_img_copy = cv_image.clone();//元画像のコピー
	//2値画像のコピー
	
	//閾値処理のあたりがあまり有効には働いていない
	//ない方がQRコード見つけやすいまであるかも
	
	
	
	//マスク画像用
	//vector<vector<Point> > contours_subset;
	Mat mask = Mat::zeros(height, width, CV_8UC1);
	Mat result = Mat::zeros(height, width, CV_8UC1);
	
	vector<Point>    approxContours;        // 近似結果の輪郭を格納する領域
    //double epsiron = 32.0f;
	
	//読み取れなかった場合
	//if(n==0)
	//{
	
	//輪郭検出
	
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(cv_image, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	//一番外の輪郭だけ取り出すようにすると、少しは軽くなる
	//大体、輪郭の数が全て取り出した時の半分
	
	//各輪郭はcontours[i]
	
	std::vector< std::vector<cv::Point> > contours_qr;
	int j = 0;
	
	cv::Mat black_img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3); 


	//輪郭のうち、QRコードの可能性がないものを排除する
for(int i = 0; i < contours.size(); i++) {
	 // ある程度の面積が有るものだけに絞る

	 
	 double area = contourArea(contours[i],false);
    if(area > 10000 && area < width*height*0.5) {//小さいもの,大きすぎるものは取り除く
		
		//contours_subset.push_back(contours.at(i));
		contours[j] = contours[i];
		j++;
		
	}
	
}
	
	Mat proj_mat;
	Mat dst;
	
	 Point2f dstPoint[] =
	  {
        Point(256, 256),
        Point(0, 256),
        Point(0, 0),
        Point(256, 0),
		};
		
		//結局無理やり正方形に当てはめればいいのではないか？
	
for(int i = 0; i <= j; i++) {
      
    //輪郭を直線近似する
    approxPolyDP(contours[i], approxContours, 0.01 * cv::arcLength(contours[i], true), true);
    //座標の順序を一定にする
    cv::convexHull(approxContours, approxContours);
    
	//iの値に対応して、近似した領域を出せる。したがって、順番に全ての矩形な領域を導出できるはず
	
	//近似だから、曲面の場合ははみ出してしまう領域があることになる。
	//その分の補正が必要になる

	//近似した場合に、矩形になるものがQRコードだと考える
	if(approxContours.size() == 4)
	{
		
		//短形なものに輪郭線をつける
		//cv::polylines(img_copy, contours[i], true, cv::Scalar(0, 255, 0), 2);
		
		//透視変換用
		
		//透視変換の座標
		//qrコードが写っている輪郭に合わせて、座標を変える必要がある
		
	//一応右下から時計回りに順に並んでいるものと考える
       Point2f oContours[] = 
       {
        approxContours[0],
        approxContours[1],
        approxContours[2],
        approxContours[3],
		};
      
    proj_mat = getPerspectiveTransform(oContours, dstPoint);
    
    //warpPerspective( image, dst, proj_mat, Size(width, height));
    warpPerspective( image, dst, proj_mat, Size(256, 256));
    
    //qrコードをわかりやすいように、2値変換後の画像から引っ張ってくる
    
    //変換を行うとものすごく重くなる
    //for文1個でやらせているのがそもそも構造的に良くないということか
    
    
    //本来はここで別の画像としてそれぞれを保存する必要がある
    //Size(出力画像のサイズ)
	
    //imshow("dst",dst);
    //画像の表示は速度には影響していない
    
    //0725_変換はできているが、どこをどう変換されているかがわからない
    //変換した後がちゃんとした形になる画像で試さないとわからん
    
    //正方形に変換した後、QRコードを探す
    //QRコードだけが貼ってあるから、QRコードの更に外枠を四角形と認識して変換
    
		
			//}
			
//}
	//imshow("re",cv_img_copy);
	//マスク画像用 
	
    //drawContours(mask,contours,-1,Scalar(255),-1);
    //cv_img_copy.copyTo(result,mask);
    
    //resultがマスク画像
    
    //imshow("result",cv_img_copy);
}
    
    //マスク処理もまあまあ重い
	
	//dstに処理後のデータが入っている
	//これが,QRコードの正しい画像になれば良い
	
	//QRコードの画像を用意したほうが早いかな？
	
	if (!dst.empty()) {//dstが入った時だけQRコードの認識をする
  
	Mat dst_gray;
	cvtColor(dst, dst_gray,CV_RGB2GRAY);
	cv::threshold(dst_gray, dst_gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//2値変換
	//qrコードがぼんやりしている
	imshow("dst",dst_gray);
	//cv::waitKey(1);
	
	ImageScanner scanner;
	int width  = dst_gray.cols;
	int height = dst_gray.rows;
	
	
	// wrap image data
    Image qr_image(width, height, "Y800", (unsigned char*)dst_gray.data, width * height);
    //unsigned char型　保存できる値は　0-255です
    //恐らくグレースケールなので0-255までの値しか入らないということだと思う。
    //なので、0-255以外の値にならないようにunsigned charをつけているのではないかと考えている
    
    //"Y800"の後の4つ目の変数がqrコードのscanに使っているデータのようだ
	
    // scan the image for barcodes
    int n = scanner.scan(qr_image);
    //読み取れている場合はn=1,読み取れていないとn=0
    //printf("%d",n);
    
    //なので、for文に組み込みたいときはnの0,1でif文を作れば良い？
    
    if(n!=0)
    {
	std::vector<cv::Point> corners;
    // extract results
    for(Image::SymbolIterator symbol = qr_image.symbol_begin();//zbar::Image::SymbolIterator
        symbol != qr_image.symbol_end();
        ++symbol) {
        // do something useful with results
       
        cout << "decoded " << symbol->get_type_name()
             << " symbol \"" << symbol->get_data() << '"' << endl;
             //get_type_nameの方が、どの種類のバーコードか、get_dataがバーコードの内容を指している
            std_msgs::String barcode_string;
			barcode_string.data = symbol->get_data();
			barcode_pub_.publish(barcode_string);
			
			//画像内でのqrコードの位置が分かる
			//0718 動作確認 緑の線でQRコードが囲まれていた
			if (symbol->get_location_size() == 4) {

				//symbolの中に、QRコードの4点が記録されていて、それに合わせて線を引いている
				//cv::line 画像内への直線の描画
				//線の色は赤
                line(img_copy, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(255, 0, 0), 2, 8, 0);
            }   
            
            //qrコードが複数ある場合に、前のやつと別だと分かるようにする必要がある
    }
    
}

	
	 //輪郭に線を引く
	 //線の色は緑
	line(img_copy, approxContours[0], approxContours[1], Scalar(0, 255, 0), 2, 8, 0);
    line(img_copy, approxContours[1], approxContours[2], Scalar(0, 255, 0), 2, 8, 0);
    line(img_copy, approxContours[2], approxContours[3], Scalar(0, 255, 0), 2, 8, 0);
    line(img_copy, approxContours[3], approxContours[0], Scalar(0, 255, 0), 2, 8, 0);
    
    
}
	

}//for文の中にqrコードデコーダを入れる場合

//0726　画像がクソほど重たくなるが、一応これなら動くことは動く
//あとは、warp処理で変換した後の画像でも動くかが問題
//}
    //imshow("gray_copy",gray_copy);
    imshow("img_copy",img_copy);

    //imshow("image",image);
    
    //
    //imshow("result",result);
	cv::waitKey(1);

}
 
int main(int argc, char** argv)
{
	ros::init (argc, argv, "img_subscriber");
	ros::NodeHandle nh("~");
 
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
 
	//code = nh.advertise<std::string>("qr_data", 1);
	 barcode_pub_ = nh.advertise<std_msgs::String>("barcode", 1);
 
	ros::spin();
 
	return 0;
}
