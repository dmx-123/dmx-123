#include "project.h"

using namespace cv;
using namespace std;

#define maxFrame 4540 
#define kMinNumFeature 2000

// Define member functions of class Monocular that we claimed in header file
Monocular::Monocular(double width, double height, double fx, double fy, double cx, double cy,
	double k1, double k2, double p1, double p2, double k3):
	width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), 
	distortion_(fabs(k1) > 0.0000001)
{
	d_[0] = k1; d_[1] = k2; d_[2] = p1; d_[3] = p2; d_[4] = k3;	
}

Monocular::~Monocular() {}        

// (Line 21-146) Define member functions of class Odometry that we claimed in header file
Odometry::Odometry(Monocular *cam): cam_(cam)
{
	focal_ = cam_->fx();
	pp_ = cv::Point2d(cam_->cx(), cam_->cy());
}

Odometry::~Odometry() {}

// Processing images(frames) in dataset. Note that the first frame could only serve as reference image  
// Use OpenCV method "findEssentialMat" and "recoverPose" for essential matrix and pose information 
void Odometry::imageProcessing(const Mat &img, int img_id)
{
	cur_img_ = img;

	if (img_id == 0)
	{
		featureDetection(cur_img_, ref_pts_); prev_img_ = cur_img_; return;
	}
	if (img_id == 1)
	{
		featureTracking(prev_img_, cur_img_, ref_pts_, cur_pts_, disparities_); 
		Mat E, R, t, mask;
		E = cv::findEssentialMat(cur_pts_, ref_pts_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
		cv::recoverPose(E, cur_pts_, ref_pts_, R, t, focal_, pp_, mask);
		cur_R_ = R.clone();
		cur_t_ = t.clone();
		ref_pts_ = cur_pts_;
		prev_img_ = cur_img_;
		return;
	}

	double scale = 1.00;    
	featureTracking(prev_img_, cur_img_, ref_pts_, cur_pts_, disparities_); 
	Mat E, R, t, mask;
	E = findEssentialMat(cur_pts_, ref_pts_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
	recoverPose(E, cur_pts_, ref_pts_, R, t, focal_, pp_, mask);
	scale = getRatio(img_id);
	if (scale > 0.1) 
	{
		cur_t_ = cur_t_ + scale*(cur_R_*t);
		cur_R_ = R*cur_R_;
	}
	if (ref_pts_.size() < kMinNumFeature)
	{
		featureDetection(cur_img_, ref_pts_);
		featureTracking(prev_img_, cur_img_, ref_pts_, cur_pts_, disparities_);
	}
	ref_pts_ = cur_pts_;

	prev_img_ = cur_img_;
}

// Obtaining the scaling factor from KITTI ground truth file 
// |r| = sqrt((x1-x0)^2+(y1-y0)^2+(z1-z0)^2)
double Odometry::getRatio(int img_id)
{
	string line;
	int i = 0;
	ifstream map_info("/home/mingxin/Downloads/KITTI/00/00.txt");
	double x = 0, y = 0, z = 0;
	double x_p, y_p, z_p;
	if (map_info.is_open())
	{
		while ((getline(map_info, line)) && (i <= img_id))
		{
			z_p = z;  x_p = x;  y_p = y;
			istringstream in(line);
			for (int j = 0; j < 12; j++)  {
				in >> z;
				if (j == 7) y = z;
				if (j == 3)  x = z;
			}
			i++;
		}
		map_info.close();
	}
	else {
		std::cerr<< "Unable to open file"; 
		return 0;
	}
	return sqrt((x - x_p)*(x - x_p) + (y - y_p)*(y - y_p) + (z - z_p)*(z - z_p));
}

// Use FAST method (cv::FAST) for feature points detection
void Odometry::featureDetection(Mat image, vector<Point2f> &px_vec)
{
	vector<KeyPoint> keypoints;
	int fast_threshold = 20;
	bool non_max_suppression = true;
	FAST(image, keypoints, fast_threshold, non_max_suppression);
	KeyPoint::convert(keypoints, px_vec);
}

// Adopt LK optical flow method (cv::calcOpticalFlowPyrLK) for feature tracking
// This is acutally a norm minimization problem   
void Odometry::featureTracking(Mat image_ref, Mat image_cur,
		    vector<Point2f> &px_ref, vector<Point2f> &px_cur, vector<double> &disparities)
{
	const double lk_win_size = 21.0;
	const int lk_max_iter = 30;
	const double lk_eps = 0.001;
	vector<uchar> status;
	vector<float> error;
	
	TermCriteria termcrit(TermCriteria::COUNT + TermCriteria::EPS, lk_max_iter, lk_eps);
	calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, px_cur, status, error,
		cv::Size2i(lk_win_size, lk_win_size), 4, termcrit, 0);

	vector<Point2f>::iterator px_ref_it = px_ref.begin();
	vector<Point2f>::iterator px_cur_it = px_cur.begin();
	disparities.clear(); 
	disparities.reserve(px_cur.size());
	for (size_t i = 0; px_ref_it != px_ref.end(); ++i)
	{
		if (!status[i])
		{
			px_ref_it = px_ref.erase(px_ref_it);
			px_cur_it = px_cur.erase(px_cur_it);
			continue;
		}
		disparities.push_back(norm(Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
		++px_ref_it;
		++px_cur_it;
	}
}

// The whole working process of our monocular visual odometry
int main(int argc, char *argv[])
{	
	// Set up a camera
	Monocular *camera = new Monocular(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157);
	Odometry visual(camera);

	ofstream out("pos.txt");        // output location data to pos.txt
	char text[100];
	int font_face = FONT_HERSHEY_PLAIN;  double font_scale = 1;  int thickness = 1;
	Point text_org(10, 50);
	namedWindow("Road condition", WINDOW_AUTOSIZE);
	namedWindow("Trajectory", WINDOW_AUTOSIZE);
	Mat traj = Mat::zeros(700, 700, CV_8UC3);  

	double x=0.0, y=0.0,z=0.0;
	for (int img_id = 0; img_id < maxFrame; ++img_id)
	{
		// Read and process image in KITTI dataset, sequence 00
		stringstream ss;
		ss << "/home/mingxin/Downloads/KITTI/00/image_3/" << setw(6) << setfill('0') << img_id << ".png";
		Mat img(imread(ss.str().c_str(), 0));
		assert(!img.empty());

		visual.imageProcessing(img, img_id);
		Mat cur_t = visual.getCurrentT();
		if (cur_t.rows!=0)
		{
			x = cur_t.at<double>(0);
			y = cur_t.at<double>(1);
			z = cur_t.at<double>(2);
		}
		out << x << " " << y << " " << z << std::endl;

		// Draw the trajectory on opened window
		int draw_x = int(x) + 350;
		int draw_y = int(-z) + 580;
		cv::circle(traj, Point(draw_x, draw_y), 1, CV_RGB(255, 0, 0), 2);    

		cv::rectangle(traj, Point(10, 30), Point(810, 100), CV_RGB(0, 0, 0), cv::FILLED);  
		std::sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x, y, z);
		cv::putText(traj, text, text_org, font_face, font_scale, Scalar::all(255), thickness, 8);
		
		// Although we process grayscale image, we still want to output original image
		Mat imageShow;
		imageShow = imread(ss.str().c_str(), 1);
		imshow("Road condition", imageShow);
		imshow("Trajectory", traj);
		waitKey(1);
	}
	return 0;
}
