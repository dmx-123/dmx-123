#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

// Design a camera class that contains all the parameters
class Monocular
{
    public:
	Monocular(double width, double height, double fx, double fy, double cx, double cy,
		double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
	~Monocular();

	inline int width() const { return width_; }
	inline int height() const { return height_; }
	inline double fx() const { return fx_; };
	inline double fy() const { return fy_; };
	inline double cx() const { return cx_; };
	inline double cy() const { return cy_; };
	inline double k1() const { return d_[0]; };
	inline double k2() const { return d_[1]; };
	inline double p1() const { return d_[2]; };
	inline double p2() const { return d_[3]; };
	inline double k3() const { return d_[4]; };

private:
	double width_, height_;  
	double fx_, fy_;      
	double cx_, cy_;      
	bool distortion_;     
	double d_[];         
};

// Design an odometry class and claim its member functions
// The member functions specified what our odometry are supposed to do 
class Odometry
{
    public:
        Odometry(Monocular *cam);
        ~Odometry();
        void imageProcessing(const Mat &img, int img_id);
        Mat getCurrentT() { return cur_t_; }

    protected:
        double getRatio(int img_id);
        void featureDetection(Mat image, vector<Point2f> &pts_vec);
        void featureTracking(Mat image_ref, Mat image_cur,
		    vector<Point2f> &ref_pts, vector<Point2f> &cur_pts, vector<double> &disparities);
    
    protected:         
    	Mat cur_img_;                     
	    Mat prev_img_;                    
	    Mat cur_R_;
	    Mat cur_t_;
	    vector<Point2f> ref_pts_;        //reference pixel points
	    vector<Point2f> cur_pts_;        //current pixel points
	    vector<double> disparities_;    
	    double focal_;
	    Point2d pp_;
        Monocular *cam_;
};