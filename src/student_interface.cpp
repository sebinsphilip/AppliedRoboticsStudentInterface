#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>
#include <atomic>
#include <unistd.h>

#define MIN_AREA_SIZE 30
#define FIND_ROBOT_DEBUG_PLOT 0
#define FIND_OBSTRACLES_DEBUG_PLOT 0
#define FIND_VICTIM_DEBUG_PLOT 0

namespace student {

 void loadImage(cv::Mat& img_out, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT LOADED" );
    std::string filepath;
    filepath.append (config_folder);
    filepath.append ("/img.jpg");
    std::cout << "The file path to open: " << filepath.c_str() << std::endl;
    img_out = cv::imread(filepath.c_str(), cv::IMREAD_COLOR ); // Read the file
    if( img_out.empty() ) // Check for invalid input
    {
	    std::cout << "Could not open or find the image" << std::endl ;
    }

 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
    static bool firstEntry = true;
    static std::string output_folder_path , output_folder_name;
    static int id = 0;
    if (firstEntry)
    {
    std::cout << "Enter output folder name (rewrites):";
    std::cin >> output_folder_name;
    output_folder_path.append (config_folder);
    output_folder_path.append ("/");
    output_folder_path.append (output_folder_name);
    output_folder_path.append ("/");
    std::stringstream folder_command;
    folder_command << "mkdir -p " << output_folder_path;
    std::cout << "output folder path to save:" << output_folder_path << std::endl;
    std::system (folder_command.str().c_str());
    firstEntry = false;
    }
    cv::imshow (topic, img_in);
    char c;
    c = cv::waitKey(30);

    std::stringstream img_file;
    switch (c) {
		case 's':
			img_file <<output_folder_path  << std::setfill('0')
					<< std::setw(3)  << (id++) << ".jpg";
			cv::imwrite( img_file.str(), img_in );

			std::cout << "Saved image " << img_file.str() << std::endl;
			break;
		default:
				break;
    }


  }


    // Defintion of the function pickNPoints and the callback mouseCallback.
    // The function pickNPoints is used to display a window with a background
    // image, and to prompt the user to select n points on this image.
    static cv::Mat bg_img;
    static std::vector<cv::Point2f> result;
    static std::string name;
    static std::atomic<bool> done;
    static int n;
    static double show_scale = 1.0;

    void mouseCallback(int event, int x, int y, int, void* p)
    {
      if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

      result.emplace_back(x*show_scale, y*show_scale);
      cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
      cv::imshow(name.c_str(), bg_img);

      if (result.size() >= n) {
        usleep(500*1000);
        done.store(true);
      }
    }

    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
    {
      result.clear();
      cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
      cv::resize(img, bg_img, small_size);
      //bg_img = img.clone();
      name = "Pick " + std::to_string(n0) + " points";
      cv::imshow(name.c_str(), bg_img);
      cv::namedWindow(name.c_str());
      n = n0;

      done.store(false);

      cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
      while (!done.load()) {
        cv::waitKey(500);
      }

      cv::destroyWindow(name.c_str());
      return result;
    }


  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );

    std::vector<cv::Point2f> image_points;

    image_points = pickNPoints(4, img_in);

    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    return ok;

  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );

    cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane,
                        const std::vector<cv::Point2f>& dest_image_points_plane,
                        cv::Mat& plane_transf, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }


void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
            const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }



bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){

    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask;
    cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

    // Preparing the kernel matrix
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

    // Filter (applying an erosion and dilation) the image
    cv::erode(red_mask, red_mask, kernel);
    cv::dilate(red_mask, red_mask, kernel);


    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process red mask
    contours_img = hsv_img.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i=0; i<contours.size(); ++i)
    {
        //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
        approxPolyDP(contours[i], approx_curve, 7, true);

        Polygon scaled_contour;
        for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }
        obstacle_list.push_back(scaled_contour);
#if FIND_OBSTRACLES_DEBUG_PLOT
        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
        std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
#endif
    }
#if FIND_OBSTRACLES_DEBUG_PLOT
    std::cout << std::endl;
    cv::imshow("Original", contours_img);
    cv::imshow("red mask", red_mask);
    cv::waitKey(0);
#endif

    return true;
}

bool processVictimsGate(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate){

    // Find green regions
    cv::Mat green_mask;
    int appr_curve_size = 0;

    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

#if FIND_VICTIM_DEBUG_PLOT
    cv::Mat contours_img;
    contours_img = hsv_img.clone();
#endif

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    int victim_id = 0;
    for (int i=0; i<contours.size(); ++i)
    {

        const double area = cv::contourArea(contours[i]);

        if(area < 500) continue;

        //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
        approxPolyDP(contours[i], approx_curve, 10, true);
        appr_curve_size = approx_curve.size();
        if(appr_curve_size < 4) continue;
#if FIND_VICTIM_DEBUG_PLOT
        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
        std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
#endif
        if(appr_curve_size == 4)
        {
            //we found the rectangular gate
            for (const auto& pt: approx_curve) {
                gate.emplace_back(pt.x/scale, pt.y/scale);
            }

            continue;
        }
        // Here its definetly the circular victims

        Polygon scaled_contour;
        for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }
        victim_list.push_back({victim_id++, scaled_contour});
    }
#if FIND_VICTIM_DEBUG_PLOT
    cv::imshow("Victims", contours_img);
    cv::imshow("green mask", green_mask);
    cv::waitKey(0);
#endif

    return true;
}


bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);


    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processVictimsGate(hsv_img, scale, victim_list, gate);
    if(!res2) std::cout << "processVictimsGate return false" << std::endl;
    return res1 && res2;

}

bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
	//throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );
	// Convert color space from BGR to HSV
	bool found = false;
	cv::Mat hsv_img;
	cv::cvtColor(img_in, hsv_img,
			cv::COLOR_BGR2HSV);

	// Extract blue color region
	cv::Mat blue_mask;
	cv::inRange(hsv_img, cv::Scalar(100, 120, 150),
			cv::Scalar(135, 255, 255), blue_mask);
	// Filter (applying erosion and dilation) the image
	cv::Mat  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
	cv::erode(blue_mask, blue_mask, kernel);
	cv::dilate(blue_mask, blue_mask, kernel);

	// Find blue mask contours
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point> approx_curve;
	cv::findContours(blue_mask, contours,
			cv::RETR_EXTERNAL,
			cv::CHAIN_APPROX_SIMPLE);

#if FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
	cv::imshow("findRobotHsv", hsv_img);
	cv::waitKey (0);
	cv::imshow("findRobotMask", blue_mask);
	cv::waitKey (0);
	cv::Mat contours_img;
	contours_img = img_in.clone();
	cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
	cv::imshow("contours", contours_img);
	cv::waitKey (0);
#endif

	for (int i=0; i<contours.size(); i++)
	{


		// Approximate the i-th contours
		cv::approxPolyDP(contours[i],
				approx_curve, 30, true);

		// Check the number of edge of the aproximate contour
		if (approx_curve.size() != 3) continue;

		// If area is too low, then skip
		double area = cv::contourArea(approx_curve);
		if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives

#if FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
		std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
		std::cout << (i+1) << ") Aprox Contour size: " << approx_curve.size() << std::endl;
		std::cout << "Area: " << area << std::endl;

		cv::drawContours(contours_img, approx_curve, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA);
#endif
		found = true;  // we found the blue triangle exit!
		break;
	}
	// If we found the robot triangle set robot position and create the triangle poligon
	if (found)
	{
		// Find the position of the robot
		// NB: the position of the robot coincide with the baricenter of the triangle
		double cx = 0, cy = 0;
		double tmp_cx = 0, tmp_cy = 0;
		// emplace back every vertex on triangle (output of this function)
		for (const auto& pt: approx_curve) {
			tmp_cx = pt.x/scale; cx += tmp_cx;
			tmp_cy = pt.y/scale; cy += tmp_cy;
			triangle.emplace_back(tmp_cx, tmp_cy);
			// remember to use the scale to convert the position on the image
			// (pixels) to the position in the arena (meters)
		}

		cx /= 3; // 'found' will be only true for a triangle, hence hardcoding 3
		cy /= 3;

		// Find the robot orientation (i.e the angle of height relative to the base with the x axis)
		double dst = 0;
		Point top_vertex; //
		for (auto& vertex: triangle)
		{
			const double dx = vertex.x-cx;
			const double dy = vertex.y-cy;
			const double curr_d = dx*dx + dy*dy;
			if (curr_d > dst)
			{
				dst = curr_d;
				top_vertex = vertex;
			}
		}

		// Store the position of the robot in the output
		x = cx;
		y = cy;

		// Compute the robot orientation
		const double dx = cx - top_vertex.x;
		const double dy = cy - top_vertex.y;
		theta = std::atan2(dy, dx);

#if FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
		// Draw over the imag ethe ba
		cv::Mat contours_img;
		cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
		cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
		cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
		cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
		cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);
		std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << std::endl;
#endif
	}

#if  FIND_ROBOT_DEBUG_PLOT  // do this only if FIND_DEBUG_PLOT is defined
	cv::imshow("findRobot", contours_img);
	cv::waitKey(0);
#endif

	return found;

}

bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
	throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
}


}
