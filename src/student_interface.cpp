#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "planning_ompl.hpp"
#include "clipper.hpp"
#include "dubinsMP.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>
#include <atomic>
#include <unistd.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

//#include "ompl-1.5.1/src/ompl/geometric/planners/rrt/RRTstar.h"
// include <ompl/geometric/planners/rrt/RRTstar.h>

#define MIN_AREA_SIZE 30
#define FIND_ROBOT_DEBUG_PLOT 0
#define FIND_OBSTRACLES_DEBUG_PLOT 0
#define FIND_VICTIM_DEBUG_PLOT 0
#define FIND_VICTIM_OCR_DEBUG 0
#define DEBUG 0
#define DUBINS_SAMPLING_SIZE 20
#define DUBINS_K_MAX 15
#define RRT_STAR_FOLDER_PATH "/tmp/path/"

// namespace om = ompl::geometric::RRTstar;


namespace student {


//-------------------------------------------------------------------------
//          LOAD IMAGE IMPLEMENTATION
//-------------------------------------------------------------------------

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

 //-------------------------------------------------------------------------
 //          GENERIC IMAGE LISTENER IMPLEMENTATION
 //-------------------------------------------------------------------------

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

  //-------------------------------------------------------------------------
  //          EXTRINSIC CALIB IMPLEMENTATION - MOUSE CALLBACK, PICK N POINTS
  //-------------------------------------------------------------------------

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

  //-------------------------------------------------------------------------
  //          EXTRINSIC CALIB IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Perform extrinsic camera calibration by using the user points and the solvePnP function

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );

    std::vector<cv::Point2f> image_points;

    image_points = pickNPoints(4, img_in);

    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    return ok;

  }

  //-------------------------------------------------------------------------
  //          UNDISTORT IMAGE IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Undistort the image based on the distortion coefficients and the camera matrix

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );

    cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

  }

  //-------------------------------------------------------------------------
  //          FIND PLANE TRANSFORM IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Find the plane transform for the perspective calculations, based on the chosen calibration points from

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

  //-------------------------------------------------------------------------
  //          UNWARP IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Unwarp images by changing their perspective

  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
            const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  //-------------------------------------------------------------------------
  //          PROCESS OBSTACLES IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Scan the map and find obstacles based on shape and color

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

  /*Function to sort victim list based on recognised OCR index*/

  bool sort_victim (const std::pair<int,Polygon> &a,
                        const std::pair<int,Polygon> &b)
  {
          return (a.first < b.first);
  }

  //-------------------------------------------------------------------------
  //          PROCESS VICTIMS GATE IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Scan the map and find victims by their shape and color

  bool processVictimsGate(const cv::Mat& img_in, const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate){

    cv::Mat green_mask;
    int appr_curve_size = 0;
    // Find green regions
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);
    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));


    //OCR specific filtering
    cv::Mat green_mask_ocr;
    cv::inRange(hsv_img, cv::Scalar(45, 30, 30), cv::Scalar(75, 255, 255), green_mask_ocr);
    cv::dilate(green_mask_ocr, green_mask_ocr, kernel);
    cv::erode(green_mask_ocr, green_mask_ocr, kernel);
    cv::Mat green_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask_ocr, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
#if FIND_VICTIM_OCR_DEBUG
    cv::imshow("Numbers", green_mask_inv);
    cv::waitKey(0);
#endif
    // Create Tesseract object
    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    // Initialize tesseract to use English (eng)
    ocr->Init(NULL, "eng");
    // Set Page segmentation mode to PSM_SINGLE_CHAR (10)
    ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
    // Only digits are valid output characters
    ocr->SetVariable("tessedit_char_whitelist", "12345");
    img_in.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes

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
        kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
        cv::Rect boundRect = boundingRect(cv::Mat(approx_curve)); // find bounding box for each green blob
        cv::Mat processROI(filtered, boundRect); // extract the ROI containing the digit

        if (processROI.empty()) continue;

        cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
        cv::threshold( processROI, processROI, 100, 255, 0 ); // threshold and binarize the image, to suppress some noise

        // Apply some additional smoothing and filtering
        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);

#if FIND_VICTIM_OCR_DEBUG
        // Show the actual image passed to the ocr engine
        cv::imshow("ROI", processROI);
        cv::waitKey(0);
#endif

        // Set image data
        ocr->SetImage(processROI.data, processROI.cols, processROI.rows, 3, processROI.step);
        int num = -1, max_conf=0, conf=0;
        std::string num_string, num_string_confident;
        cv::Mat processROI_temp(processROI);
        cv::Mat processROI_prev(processROI);
        for (int h = 0; h< 8; h++)
        {
            if (4 == h)
            {
                cv::flip(processROI, processROI_temp, 1);
            }
            if (0 != (h%4))
            {
                cv::rotate(processROI_prev, processROI_temp, (h-1)%4);
            }
            ocr->SetImage(processROI_temp.data, processROI_temp.cols, processROI_temp.rows, 3, processROI_temp.step);
            num_string = std::string(ocr->GetUTF8Text());
            conf = ocr->MeanTextConf();
            if (max_conf < conf)
            {
                max_conf = conf;
                num_string_confident = num_string;
            }

            processROI_prev = processROI_temp;
#if FIND_VICTIM_OCR_DEBUG
            std::cout << "Recognized digit: " << num_string<< "size" << num_string.size() << "comf:" << conf <<std::endl;
            cv::imshow("ROI", processROI);
            cv::waitKey(0);
#endif
        }
        if (std::isdigit (num_string_confident[0]))
        {
            victim_id = atoi (num_string_confident.c_str());
            std::cout << "VIctim id detected:" << victim_id <<std::endl;
        }
        else
        {
            std::cout << "Error in finding victim OCR!!" << std::endl;
            continue;
        }
        for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }
        victim_list.push_back({victim_id, scaled_contour});
    }
    //close OCR
    ocr->End();
    sort(victim_list.begin(), victim_list.end(), sort_victim);
#if FIND_VICTIM_DEBUG_PLOT
    cv::imshow("Victims", contours_img);
    cv::imshow("green mask", green_mask);
    cv::waitKey(0);
#endif

    return true;
  }

  //-------------------------------------------------------------------------
  //          PROCESS MAP IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Processes the Obstacles and the Victim-gate of the map

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Process Obstacles
    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    // Process Victims-gate
    const bool res2 = processVictimsGate(img_in, hsv_img, scale, victim_list, gate);
    if(!res2) std::cout << "processVictimsGate return false" << std::endl;
    return res1 && res2;

  }

  //-------------------------------------------------------------------------
  //          FIND ROBOT IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Find the position and orientation of the robot

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
                std::vector<cv::Point2f> triangle_p(3);
                cv::Point2f robot_circle_temp;
                float robot_radius_temp;
  		// emplace back every vertex on triangle (output of this function)
  		for (const auto& pt: approx_curve) {
  			tmp_cx = pt.x/scale; cx += tmp_cx;
  			tmp_cy = pt.y/scale; cy += tmp_cy;
  			triangle.emplace_back(tmp_cx, tmp_cy);
                        triangle_p.emplace_back (tmp_cx, tmp_cy);
  			// remember to use the scale to convert the position on the image
  			// (pixels) to the position in the arena (meters)
  		}
                // For plygon offsetting, find robot min. enclosing circle radius
                cv::minEnclosingCircle (triangle_p, robot_circle_temp, robot_radius_temp);
                //robot_radius_temp += (robot_radius_temp)/2;
                std::ofstream outputFile("/tmp/robot_radius.txt");
                outputFile << robot_radius_temp << " " << scale;
                cv::imwrite("/tmp/map.jpg", img_in);
                outputFile.close();
                //std::cout <<"Robo-coordinates" <<robot_circle_temp.x << " " << robot_circle_temp.y << " " << robot_radius_temp << std::endl;

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

  void drawDubinsCurve (const Path pth1, Path &path, float& theta_intermediate)
  {
      for (int i = 0; i<= DUBINS_SAMPLING_SIZE; i++)
      {
          Pose p;
          float dubins_strip = pth1.points[0].s/DUBINS_SAMPLING_SIZE*i;
          circline(dubins_strip, pth1.points[0].x, pth1.points[0].y, pth1.points[0].kappa, pth1.points[0].theta, p);
          path.points.emplace_back (dubins_strip, p.x, p.y, p.theta,pth1.points[0].kappa);
          theta_intermediate = p.theta;
          //std::cout << pth1.points[0].s/100*i << " " << p.x << " " <<  p.y << " " <<  p.theta << " " << pth1.points[0].kappa << std::endl;
      }

  }

 //-------------------------------------------------------------------------
 //          PLAN PATH IMPLEMENTATION
 //-------------------------------------------------------------------------

 bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate,
                const float x, const float y, const float theta, Path& path,
                const std::string& config_folder){
     float radius, gatex, gatey;
     cv::Point2f circle;
     std::vector<float> radius_list;
     Polygon circle_list;
     DubinsPathType path_enum;
     Pose p01(0,0, 0, 0,0), p11(0,0,0,0,0);
     Pose p02(0,0, 0, 0,0), p12(0,0,0,0,0);
     Pose p03(0,0, 0, 0,0), p13(0,0,0,0,0);
     std::vector<Pose> vect_Pose_1({p01, p11});
     std::vector<Pose> vect_Pose_2({p02, p12});
     std::vector<Pose> vect_Pose_3({p03, p13});
     Path pth1(vect_Pose_1), pth2(vect_Pose_2), pth3(vect_Pose_3);
     float L;
#if DEBUG
     std::cout << "x,y,theta:" << x << ", " << y << ",theta: " << theta << std::endl;
     for (int i=0; i<borders.size(); ++i)
     {
         std::cout << (i+1) << ") border: " << borders[i].x << "," << borders[i].y << std::endl;
     }
#endif
     std::vector<cv::Point2f> gate_p(gate.size());
     for (int i=0; i<gate.size(); i++)
     {
#if DEBUG
         std::cout << (i+1) << ") gate: " << gate[i].x << "," << gate[i].y << std::endl;
#endif
         gate_p[i].x = gate[i].x;
         gate_p[i].y = gate[i].y;
     }
     minEnclosingCircle (gate_p, circle, radius);
#if DEBUG
     std::cout << "gate_circle:" << circle.x << " " << circle.y << "radius:" << radius <<std::endl;
#endif
     gatex = circle.x; gatey = circle.y;
     //Read the robot radius
     float robot_radius = 0.0;
     double scale = 0.0;
     cv::Mat map;
     std::ifstream outputFile("/tmp/robot_radius.txt");
     outputFile >> robot_radius >> scale;
     outputFile.close();
     map = cv::imread("/tmp/map.jpg", cv::IMREAD_COLOR ); // Read the file
     if( map.empty() ) // Check for invalid input
     {
         std::cout << "Could not open or find the image" << std::endl ;
     }
     else
     {
     //cv::imshow("mapo", map);
     //cv::waitKey(0);
     }
#if DEBUG
     std::cout << "Robot radius found:" << robot_radius << "scale:" << scale << std::endl;
#endif

     float theta_temp = 0;
     for (int i=0; i<obstacle_list.size(); ++i)

     {
         std::vector<cv::Point2f> obstacle_p(obstacle_list[i].size());
#if DEBUG
         std::cout << "-------------- OBSTRACLE LIST -------------------" << std::endl;
#endif
         for (int j=0; j<obstacle_list[i].size(); ++j)
         {
#if DEBUG
             std::cout << (j+1) << ") poly: " << (obstacle_list[i])[j].x << "," << (obstacle_list[i])[j].y << std::endl;
#endif
             obstacle_p[j].x = (obstacle_list[i])[j].x;
             obstacle_p[j].y = (obstacle_list[i])[j].y;
         }
         minEnclosingCircle (obstacle_p, circle, radius);
#if DEBUG
         std::cout  << circle.x << "," << circle.y << "radius:" << radius <<std::endl;
#endif
         //radius += robot_radius;
#if DEBUG
         std::cout << "radius after ofsetting:" << radius << std::endl;
#endif
         Point temp(circle.x, circle.y);
         circle_list.push_back(temp);
         radius_list.push_back(radius);
         cv::circle(map, cv::Point(circle.x*scale, circle.y*scale), radius*scale, cv::Scalar(0,0,0));
     }
     //cv::imshow("mapl", map);
     //cv::waitKey(0);
     float prev_goal_x  = x;
     float prev_goal_y = y;
     theta_temp = theta;
     float ang = 0;
     float theta_intermediate = 0;
     int skip_index = 0, first_try = 1;
     std::string full_path = RRT_STAR_FOLDER_PATH;
     boost::filesystem::path dir(RRT_STAR_FOLDER_PATH);
     boost::filesystem::remove_all(dir);
     if(!(boost::filesystem::exists(dir))){
         boost::filesystem::create_directory(dir);
     }
     for (int i=0; i<victim_list.size(); ++i)

     {
         full_path = RRT_STAR_FOLDER_PATH;
         std::vector<cv::Point2f> victim_p(victim_list[i].second.size());
         full_path.append(std::to_string(victim_list[i].first));
         full_path.append(".txt");

#if DEBUG
         std::cout << "-------------- VICTIM LIST-------------------:" << victim_list[i].first << "File path:"<<full_path <<std::endl;
#endif
         for (int j=0; j<victim_list[i].second.size(); ++j)
         {
#if DEBUG
             std::cout << (j+1) << ") poly: " << ((victim_list[i]).second)[j].x << "," << ((victim_list[i]).second)[j].y << std::endl;
#endif
             victim_p[j].x = ((victim_list[i]).second)[j].x;
             victim_p[j].y = ((victim_list[i]).second)[j].y;
         }
         minEnclosingCircle (victim_p, circle, radius);
#if DEBUG
         std::cout << "victim_circle:" << circle.x << " " << circle.y << "radius:" << radius <<std::endl;
#endif
         /* Plan motion from last victim/start point to next victim (local goal) */
         plan (1, PLANNER_RRTSTAR, OBJECTIVE_WEIGHTEDCOMBO, full_path, borders, prev_goal_x, prev_goal_y, circle.x, circle.y, circle_list, radius_list );
         std::ifstream planFile(full_path);
        skip_index = 0;
        first_try = 1;
        while (planFile.peek() != EOF)
         {
             float planx, plany, planx1, plany1;
             if (first_try)
             {
                 planFile >> planx >> plany;
                 prev_goal_x = planx;
                 prev_goal_y = plany;
             }
             planFile >> planx1 >> plany1;
             planFile >> planx1 >> plany1;
             planFile >> planx1 >> plany1;
             if (planx1 == prev_goal_x)
                 break;
             std::cout << prev_goal_x << " " << prev_goal_y << std::endl;
             std::cout << planx1<< " " << plany1 << std::endl << std::endl;
             //skip_index++;
             //if (0 == skip_index%2)
             //    continue;
             //std::cout << planx << " " << plany << std::endl;
             //std::cout << planx1 << " " << plany1 << std::endl;
             //dubins (planx, plany,0,
             //        planx1, plany1, 0,
             //        10, path_enum, pth1, pth2, pth3, L);
             //path.push_back(pth1.points);
             //path.push_back(pth2.points);
             //path.push_back(pth3.points);
             cv::line (map, cv::Point(prev_goal_x*scale, prev_goal_y*scale), cv::Point(planx1*scale, plany1*scale), cv::Scalar(0,0,0));
             ang = calctheta (prev_goal_x, prev_goal_y, planx1, plany1);
             //ang = atan2((plany1-circle.y),(planx1-circle.x));
             dubins (prev_goal_x, prev_goal_y, theta_temp,
                     planx1, plany1,ang,
                     DUBINS_K_MAX, path_enum, pth1, pth2, pth3, L);
             drawDubinsCurve (pth1, path, theta_intermediate);
             drawDubinsCurve (pth2, path, theta_intermediate);
             drawDubinsCurve (pth3, path, theta_intermediate);
             prev_goal_x = planx1; prev_goal_y = plany1;

             std::cout << "theta:[" << theta_temp <<  std::endl;
             std::cout << "angle:[" << ang <<  std::endl;
             std::cout << "circlex:[" << circle.x <<  std::endl;
             std::cout << "circley:[" << circle.y <<  std::endl;
             std::cout << "planx:[" << planx <<  std::endl;
             std::cout << "plany:[" << plany <<  std::endl;

             //theta_temp = theta_intermediate;
             theta_temp = ang;
             first_try = 0;
#if DEBUG

             std::cout << "i:[" << i << "] pth1.points.x:" << pth1.points[0].x << " pth1.points.y:" << pth1.points[0].y << " pth1.points.theta:"
                 << pth1.points[0].theta << "pth1.points.s:" << pth1.points[0].s << "pth1.points.kappa:" << pth1.points[0].kappa << std::endl;
             std::cout << "i:[" << i << "] pth2.points.x:" << pth2.points[0].x << " pth2.points.y:" << pth2.points[0].y << " pth2.points.theta:"
                 << pth2.points[0].theta << "pth2.points.s:" << pth2.points[0].s << "pth2.points.kappa:" << pth2.points[0].kappa << std::endl;
             std::cout << "i:[" << i << "] pth3.points.x:" << pth3.points[0].x << " pth3.points.y:" << pth3.points[0].y << " pth3.points.theta:"
                 << pth3.points[0].theta << "pth3.points.s:" << pth3.points[0].s << "pth3.points.kappa:" << pth3.points[0].kappa << " L:" << L<< "path:" <<path_enum << std::endl;
#endif
         }
         planFile.close ();
     }
     full_path = RRT_STAR_FOLDER_PATH;
     full_path.append("goal.txt");
     /* Plan motion from last victim to goal*/
     plan (1, PLANNER_RRTSTAR, OBJECTIVE_WEIGHTEDCOMBO, full_path, borders, prev_goal_x, prev_goal_y, gatex, gatey, circle_list, radius_list );
     std::ifstream planFile(full_path);
     skip_index = 0;
     first_try = 1;
     std::cout << "thetab4:[" << theta_temp <<  std::endl;
     while (planFile.peek() != EOF)
     {
         float planx, plany, planx1, plany1;
         if (first_try)
         {
             planFile >> planx >> plany;
             prev_goal_x = planx;
             prev_goal_y = plany;
         }
         planFile >> planx1 >> plany1;
         planFile >> planx1 >> plany1;
         planFile >> planx1 >> plany1;
         if (planx1 == prev_goal_x)
             break;
         //skip_index++;
         //if (0 == skip_index%2)
         //    continue;
         //std::cout << planx << " " << plany << std::endl;
         //std::cout << planx1 << " " << plany1 << std::endl;
         //dubins (planx, plany,0,
         //        planx1, plany1, 0,
         //        10, path_enum, pth1, pth2, pth3, L);
         //path.push_back(pth1.points);
         //path.push_back(pth2.points);
         //path.push_back(pth3.points);
         cv::line (map, cv::Point(prev_goal_x*scale, prev_goal_y*scale), cv::Point(planx1*scale, plany1*scale), cv::Scalar(0,0,0));
         //dubins (gatex, gatey, theta_intermediate,
         std::cout << prev_goal_x << " " << prev_goal_y << std::endl;
         std::cout << planx1<< " " << plany1 << std::endl << std::endl;
         ang = calctheta (prev_goal_x, prev_goal_y, planx1, plany1);
         dubins (prev_goal_x, prev_goal_y, theta_temp,
                 planx1, plany1, ang,
                 DUBINS_K_MAX, path_enum, pth1, pth2, pth3, L);
         drawDubinsCurve (pth1, path, theta_intermediate);
         drawDubinsCurve (pth2, path, theta_intermediate);
         drawDubinsCurve (pth3, path, theta_intermediate);
         prev_goal_x = planx1; prev_goal_y = plany1;
         theta_temp = ang;
         first_try = 0;
         std::cout << "thetain:[" << theta_temp <<  std::endl;
#if DEBUG
         std::cout << "i:["  << "] pth1.points.x:" << pth1.points[0].x << " pth1.points.y:" << pth1.points[0].y << " pth1.points.theta:"
             << pth1.points[0].theta << "pth1.points.s:" << pth1.points[0].s << "pth1.points.kappa:" << pth1.points[0].kappa << std::endl;
         std::cout << "i:["  << "] pth2.points.x:" << pth2.points[0].x << " pth2.points.y:" << pth2.points[0].y << " pth2.points.theta:"
             << pth2.points[0].theta << "pth2.points.s:" << pth2.points[0].s << "pth2.points.kappa:" << pth2.points[0].kappa << std::endl;
         std::cout << "i:["<< "] pth3.points.x:" << pth3.points[0].x << " pth3.points.y:" << pth3.points[0].y << " pth3.points.theta:"
             << pth3.points[0].theta << "pth3.points.s:" << pth3.points[0].s << "pth3.points.kappa:" << pth3.points[0].kappa << " L:" << L<< "path:" <<path_enum << std::endl;
#endif
     }
     planFile.close ();
     // cv::imshow("mapl", map);
     // cv::waitKey(0);


     return true;
 }



}
