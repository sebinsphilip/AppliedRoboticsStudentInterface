#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>
#include <atomic>
#include <unistd.h>

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

    cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

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
    throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );
  }


void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
            const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
  }


}
