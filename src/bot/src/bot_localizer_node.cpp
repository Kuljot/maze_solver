#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <opencv2/opencv.hpp> 
#include <sstream>



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <geometry_msgs/msg/point.hpp>
// #include "bot_mapper.cpp"


using namespace std::chrono_literals;
using std::placeholders::_1;



class BotLocalizer : public rclcpp::Node
{
  public:
    BotLocalizer()
    : Node("bot_localizer_node")
    {
        this->graphified=false;
        this->crop_pixel=20;
        this->is_bg_extracted=false;
        this->one_pass_called=false;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/unit_box/image_raw", 10, std::bind(&BotLocalizer::image_callback, this, _1));
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("path_img", 1);
        location_publisher_=this->create_publisher<geometry_msgs::msg::Point>("bot_location_in_image", 1);
    }
  private:
    mutable bool is_bg_extracted=true;
    mutable int  orig_X=0;
    mutable int orig_Y=0;
    mutable int orig_rows=0;
    mutable int orig_cols=0;
    mutable int orig_rot=0;
    mutable cv::Point2f center_;
    mutable float radius_=0;
    mutable bool graphified=false;
    mutable bool one_pass_called=false;
    mutable int crop_pixel=5;
    mutable bool turn=false;
    mutable int top_left=0;
    mutable int top_rgt = 0;
    mutable int btm_left=0;
    mutable int btm_rgt = 0;
    mutable int top=0;
    mutable int rgt=0;
    mutable int btm=0;
    mutable int lft=0;
    mutable int no_of_pathways=0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr location_publisher_;

    cv::Mat connect_edges(cv::Mat img) const
    {
      //Kernel to be applied on the image
      cv::Mat kernel_=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(20,20));
      //Apply the kernel onto the image
      cv::Mat result;
      cv::morphologyEx(img,result,cv::MORPH_CLOSE,kernel_);
      return result;
    }

    int smallest_object_idx(std::vector<std::vector<cv::Point> > contours, int noise_thresh=20) const
    {

      int min_contour_area=100;
      int min_contour_idx=-1;

      for (size_t idx = 0; idx < contours.size(); idx++) {

        int area=cv::contourArea(contours[idx]);

        if ((area < min_contour_area) && (area > noise_thresh)){

          min_contour_area = area;
          min_contour_idx = idx;
        }
      RCLCPP_INFO(this->get_logger(), "Min Area [%d]",min_contour_area);        
      return min_contour_idx;
      }
    }
    cv::Mat extract_bg(cv::Mat frame) const
    {
        //Extract the mask of all rois(Region of Interest)
        //Convert the image to grayscale

        cv::Mat gray_image;
        cv::cvtColor(frame,gray_image,cv::COLOR_BGR2GRAY);
        cv::imshow("grey_image",gray_image);

        //Detect the edges from the grascale image using Canny Edge detector
        // Threshhold 1=50
        // Threshhold 2=150
        // aperture size=3
        cv::Mat edges;
        cv::Canny(gray_image,edges,25,80,3);
        //Connect the disjoint images that are close enough
        cv::imshow("edges",edges);
        edges=this->connect_edges(edges);
        //Find the contours in the image
        std::vector<std::vector<cv::Point> > contours_;
        cv::findContours(edges,contours_,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
        
        cv::Mat contourImage = cv::Mat::zeros(frame.size(), CV_8UC1);
        //Loop and get ids of the contours
            for (auto& cnt : contours_) {
                //-1 thickness
                cv::drawContours(contourImage, std::vector<std::vector<cv::Point>>{cnt}, 0, cv::Scalar(255), -1);
            }
        cv::imshow("Contours",contourImage);
        //Extract the background model by 
        //Removing the smallest object(bot from background)
        int min_contour_idx=this->smallest_object_idx(contours_);
        //Region of interset with no bot mask
        cv::Mat roi_no_bot_mask=contourImage.clone();

        //If the smallest Object in the foreground found
        if( min_contour_idx != -1)
        {
            //0 is to remove        
            cv::drawContours(roi_no_bot_mask, contours_, min_contour_idx, 0,-1);
            
            // Draw dilated bot mask
            cv::Mat bot_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
            cv::drawContours(bot_mask, contours_, min_contour_idx, 255,-1);
            cv::drawContours(bot_mask, contours_, min_contour_idx, 255,3);
            cv::imshow("Bot Mask",bot_mask);
            //Find the bounding circle
            cv::minEnclosingCircle(contours_[min_contour_idx],this->center_,this->radius_);

            cv::Mat not_car_mask;
            cv::bitwise_not(bot_mask,not_car_mask);
            cv::Mat frame_car_removed;
            cv::bitwise_and(frame,frame,frame_car_removed,not_car_mask);
            // Generate groud replica to fill in the area from where car is removed
            cv::Mat base_clr=cv::Mat::zeros(frame_car_removed.size(), frame_car_removed.type());
            cv::Mat ground_replica = cv::Mat::ones(frame_car_removed.size(), frame_car_removed.type());
            cv::Mat bg_model;

            cv::bitwise_and(ground_replica,ground_replica,bg_model,bot_mask);
            cv::bitwise_or(bg_model,frame_car_removed,bg_model);
        }
        cv::Mat maze_occupancy_grid = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::bitwise_not(roi_no_bot_mask,maze_occupancy_grid); 
        cv::imshow("Occupancy Grid",maze_occupancy_grid);
        return maze_occupancy_grid;
    }

    

    void image_callback(const sensor_msgs::msg::Image & msg) const
    {
    // Show Image inside a window 
      cv::Mat path_img;
      geometry_msgs::msg::Point pt;
      try
      {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        if (cv_ptr==nullptr){
          RCLCPP_INFO(this->get_logger(), "cv_ptr is nullptr");
        }
        else{
          cv::Mat img;
          cv::resize(cv_ptr->image,img,cv::Size(480,480),cv::INTER_LINEAR);
          img=this->extract_bg(img);
          cv::Mat clr_img;
          cv::cvtColor(img,clr_img,cv::COLOR_GRAY2BGR);
          RCLCPP_INFO(this->get_logger(), "Bot detected at X[%f] & Y[%f]",center_.x,center_.y);
          pt.x=center_.x;
          pt.y=center_.y;
          pt.z=0;
          cv::circle(clr_img,center_,radius_,(0,0,255),2);
          cv::imshow("Top Camera Image",clr_img);
          if(this->is_bg_extracted==false){
            path_img=this->graphify(img,this->is_bg_extracted).clone();
            this->is_bg_extracted=true;
          }
        }
        // std_msgs::msg::Header header; // empty header
        // cv_bridge::CvImage img_bridge;
        // sensor_msgs::msg::Image img_msg;
        // img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,path_img);
        // img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        // Convert the OpenCV image to a ROS2 sensor_msgs::Image message
        // cv::imshow("Path Image",path_img);
        // cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", path_img);
        // sensor_msgs::msg::Image::SharedPtr ros_image = cv_image.toImageMsg();
        // // Publish the ROS2 Image message
        // img_publisher_->publish(*ros_image);
        // img_publisher_->publish(img_msg);
        location_publisher_->publish(pt);

      }
      catch (cv_bridge::Exception &e)
      {
         RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      
      cv::waitKey(1);
        
    }

    void thinningIteration(cv::Mat& img, int iter) const
    {
        CV_Assert(img.channels() == 1);
        CV_Assert(img.depth() != sizeof(uchar));
        CV_Assert(img.rows > 3 && img.cols > 3);

        cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

        int nRows = img.rows;
        int nCols = img.cols;

        if (img.isContinuous()) {
            nCols *= nRows;
            nRows = 1;
        }

        int x, y;
        uchar *pAbove;
        uchar *pCurr;
        uchar *pBelow;
        uchar *nw, *no, *ne;    // north (pAbove)
        uchar *we, *me, *ea;
        uchar *sw, *so, *se;    // south (pBelow)

        uchar *pDst;

        // initialize row pointers
        pAbove = NULL;
        pCurr  = img.ptr<uchar>(0);
        pBelow = img.ptr<uchar>(1);

        for (y = 1; y < img.rows-1; ++y) {
            // shift the rows up by one
            pAbove = pCurr;
            pCurr  = pBelow;
            pBelow = img.ptr<uchar>(y+1);

            pDst = marker.ptr<uchar>(y);

            // initialize col pointers
            no = &(pAbove[0]);
            ne = &(pAbove[1]);
            me = &(pCurr[0]);
            ea = &(pCurr[1]);
            so = &(pBelow[0]);
            se = &(pBelow[1]);

            for (x = 1; x < img.cols-1; ++x) {
                // shift col pointers left by one (scan left to right)
                nw = no;
                no = ne;
                ne = &(pAbove[x+1]);
                we = me;
                me = ea;
                ea = &(pCurr[x+1]);
                sw = so;
                so = se;
                se = &(pBelow[x+1]);

                int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                        (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                        (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                        (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
                int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
                int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
                int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

                if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                    pDst[x] = 1;
            }
        }

        img &= ~marker;
    }

    /**
     * Function for thinning the given binary image
     *
     * Parameters:
     * 		src  The source image, binary with range = [0,255]
     * 		dst  The destination image
     */
    void thinning(const cv::Mat& src, cv::Mat& dst) const
    {
        dst = src.clone();
        dst /= 255;         // convert to binary image

        cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
        cv::Mat diff;

        do {
            this->thinningIteration(dst, 0);
            this->thinningIteration(dst, 1);
            cv::absdiff(dst, prev, diff);
            dst.copyTo(prev);
        } 
        while (cv::countNonZero(diff) > 0);

        dst *= 255;
    }

    

    cv::Mat one_pass(cv::Mat img) const
    {
        // Create a window to display  Detected Interested Points
        int rows =img.size[0];
        int columns =img.size[1];
        int row=0;
        int column=0;
        int node_no=0;
        int cnt=0;
        std::vector<cv::Point2f> end_pts;
        std::vector<cv::Point2f> tri_pts;
        std::vector<cv::Point2f> turn_pts;
        // Graph g;
        cv::Mat out_img=img.clone();
        cv::cvtColor(img,out_img,cv::COLOR_GRAY2BGR);
        for (row=0;row<rows;row++)
        {
            for (column=0;column<columns;column++)
            {
                if(img.at<uchar>(column,row)==255)
                {
                    // std::cout<<img.size[0]<<"&"<<img.size[1]<<"&"<<img.at<int>(row/2,column/2)<<std::endl;
                    no_of_pathways=this->get_surround_pixel_intensities(img,row,column);
                    // std::cout<<no_of_pathways<<std::endl;
                    cv::Point2f center;
                    center.x=row;
                    center.y=column;
                    // bool added=g.add_node(center,no_of_pathways);
                    if(no_of_pathways==1)
                    {
                        if(end_pts.size()==0)
                        {
                            end_pts.push_back(center);
                            cv::putText(out_img,std::to_string(end_pts.size()),center,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255));
                            cv::circle(out_img,center,10,cv::Scalar(0,0,255),2);
                        }
                        else
                        {
                            bool present=false;
                            for(int i=0;i<end_pts.size();i++)
                            {
                                if(cv::norm(end_pts.at(i)-center)<4){
                                    present=true;
                                    break;
                                }
                            }
                            if(present==false)
                            {
                                end_pts.push_back(center);
                                cv::putText(out_img,std::to_string(end_pts.size()),center,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255));
                                cv::circle(out_img,center,10,cv::Scalar(0,0,255),2);
                            }
                        }
                        
                    }
                    if(no_of_pathways==5)
                    {
                        if(tri_pts.size()==0)
                        {
                            tri_pts.push_back(center);
                            cv::circle(out_img,center,10,cv::Scalar(255,0,0),2);
                        }
                        else
                        {
                            bool present=false;
                            for(int i=0;i<tri_pts.size();i++)
                            {
                                if(cv::norm(tri_pts.at(i)-center)<10){
                                    present=true;
                                    break;
                                }
                            }
                            if(present==false)
                            {
                                tri_pts.push_back(center);
                                // cv::putText(out_img,std::to_string(tri_pts.size()),center,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,0,0));
                                cv::circle(out_img,center,10,cv::Scalar(255,0,0),2);
                            }
                        }
                    }
                    if(no_of_pathways==2 && this->turn)
                    {
                        if(turn_pts.size()==0)
                        {
                            turn_pts.push_back(center);
                            cv::circle(out_img,center,10,cv::Scalar(0,255,0),0.2);
                        }
                        else
                        {
                            bool present=false;
                            for(int i=0;i<turn_pts.size();i++)
                            {
                                if(cv::norm(turn_pts.at(i)-center)<10){
                                    present=true;
                                    break;
                                }
                            }
                            if(present==false)
                            {
                                turn_pts.push_back(center);
                                // cv::putText(out_img,std::to_string(tri_pts.size()),center,cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(255,0,0));
                                cv::circle(out_img,center,10,cv::Scalar(0,255,0),0.2);
                            }
                        }
                        this->turn=false;
                    }
                }
            }
        }
        
        // for (int i=0;i<g.graph.size();i++){
        //     std::cout<<"GRAPH"<<i<<g.graph[i]<<std::endl;
        // }
        return out_img;
    }

    int get_surround_pixel_intensities(cv::Mat img, int curr_row, int curr_col) const
    {
        //cv::threshold(img,img,1,1,cv::THRESH_BINARY);
        
        int rows=img.size[0];
        int columns=img.size[1];
        // std::cout<<rows<<"&"<<columns<<std::endl;

        //State variables to check if our point is at the boundary condition
        bool top_row=false;
        bool btm_row=false;
        bool lft_col=false;
        bool rgt_col=false;
        int top_left=0;
        int top_rgt = 0;
        int btm_left=0;
        int btm_rgt = 0;
        int top=0;
        int rgt=0;
        int btm=0;
        int lft=0;


        //Check if it is a boundary condition
        if (curr_row==0)
        {
            top_row=true;
        }
        if (curr_row==(rows-1))
        {
            // Bottom row == row below is not accesible
            btm_row=true;
        }
        if (curr_col==0)
        {
            // Left col == col to left not accesible
            lft_col=true;
        }
        if (curr_col==(columns-1))
        {
            // Right col == col to right is not accesible
            rgt_col=true;
        }
        //  Extracting surround pixel intensities and Addressing boundary conditions (if present)
        if (top_row || lft_col)
        {
            top_left = 0;
        }
        else
        {
            top_left = img.at<uchar>(curr_col-1,curr_row-1);
        }
        if( top_row || rgt_col )
        {
            top_rgt = 0;
        }
        else
        {
            top_rgt = img.at<uchar>(curr_col+1,curr_row-1);
        }

        if( btm_row or lft_col )
        {
            btm_left = 0;
        }
        else
        {
            btm_left = img.at<uchar>(curr_col-1,curr_row+1);
        }

        if( btm_row or rgt_col )
        {
            btm_rgt = 0;
        }
        else
        {
            btm_rgt = img.at<uchar>(curr_col+1,curr_row+1);
        }
        
        // If the point we are at is anywhere on the top row, Then
        //              ===> Its top pixel is definitely not accesible
        if (top_row)
        {
            top = 0;
        }
        else
        {
            top = img.at<uchar>(curr_col,curr_row-1);
        }
        if (rgt_col)
        {
            rgt = 0;
        }
        else
        {
            rgt = img.at<uchar>(curr_col+1,curr_row);
        }
        
        if (btm_row)
        {
            btm = 0;
        }
        else
        {
            btm = img.at<uchar>(curr_col,curr_row+1);
        }

        if (lft_col)
        {
            lft = 0;
        }
        else
        {
            lft = img.at<uchar>(curr_col-1,curr_row);
        }

        int no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         )/255; 
        if(no_of_pathways==2)
        {
            if ((top==255 && btm==255) || (lft==255 && rgt==255) || (btm_left==255 && top_rgt==255) || (btm_rgt==255 && top_left==255))
            {
                this->turn=false;
            }
            else
            {
                this->turn=true;
            }
        }      
        return no_of_pathways;
    }
    

    cv::Mat graphify(cv::Mat extracted_maze, bool is_one_pass_called) const
    {
        std::cout<<"Graphipy Called"<<std::endl;
        cv::Mat cropped_img;
        if (not this->graphified)
        {
            // Thinning Operation on the maze
            cv::Mat thinned_img;
            this->thinning(extracted_maze,thinned_img);
            //Dilate and perform thinning again to remove unneccesary turns
            cv::Mat thinned_dilated_img;
            cv::Mat kernel_=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10));
            cv::morphologyEx(thinned_img,thinned_dilated_img,cv::MORPH_CLOSE,kernel_);
            cv::threshold(thinned_dilated_img,thinned_dilated_img,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
            this->thinning(thinned_dilated_img,thinned_img);
            //Remove the boundary pixels
            cv::Rect region_of_interest(this->crop_pixel,this->crop_pixel,480-2*this->crop_pixel,480-2*this->crop_pixel);
            cropped_img=thinned_dilated_img(region_of_interest);
            //Pass it to the one_pass function to find the nodes
            cv::Mat junction;
            if(one_pass_called==false)
            {
                junction=this->one_pass(cropped_img);
                cv::imshow("Junctions",junction);
            }
        }
        cv::imshow("Cropped img",cropped_img);
        //Save the image
        std::string file_path = "/home/kuljot/Project/maze_solver/maze_solver/src/bot/maps/cropped_image.png"; // Specify the file path
        cv::imwrite(file_path, cropped_img);

        // cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", cropped_img);
        // sensor_msgs::msg::Image::SharedPtr ros_image = cv_image.toImageMsg();
        // // Publish the ROS2 Image message
        // this->img_publisher_->publish(*ros_image);
        return cropped_img;
    }

    
    
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BotLocalizer>());
  rclcpp::shutdown();
  return 0;
}

