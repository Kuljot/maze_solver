#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
// #include <numpy.hpp>

#include <opencv2/opencv.hpp> 
// #include <opencv2/ximgproc.hpp>

#include "thinning.cpp"



class BotMapper
{
    public:
    BotMapper()
    {
        this->graphified=false;
        this->crop_pixel=5;

    }

    void one_pass(cv::Mat img)
    {
        cv::Mat maze_bgr;
        cv::cvtColor(img,maze_bgr,cv::COLOR_GRAY2BGR);
        // Create a window to display  Detected Interested Points
        cv::namedWindow("Maze interest points",cv::WINDOW_FREERATIO);
        int rows =img.size[0];
        int columns =img.size[1];
        for (int row=0;row<rows;row++)
        {
            for (int column=0;column<columns;column++)
            {
                if(img.at<int>(row,column)==255)
                {

                }
            }
        }

    }

    void get_surround_pixel_intensities(cv::Mat img, int curr_row, int curr_col)
    {
        cv::threshold(img,img,1,1,cv::THRESH_BINARY);
        int rows=img.size[0];
        int column=img.size[1];

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
        if (curr_row==(rows-1))
        {
            // Right col == col to right is not accesible
            rgt_col=true;
        }
        //  Extracting surround pixel intensities and Addressing boundary conditions (if present)
        if (top_row or lft_col)
        {
            top_left = 0;
        }
        else
        {
            top_left = img.at<int>(curr_row-1,curr_col-1);
        }
        if( top_row or rgt_col )
        {
            top_rgt = 0;
        }
            
        else
        {
            top_rgt = img.at<int>(curr_row-1,curr_col+1);
        }

        if( btm_row or lft_col )
        {
            btm_left = 0;
        }
        else
        {
            btm_left = img.at<int>(curr_row+1,curr_col-1);
        }

        if( btm_row or rgt_col )
        {
            btm_rgt = 0;
        }
        else
        {
            btm_rgt = img.at<int>(curr_row+1,curr_col+1);
        }
        
        // If the point we are at is anywhere on the top row, Then
        //              ===> Its top pixel is definitely not accesible
        if (top_row)
        {
            top = 0;
        }
        else
        {
            top = img.at<int>(curr_row-1,curr_col);
        }
        if (rgt_col)
        {
            rgt = 0;
        }
        else
        {
            rgt = img.at<int>(curr_row,curr_col+1);
        }
        
        if (btm_row)
        {
            btm = 0;
        }
        else
        {
            btm = img.at<int>(curr_row+1,curr_col);
        }

        if (lft_col)
        {
            lft = 0;
        }
        else
        {
            lft = img.at<int>(curr_row,curr_col-1);
        }

        int no_of_pathways = ( top_left + top      + top_rgt  +
                           lft      + 0        + rgt      + 
                           btm_left + btm      + btm_rgt        
                         );
        if(no_of_pathways>2)
        {  
            std::cout<<"  [ top_left , top      , top_rgt  ,lft    , rgt      , btm_left , btm      , btm_rgt   ] \n [ "<<top_left<<" , "<<top<<" , "<<top_rgt<<" ,\n   "<<lft<<" , "<<"-"<<" , "<<rgt<<" ,\n   "<<btm_left<<" , "<<btm<<" , "<<btm_rgt<<" ] "<<std::endl;
            std::cout<<"\nno_of_pathways [row,col]= [ "<<curr_row<<" , "<<curr_col<<" ] "<<no_of_pathways<<std::endl; 

        }
        this->top_left=top_left;
        this->top=top;
        this->top_rgt=top_rgt;
        this->rgt=rgt;
        this->btm_rgt=btm_rgt;
        this->btm=btm;
        this->btm_left=btm_left;
        this->lft=lft;
        this->no_of_pathways=no_of_pathways;
    }
    

    void graphify(cv::Mat extracted_maze)
    {
        if (not this->graphified)
        {
            //cv::imshow("Extracted Maze",extracted_maze);
            // Thinning Operation on the maze
            cv::Mat thinned_img;
            thinning(extracted_maze,thinned_img);
            //Dilate and perform thinning again to remove unneccesary turns
            cv::Mat thinned_dilated_img;
            cv::Mat kernel_=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(2,2));
            cv::morphologyEx(thinned_img,thinned_dilated_img,cv::MORPH_CLOSE,kernel_);
            cv::threshold(thinned_dilated_img,thinned_dilated_img,0,255,cv::THRESH_BINARY | cv::THRESH_OTSU);
            thinning(thinned_dilated_img,thinned_img);
            cv::imshow("Thined img",thinned_dilated_img);
            //Remove the boundary pixels
            cv::Rect region_of_interest(this->crop_pixel,this->crop_pixel,480-this->crop_pixel,480-this->crop_pixel);
            cv::Mat cropped_img=thinned_dilated_img(region_of_interest);
            cv::imshow("Cropped img",cropped_img);
            //Overlay Map on the maze
            this->one_pass(cropped_img);
        }
    }

    private:
    bool graphified=false;
    int crop_pixel=5;
    int top_left=0;
    int top_rgt = 0;
    int btm_left=0;
    int btm_rgt = 0;
    int top=0;
    int rgt=0;
    int btm=0;
    int lft=0;
    int no_of_pathways=0;
};