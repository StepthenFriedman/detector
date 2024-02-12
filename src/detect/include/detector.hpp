#pragma once
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "cfg.hpp"
#include "armor.hpp"

#ifdef DEBUG 
#include <iostream>
#endif

namespace detect{
    int isInQuads (cv::Point2f quads[4],std::shared_ptr<Light> l){
        ///check whether light l is inside quads(a,b,c,d).
        ///check 2 points: the highest and the lowest of light l.
        int clockwise,i;cv::Point2f checkpoint,vectors[4];

        checkpoint=y_max(l->vertices);
        for (i=0;i<4;i++){
            vectors[i]=cv::Point2f(getvec(checkpoint,quads[i]));
        }
        clockwise = crossproduct(vectors[3],vectors[0]);
        for (int i=0;i<3;i++){
            if (crossproduct(vectors[i],vectors[i+1])!=clockwise) goto check2;
        }
        return 1;
        check2:
        checkpoint=y_min(l->vertices);
        for (i=0;i<4;i++){
            vectors[i]=cv::Point2f(getvec(checkpoint,quads[i]));
        }
        clockwise = crossproduct(vectors[3],vectors[0]);
        for (int i=0;i<3;i++){
            if (crossproduct(vectors[i],vectors[i+1])!=clockwise) return 0;
        }
        return 1;
    }

    int contain_lights(int i,int j,std::shared_ptr<Light>* lights){
        ///check whether there is another light k between light i and j.
        ///if so, an armor cannot be formed from i and j.
        if (j==i+1) return 0;
        cv::Point2f quads[4]={y_max(lights[i]->vertices),y_min(lights[i]->vertices),y_min(lights[j]->vertices),y_max(lights[j]->vertices)};
        for (int k=i+1;k<j;k++){
            if (isInQuads(quads,lights[k])) return 1;
        }
        return 0;
    }
    class Detector{
	public:
        std::vector<cv::Point2f> transform={
            cv::Point2f(TRANS_WID,0.),cv::Point2f(TRANS_WID,TRANS_HEI),cv::Point2f(0.,TRANS_HEI),cv::Point2f(0.,0.),
        };
        std::vector<cv::Point3f> armor_world_pos={
            cv::Point3f(TRANS_HEI,0.,0.),cv::Point3f(TRANS_HEI,TRANS_WID,0.),cv::Point3f(0.,TRANS_WID,0.),cv::Point3f(0.,0.,0.),
        };
        std::vector<std::vector<cv::Point>> contours;
        std::shared_ptr<Light> lights[LIGHT_ARRAY_LENGTH];
        std::shared_ptr<Armor> armors[ARMOR_ARRAY_LENGTH];

        cv::Mat_<double> decimal[5];
        int light_count,armor_count;
        std::vector<cv::Vec4i> hierarchy;

        #if PREPROCESSOR == CHANNELSUB
        std::vector<cv::Mat> channels;
        cv::Mat color,other_color,color_around,
            color_contour;
        #elif PREPROCESSOR == BINARY_THRESHOLDING
        cv::Mat src_gray;
        #endif
		Detector(){
            #if IDENTIFY_METHOD == MSE
            for (int i=0;i<5;i++){
                cv::Mat temp=cv::imread(DECIMAL_PATH(i+1),cv::IMREAD_GRAYSCALE);
                cv::Size sz=temp.size();
                cv::Rect cop=cv::Rect(sz.width/2.-TRANS_WID/2.,sz.height/2.-TRANS_HEI/2.,TRANS_WID,TRANS_HEI);
                cv::Mat_<double> copped=temp(cop);
                cv::threshold(copped,copped,100,10,cv::THRESH_BINARY);
                decimal[i]=copped;
            }
            #endif
        };
        void run(cv::Mat frame,std_msgs::msg::String& message){
            cv::Mat rvecs(1,3,CV_64F),tvecs(1,3,CV_64F);
            message.data="";
            contours.clear();
            light_count=0;
            armor_count=0;
            cv::RotatedRect r_rect;
            std::shared_ptr<Light> light;
            std::shared_ptr<Armor> armor;

            #if PREPROCESSOR == BINARY_THRESHOLDING
            cv::cvtColor(frame,src_gray,cv::COLOR_BGR2GRAY);
            cv::threshold(src_gray,src_gray, BINARY_THRESHOLD, 100 ,cv::THRESH_BINARY);

            #elif PREPROCESSOR == CHANNELSUB
            
            cv::split(frame, channels);
            cv::threshold(channels[COLOR],color,COLOR_BINARY_THRESHOLD,255,cv::THRESH_BINARY);
            cv::threshold(channels[(COLOR+1)%3],other_color,OTHER_COLOR_BINARY_THRESHOLD,1,cv::THRESH_BINARY_INV);
            cv::multiply(color,other_color,color);
            cv::threshold(channels[(COLOR+2)%3],other_color,OTHER_COLOR_BINARY_THRESHOLD,1,cv::THRESH_BINARY_INV);
            cv::multiply(color,other_color,color);
            
            cv::filter2D(color,color_around,-1,pooling_kernal,cv::Point(3,3),0,cv::BORDER_CONSTANT);
            cv::threshold(color,color_contour,1,1,cv::THRESH_BINARY_INV);
            cv::threshold(color_around,color_around,1,1,cv::THRESH_BINARY);
            cv::multiply(channels[1],color_contour,channels[1]);
            cv::multiply(channels[1],color_around,channels[1]);
            cv::threshold(channels[1],channels[1],BINARY_THRESHOLD,255,cv::THRESH_BINARY);

            #endif
            
            cv::findContours(channels[1], contours, hierarchy, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
            for( size_t i = 0; i< contours.size(); i++){
                r_rect = cv::minAreaRect(contours[i]);
                light = std::make_shared<detect::Light>(r_rect);
                if (light->is_light(&frame)) {
                    lights[light_count++]=light;
                    #ifdef DEBUG
                    light->draw(&frame);
                    if (light_count>=LIGHT_ARRAY_LENGTH-1) std::cout<<"overflow!\n";
                    #endif
                }
            }
            
            std::sort(lights,lights+light_count,cmpx);
            
            for (int i=0;i<light_count-1;i++) {
                int maxx=lights[i]->center.x+(lights[i]->hei)*ARMOR_RATIO_MAX;
                for (int j=i+1;j<light_count; j++){
                    if (lights[j]->center.x > maxx) break;
                    if (abssub(lights[i]->angle, lights[j]->angle)>LIGHT_ANGLE_ERR || contain_lights(i,j,lights)) continue;
                    armor = std::make_shared<detect::Armor>(lights[i],lights[j]);
                    armor->get_boarders();
                    armor->draw(&frame);
                    if (armor->is_armor()) {
                        
                        if (armor->get_number(frame,transform,decimal)){
                            armors[armor_count++]=armor;
                            #if IDENTIFY_METHOD == MSE
                            #endif
                        }
                    }
                }
            }
            message.data+=std::to_string(armor_count)+' ';
            for (int i=0;i<armor_count;i++) armors[i]->PnP(armor_world_pos,message,rvecs,tvecs);
            if (!frame.empty()) imshow("frame", frame),imshow("color",channels[1]);
            else std::cout<<"Frame empty\n";
            cv::waitKey(10);
		}
	};
    
}