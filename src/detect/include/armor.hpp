#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "cfg.hpp"
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>

#ifdef DEBUG
#include <iostream>
#include <cstdio>
#endif

/*
    contains armor, light and functions.
*/

namespace detect{
    int cmpy (const cv::Point2f & a, const cv::Point2f & b) {return a.y < b.y;}
    cv::Point2f y_max(cv::Point2f vertices[4]){
        cv::Point2f res=vertices[0];
        for (int i=1;i<4;i++) if (res.y<vertices[i].y) res=vertices[i];
        return res;
    }
    cv::Point2f y_min(cv::Point2f vertices[4]){
        cv::Point2f res=vertices[0];
        for (int i=1;i<4;i++) if (res.y>vertices[i].y) res=vertices[i];
        return res;
    }

    double camera_matrix_data[9]={
        1257.12121, 0.,         711.11997,
        0.,         1256.42896, 562.49495,
        0.,         0.,         1.
    };
    
    double distCoeffs_data[5]={0.,0.,0.,0.,0.};
    double pooling_kernal_data[1600];
    double* x=std::fill_n(pooling_kernal_data,1600,1);
    
    cv::Mat camera_matrix(3, 3, CV_64F, camera_matrix_data);
    cv::Mat distCoeffs(1, 5, CV_64F, distCoeffs_data);
    cv::Mat pooling_kernal(40, 40, CV_64F, pooling_kernal_data);
    class Light{
	public:
	    float angle,wid,hei;
        cv::Point2f center, vertices[4];
		Light(const cv::RotatedRect rect){
            angle=rect.angle;
            if (rect.size.height<rect.size.width) {
                hei=rect.size.width;wid=rect.size.height;
            }else {
                hei=rect.size.height;wid=rect.size.width;
                angle+=90.;
            }
            center=rect.center;
            rect.points(vertices);
        };
        int is_light(cv::Mat* frame){
            if (hei/wid<LIGHT_MIN_RATIO) return 0;
            if (!(0 <= center.x && 0 <= wid && center.x + wid <= frame->cols && 0 <= center.y && 0 <= hei && center.y + hei <= frame->rows)) return 0;
            cv::Mat roi(*frame,cv::Rect(center.x,center.y,wid,hei));
            cv::Scalar res = cv::sum(roi);
            #if COLOR == RED
                return (res[0]*COLOR_RATIO < res[2]);
            #else
                return (res[0] > res[2]*COLOR_RATIO);
            #endif
        }
        #ifdef DEBUG
        void draw(cv::Mat* frame){
            for (int j=0;j<4;j++) cv::line(*frame, vertices[j], vertices[(j+1)%4], cv::Scalar(0,255,0), 2);
        }
        #endif
    };
    int cmpx (const std::shared_ptr<Light> & a, const std::shared_ptr<Light> & b) {return a->center.x < b->center.x;}
    class Armor{
	public:
        unsigned int id;
        std::vector<cv::Point2f> vertices={cv::Point2f(0.,0.),cv::Point2f(0.,0.),cv::Point2f(0.,0.),cv::Point2f(0.,0.)};
        cv::Point2f center;
        std::vector<cv::Mat> channels;
        std::shared_ptr<Light> l1,l2;
        cv::Rect boundRect;
        double wid,hei,angle;
        double mse_arr[9];

        Armor(std::shared_ptr<Light> light1,std::shared_ptr<Light> light2){
            id=0;
            if (light1->center.x>light2->center.x) {l2=light1;l1=light2;}
            else {l1=light1;l2=light2;}
        }
        int is_armor(){
            hei=(l1->hei+l2->hei)/2.;
            wid=absolute((l1->center.x)-(l2->center.x),(l1->center.y)-(l2->center.y));
            if ((absdiv(l1->hei, l2->hei) > ARMOR_HEIGHT_ERROR) || (hei/wid > ARMOR_RATIO_MAX) || y_max(l1->vertices).y<y_min(l2->vertices).y || y_max(l2->vertices).y<y_min(l1->vertices).y) return 0;
            center=(l1->center+l2->center)/2.;
            angle=(l1->angle+l2->angle)/2.;
            return 1;
        }
        void get_boarders(){
            std::sort(l1->vertices,l1->vertices+4,cmpy);
            std::sort(l2->vertices,l2->vertices+4,cmpy);
            vertices[0]=(l1->vertices[0]+l1->vertices[1])*.5;
            vertices[0]=vertices[0]+(vertices[0]-l1->center);

            vertices[1]=(l1->vertices[2]+l1->vertices[3])*.5;
            vertices[1]=vertices[1]+(vertices[1]-l1->center);

            vertices[2]=(l2->vertices[2]+l2->vertices[3])*.5;
            vertices[2]=vertices[2]+(vertices[2]-l2->center);


            vertices[3]=(l2->vertices[1]+l2->vertices[0])*.5;
            vertices[3]=vertices[3]+(vertices[3]-l2->center);

            auto x= (vertices[1]-vertices[2])*ARMOR_NUMBER_WID_RATIO;
            vertices[1]-=x;vertices[2]+=x;

            x= (vertices[0]-vertices[3])*ARMOR_NUMBER_WID_RATIO;
            vertices[0]-=x;vertices[3]+=x;
        }
        #if IDENTIFY_METHOD == MSE
        int get_number(cv::Mat frame,std::vector<cv::Point2f> trans,cv::Mat* digits){
            cv::Mat temp;
            cv::warpPerspective(frame, temp, cv::getPerspectiveTransform(vertices, trans), cv::Size(TRANS_WID,TRANS_HEI));
            cv::split(temp, channels);
            cv::Mat_<double> gray=(channels)[1],mse;
            double mean=cv::mean(gray)[0]/2,mse_arr[5];
            cv::threshold(gray, gray, mean, 255, cv::THRESH_BINARY);
            for (int i=0;i<5;i++){
                cv::subtract(gray,digits[i],mse);
                cv::multiply(mse,mse,mse);
                mse_arr[i]=cv::mean(mse)[0];
            }
            double minMSE=mse_arr[0];
            for (int i=1;i<5;i++) if (mse_arr[i]<minMSE) minMSE=mse_arr[id=i+1];
            if (minMSE>IDENTIFY_THRESH) id=0;
            return id!=0;
        }
        #endif
        #ifdef DEBUG
        void draw(cv::Mat* frame){
            cv::circle(*frame, center, 3, cv::Scalar(255,0,0));
            cv::putText(*frame, std::to_string(id), center, cv::FONT_HERSHEY_DUPLEX,1.0, cv::Scalar(255,0,0));
            for (int j=0;j<4;j++) cv::line(*frame, vertices[j], vertices[(j+1)%4], cv::Scalar(255,0,0), 2);
        }
        #endif
        void PnP(std::vector<cv::Point3f> real_armor,std_msgs::msg::String& msg,cv::Mat& rvecs,cv::Mat& tvecs){
            
            printf("armor: ");
            for (int i=0;i<4;i++) printf("(%.2f %.2f) ",vertices[i].x,vertices[i].y);
            putchar('\n');
            cv::solvePnP(real_armor,vertices,camera_matrix,distCoeffs,rvecs,tvecs,false,cv::SOLVEPNP_AP3P);
            msg.data+=std::to_string(id)+" ";
            for(int row = 0; row < rvecs.rows; ++row) {
                double* p = rvecs.ptr<double>(row);
                msg.data+=std::to_string(*p)+' ';
            }
            for(int row = 0; row < tvecs.rows; ++row) {
                double* p = tvecs.ptr<double>(row);
                msg.data+=std::to_string(*p)+' ';
            }
            #ifdef DEBUG
            std::cout<<msg.data<<'\n';
            #endif
        }
    };
}