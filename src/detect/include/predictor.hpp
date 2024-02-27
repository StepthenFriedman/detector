#pragma once

#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cfg.hpp"

namespace predict{
    struct armor_pos{
        double x,y,z,
            rx,ry,rz;
        armor_pos operator+(const armor_pos& b){
            armor_pos arm;
            arm.x= this->x + b.x;
            arm.y= this->y + b.y;
            arm.z= this->z + b.z;
            arm.rx= this->rx + b.rx;
            arm.ry= this->ry + b.ry;
            arm.rz= this->rz + b.rz;
            return arm;
        }
        armor_pos operator-(const armor_pos& b){
            armor_pos arm;
            arm.x= this->x - b.x;
            arm.y= this->y - b.y;
            arm.z= this->z - b.z;
            arm.rx= this->rx - b.rx;
            arm.ry= this->ry - b.ry;
            arm.rz= this->rz - b.rz;
            return arm;
        }
        public:
        void print_armor_pos(){
            printf("position:(%.2f %.2f %.2f) rotation:(%.2f %.2f %.2f)\n",this->x,this->y,this->z,this->rx,this->ry,this->rz);
        }
    };

    char* ipt;

    int read(){
        char c=*(ipt++);int x=0,f=1;
        while(c<'0'||c>'9'){if(c=='-')f=-1;c=*(ipt++);}
        while(c>='0'&&c<='9'){x=x*10+c-'0';c=*(ipt++);}
        return x*f;
    }

    double read_lf(){
        char c=*(ipt++);double x=0,flag=0;
        char f=1;
        while(c<'0'||c>'9'){if(c=='-')f=-1;c=*(ipt++);}
        while((c>='0'&&c<='9')||c=='.'){
            if (c=='.') flag=10;
            else{if (flag){x+=(double)(c-'0')/flag;flag*=10;}
                 else x=x*10.+c-'0';}
            c=*(ipt++);
        }
        return x*f;
    }

    armor_pos list[2][10];

    int count=0;

    armor_pos get_new_pos(armor_pos prev,armor_pos now){
        return now+now-prev;
    }

    void get_armor_pos(const std_msgs::msg::String::SharedPtr msg){
        ipt=(char*)msg->data.c_str();
        int n=read();
        for (int i=0;i<n;i++){
            int j=read();
            list[count%2][j].x=read_lf();
            list[count%2][j].y=read_lf();
            list[count%2][j].z=read_lf();
            list[count%2][j].rx=read_lf();
            list[count%2][j].ry=read_lf();
            list[count%2][j].rz=read_lf();
            armor_pos pred=get_new_pos(list[!(count%2)][j],list[count%2][j]);
            printf("id: %d ",j);
            pred.print_armor_pos();
        }
        memset(list[!(count%2)],0,sizeof(armor_pos)*10);
        count++;
        count%=10;
    }
}

