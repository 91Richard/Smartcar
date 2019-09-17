#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <time.h>
#include <algorithm>
using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)

geometry_msgs::Twist laser_cmd;
geometry_msgs::Twist cmd_vel;
ros::Subscriber sub;
ros::Subscriber sub1;

ros::Publisher pub;
static int tunnel_num=0;
static int over_tunnel=0;
static int f_num=0;
static int pednum = 0;

struct point                //Œ«×ø±ê
{
    float rho;
    float theta;
};
bool cmp(point x, point y)
{
    return x.rho < y.rho;
}
bool judge_front(float theta)
{
    if(fabs(theta)>1.57){return true;}
    else return false;
}
bool in_rectangle(float x,float y,float a,float b)
{
    if(fabs(x)<a&&fabs(y)<b){return true;}
    else return false;

}
bool near_judge(float a,float b,float bound)
{
    if(fabs(a)+fabs(b)<bound)
    {
        return true;
    }
    else return false;
}
//const geometry_msgs::Twist::ConstPtr & cmd
void scanCallback1(const geometry_msgs::Twist::ConstPtr& cmd_vel)   // geometry_msgs::Twist>("cmd_vel", Twist, scanCallback1)
{
    if (cmd_vel->angular.x==1) over_tunnel=2;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)  //sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
{
    int count = scan->scan_time / scan->time_increment;
    int length = 0;
    float k1=2;
    float k2=0.8;//0.8
    float l1=0.5;
    float l2=0.5;//1.0;  //K进行左右的控制，L进行前后的控制，1和2分别是前半周和后半周
    float t1=0.2; //隧道 的转向控制
    float project_x=0;
    float project_y=0;
    float p_x=0;
    float p_y=0;
    float deta_y1=0;
    float deta_y2=0;
    float safe_x1=0.35;
    float safe_x2=0.5;
    float safe_y1=0.9;
    float safe_y2=0.35;
    float tunnel_x1=0;
    float tunnel_x2=0;
    clock_t start,finish;
    float total_time;
    float nearest_left=-10;
    float nearest_right=10;
    length=count;
    point point_array[length];

    //start=clock();
    for(int i = 0; i < count; i++) {
        float rad = scan->angle_min + scan->angle_increment * i;
        float x = scan->ranges[i] * sin(rad);
        //ROS_INFO("%f, %f", degree, scan->ranges[i]);
        //ROS_INFO("%d:", count);
        float y = scan->ranges[i] * cos(rad);
        //float degree = RAD2DEG(rad);
        point_array[i].rho = fabs(scan->ranges[i]);
        point_array[i].theta = rad;
        if(i>count*0.22&&i<count*0.28)//0.22,0.28
        {
            if(x>nearest_left)
            {
                nearest_left=x;
		
                
            }
        }
        if(i>count*0.82&&i<count*0.88)//0.82,0.88
        {
            if(x<nearest_right)
            {
                nearest_right=x;
		
            }
        }

    }

    project_x = 100;
    project_y = 100;

    float tt=10;
    //sort(point_array , point_array + length , cmp);
    for (int i=0;i<count;i++){
        p_x = point_array[i].rho * sin(point_array[i].theta);
        p_y = point_array[i].rho * cos(point_array[i].theta);
        if (fabs(p_x)<0.3 && p_y>-0.9 && p_y<-0.3 && (point_array[i].rho<tt)){ //or project_y>0)){
               project_x = p_x;
               project_y = p_y;
               tt=point_array[i].rho;
            }
        else if (fabs(p_x)<0.4 && p_y>0.0 && p_y<0.2 && point_array[i].rho<tt){ //&& project_y>0){
               project_x = p_x;
               project_y = p_y;
               tt=point_array[i].rho;
        }
    
    }
    
    if((near_judge(nearest_left,nearest_right,0.9)or(fabs(nearest_left-nearest_right)<0.1))and(over_tunnel==0))
    {
	
	laser_cmd.linear.z = 1.0;
        ROS_INFO("tunnel control");
        float k = 2.0;
        laser_cmd.angular.z = k * (nearest_left + nearest_right);
    	tunnel_num+=1;
	ROS_INFO("tunnelnum: %d",tunnel_num);

    }

    else if((near_judge(nearest_left,nearest_right,1.2)or(fabs(nearest_left-nearest_right)<0.2))and(over_tunnel==2))
    {
	
	laser_cmd.linear.z = 1.0;
        ROS_INFO("tunnel control");
        float k = 4.0;
        laser_cmd.angular.z = k * (nearest_left + nearest_right);
    //	tunnel_num+=1;
    //  ROS_INFO("tunnelnum: %d",tunnel_num);

    }

    
    //else if(judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.42,0.9))
    else if(tt<10 and over_tunnel==1 and (f_num==1 or project_y<-0.3))
    {
        if (project_y<-0.3){
            f_num= 1;
	    ROS_INFO("FRONT");
            ROS_INFO("projectXXXX: %f",project_x);
            ROS_INFO("projectYYYY: %f",project_y);
        
            laser_cmd.linear.z = 0;
            laser_cmd.angular.x=project_x;
            laser_cmd.angular.y=project_y;

            deta_y1=safe_y1+project_y;

            if(project_x>-0.1)
            {
      	    laser_cmd.angular.z = k1 * (project_x-safe_x1) - l1 * deta_y1;
            ROS_INFO("turn left!!!!");
            }
            else if(project_x<-0.1)
            {
            laser_cmd.angular.z = k1 * (safe_x1 + project_x) + l1 * deta_y1;
            ROS_INFO("turn right!!!!");
            }

        }
    //else if(!judge_front(point_array[0].theta)&&in_rectangle(project_x,project_y,0.55,0.01))
        else if(f_num==1) {
            ROS_INFO("BACK");
            ROS_INFO("x: %f",project_x);
        
            laser_cmd.linear.z = 1;
            deta_y2=project_y-0.2;

            if(project_x<0)//obstacle is on the left
            {
               laser_cmd.angular.z = k2 * project_x + l2 * deta_y2;
               ROS_INFO("turn left left!!!!");
            }
            else if(project_x>0)
            {
               laser_cmd.angular.z = k2 * project_x - l2 * deta_y2;
               ROS_INFO("turn right right!!!!");
            }
        }
    
    if (pednum >0) {
        pednum++;
        laser_cmd.linear.x = 0;
        pednum %= 15;
        }
    if (pednum==14) ROS_INFO("Done");
    }

    else
    {
        laser_cmd.linear.z = -1;
        ROS_INFO("camera control");
        if (tunnel_num>60) over_tunnel=1;

        laser_cmd.linear.y = over_tunnel; 
    }

    //finish=clock();
    //total_time=(float)(finish-start)/CLOCKS_PER_SEC;
    //ROS_INFO("TOTALTIME: %f",total_time);
    

    bool ped=false;
    bool obstacle_width[20] = {false};
    int index = 0;
    float nearest_y = -0.9;
    int current;
    for(int i = count*17/20; i <= count*23/20; ++i)
    {
        current = i%count;
        float rad = scan->angle_min + scan->angle_increment * current;
        float y = scan->ranges[current] * cos(rad);
        float x = scan->ranges[current] * sin(rad);
        
        if (fabs(x)<0.22 && y>-0.64+fabs(x)*1.1) ped=true;
    } 
   /*     if (obstacle_width[index] == false)
        {
            if (y > -0.6)//-0.9)
            {
                if (x < 0.15 - index * 0.015 && x > 0.15 - (index + 1) * 0.015)
                    obstacle_width[index] = true;
                if (y > nearest_y)
                    nearest_y = x;
            }
        }
        if (obstacle_width[index] == true) //(x < 0.15 - (index + 1) * 0.015 && y > -0.6)
            ++index;
        if (index == 20)
            break;
    }

    int width_count = 0;
    for(int i = 0; i < 20; ++i)
    {
         width_count += obstacle_width[i];
    }

    if (width_count > 2 && width_count <= 20&& over_tunnel==1)
    {     */
    if (ped == true && over_tunnel==1){
         laser_cmd.linear.x = 0.0;
         pednum =1;
         laser_cmd.angular.z = 0.0;
         laser_cmd.linear.z = 2;
         ROS_INFO("pedestrian______________________");
             

}
    else if (tt<10 and over_tunnel==1) laser_cmd.linear.x = 0.1;
    else laser_cmd.linear.x = 0.2;
//    else if (width_count <= 2 or over_tunnel==0)
//         laser_cmd.linear.x = 0.2-0.1*over_tunnel;
//    else
//        ROS_INFO("______________________");


/*    if (width_count >= 2)
       {laser_cmd.linear.x = 0.0;
        laser_cmd.angular.z = 0.0;}
    else
       {laser_cmd.linear.x = 0.1;}

*/
    
    
//    ROS_INFO("laser_cmd.linear.x: %f", laser_cmd.linear.x);
    pub.publish(laser_cmd);
    

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    sub1 = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, scanCallback1);
    //sub1 = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SimpleController::cmdCallback, this);
    pub = n.advertise<geometry_msgs::Twist>("laser_cmd", 1);//cmd_vel

    //laser_cmd.linear.x = 0.1;

    ros::spin();

    return 0;
}
