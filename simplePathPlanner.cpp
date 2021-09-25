 
/*========================================
   Purpose: Obstacle avoidance in Robot-Robot interaction
      Date: 17/09/2020  
 ========================================*/

/*========================================
 Necessary libraries.
 ========================================*/
#include <iostream> 
#include <fstream>  /*C*++ from/to file */
#include <string>   /*C++ string management*/
#include <math.h>   /*C++ Mathematical operation*/
#include <vector>   /*C++ vector operation*/

/*PCL library*/
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

/*Ypspur and sensor*/
#include <ypspur.h>
#include <scip2awd.h>
#include <thread>   /*Thread operation*/
#include <mutex>    /* Thread variable management*/
#include <chrono>   /* Clock time*/
#include <signal.h> /* Keyboard event*/

/*Matrix library*/
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::MatrixXf;

using namespace std;
using namespace std::chrono; 

/* ========================================
Function definitions and global variable declarations
========================================*/
int gIsShuttingDown;
bool scannedDataReady = false;

void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}

struct urg_struct  /* Structure to store URG data */
{
    double x, y;
    urg_struct(double paramx, double paramy) : x(paramx), y(paramy) {}
};

struct viaPoints_struct  /* Structure to store via_points */
{
    double via_x, via_y,s1s2;
    viaPoints_struct(double paramx, double paramy,double params1s2) : via_x(paramx), via_y(paramy), s1s2(params1s2) {}
};

struct obstacle_struct  /* Structure to store points where the obstacle follows*/
{
    double x, y,t,r; /* center of object(x,y), the time and radius*/
    obstacle_struct(double paramx, double paramy,double paramt, double paramr) : x(paramx), y(paramy), t(paramt), r(paramr) {}
};

struct select_viaPoints_struct  /* Structure to store via_points */
{
    double x, y,t_via,t,s1s2;
    select_viaPoints_struct(double paramx, double paramy,double paramt_via,double paramt,double params1s2) : x(paramx), y(paramy),t_via(paramt_via), t(paramt),s1s2(params1s2) {}
};

struct cluster_struct  /* Structure to store Cluster information*/
{
    double d,x,y,r,xi,yi,xf,yf; /* Distance from robot,Mean-x position, Mean-y position, its radius*/
    int s; /*Cluster size*/
    cluster_struct(double paramd,int params, double paramx,double paramy, double paramr, double paramxi, double paramyi,double paramxf,double paramyf) : d(paramd), s(params), x(paramx), y(paramy), r(paramr),xi(paramxi),yi(paramyi),xf(paramxf),yf(paramyf) {}
};

/*Global variables.*/
vector<urg_struct> urg_vector;          /*Vector for storing Urg data*/
vector<urg_struct> urg_vector_copy;     /*Vector for storing copy of urg_vector*/
vector<urg_struct> urg_vector_buffered; /*Vector for storing copy of urg_vector_copy*/

double odom_x, odom_y, odom_theta;                      /* Robot's (x,y,theta) and Robot velocity and angular velocity*/
high_resolution_clock::time_point program_starting_time;        /*The time when program starts*/
  
vector<viaPoints_struct> viaPoints_vector;    /*A vector that stores the viapoints*/
vector<select_viaPoints_struct> select_viaPoints_vector;
vector<select_viaPoints_struct> tmp_select_viaPoints_vector; 
vector<obstacle_struct> obstacle_vector;      /*Vector that stores the Objects path*/
vector<cluster_struct> cluster_vector;        /*Vector that stores Cluster information*/

/* ========================================
Avoidance Algorithm thread function definition
========================================*/

void avoidance_algorithm_thread(){

  /*=================== Is Spur opened? =====================*/
  signal(SIGINT, ctrl_c);
  /*Intialize YP spur variables*/
  setvbuf( stdout, 0, _IONBF, 0 );
  if(Spur_init() < 0 )
  {
    fprintf(stderr, "[*] ERROR : cannot open spur.\n");
  }else{
    /*=================== Spur is opened, Intialize Spur variables and some other variable intializations=====================*/
    Spur_set_vel( 0.2);
    Spur_set_accel( 1.0 );
    Spur_set_angvel( M_PI/4.0);
    Spur_set_angaccel( M_PI /2.0);
    Spur_set_pos_LC( 0.0, 0.0, 0.0);
    Spur_set_pos_GL ( 0.0 , 0.0 , 0.0 );

    cluster_vector.reserve(350);  /*The maximum number of cluster we might have at one scan time*/
    obstacle_vector.reserve(15);
    viaPoints_vector.reserve(15);
    select_viaPoints_vector.reserve(15);
    tmp_select_viaPoints_vector.reserve(2);
    urg_vector_buffered.reserve(681); /*Buffered vector of URG data*/

    ofstream myfile;
    myfile.open("../log_ex_10.txt");
   

    double cluster_x_mean,cluster_y_mean,cluster_to_robot_distance,obstacle_radius = 0;/*temporary variables when calculating the closest cluster*/
    double cluster_elapsed_time, obstacle_speed;
    double robot_Ux,robot_Uy,obstacle_Ux,obstacle_Uy,robot_Vx,robot_Vy,obstacle_Vx,obstacle_Vy,robot_speed = 0.2;
    double time_CPA,distance_CPA,safety_distance = 0.5, robot_r = 0.3,obstacle_r = 0;
    double robot_defelection_from_goal = 0, finish_time_current_scan=0;
  
    double goal_x = 10.0, goal_y = 0.0, X_stop_position = 6;
    double decision_time,decision_starting_time; /*Starting time of the decision*/

    high_resolution_clock::time_point time_at_this_point;   /*Time at a particular point..*/

    int cluster_size = 0;
    int number_of_clusters = 0;
    int nearest_cluster_index = 0;
    int Number_of_decision = 0;
    int Number_code_3 = 0;
    int previous_code_isnt_3 = 1;
    int keepWhileLoop = 1;
    int calculate_via_again = 0;
    int cv; 
    int reset_kf = 0;
    double previousS1s2 = 0;
    string s;
    string s_info = ""; /* An informaton about the obstacle via_points and center position*/


    double lower_t,upper_t;
    double Previous_obstacle_pos_x,Previous_obstacle_pos_y,finish_time_previous_scan;


    double previous_x,previous_y,this_x,this_y,this_cluster_from_previous_d; 
              
    /*Kalman filter*/
    /*system models: dt, Fk, Pk, Xk*/
    double dt;  //delta t, chnage of time in second.

    int n = 4, m = 2; /*n- The number of State veriable, 
                        m- The number of Observable(Measurement)state variables*/
    MatrixXf Fk(n,n); /*State transtion matrix*/

    MatrixXf Pk_1(n,n); /*Covariance of a priori estimate*/
    MatrixXf Xk_1(n,1); /* State variable of a priori estimate*/

    MatrixXf Pk(n,n); /*Covariance of predicted estimate*/
    MatrixXf Xk(n,1); /* State variable of predicted estimate*/

    MatrixXf Pk_new(n,n); /*Optimal estimate*/
    MatrixXf Xk_new(n,1);

    /*Measurement:Zk State-to-Measurement: Hk */
    MatrixXf Zk(m,1);
    MatrixXf Hk(m,n);

    /*System noise:Q measurement noise:R*/
    MatrixXf Q(n,n);
    MatrixXf R(m,m);

    /*KalmanFilter: K, X_new, P_new*/
    MatrixXf K(n,m);
    MatrixXf X_new(n,1);
    MatrixXf P_new(n,n);

    /*Intialization of constant variables*/
    Hk << 1, 0, 0, 0,
          0, 1, 0, 0;

    Q << 0.0004, 0.0,    0.0,    0.0,
         0.0,    0.0004, 0.0,    0.0,
         0.0,    0.0,    0.0001, 0.0,
         0.0,    0.0,    0.0,    0.0001;

    R << 0.0016, 0.0, 
         0.0,    0.0016;

    /*=================== Enter to while loop, continuosly check Laser sensor's data=====================*/
    while(keepWhileLoop){ 

        /*=================== Is Urg data available?=====================*/

        while(scannedDataReady != true)   
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); /*sleep for 2 milli seconds*/
          }

        /*=================== Data is recieved =====================*/

        /*Data is recieved and set to false for incoming URG data*/
        scannedDataReady = false;

        /*Save this time as a time this cluster has been captured.*/
        time_at_this_point = high_resolution_clock::now(); 
        finish_time_current_scan = (double)duration_cast<milliseconds>(time_at_this_point - program_starting_time).count();

        /*Clear this buffer*/
        urg_vector_buffered.clear(); 

        /*Copy the URG data to Buffer*/
        urg_vector_buffered = urg_vector_copy;

        printf("      # [%d] data recieved at %f time!\n",int( urg_vector_buffered.size()), finish_time_current_scan/1000);

        /*Clear the urg_vector_copy and prepare it for next fresh URG data*/
        urg_vector_copy.clear(); 
      
        /*=================== Is target reached, if so, Stoppppppppp!!! =====================*/
        if(odom_x >= X_stop_position){    
            printf("# Goal has reached!\n");
            /* Stop robot */
            Spur_stop();
            /* get out of while loop */
            keepWhileLoop = 0;
           
        }else{ 
          
          /*=================== Does laser sensor send any Data(Does any obstacle detected)? =====================*/
          /*=================== If not, Let the robot go to the goal      =====================*/

          if(urg_vector_buffered.size() > 0){ 

            /*=========STEP A ========== Change the data from vector to point cloud, Process the Point cloud
                                          to detect the number of Clusters           =====================*/
            
            /*Intialize point cloud */
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            cloud->width    = urg_vector_buffered.size();
            cloud->height   = 1;
            cloud->is_dense = true;
            cloud->points.resize (cloud->width * cloud->height);

            /*Populate Data to point cloud */
            for(int l=0; l < urg_vector_buffered.size() ; l++){
               cloud->points[l].x = urg_vector_buffered[l].x;
               cloud->points[l].y = urg_vector_buffered[l].y;
               cloud->points[l].z =  0;
            }

            /*Creating the KdTree object for the search method of the extraction*/
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.2); /*Set 20cm as a Maximum distance between points in a cluster*/
            ec.setMinClusterSize (3);
            ec.setMaxClusterSize (681);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.extract (cluster_indices);

            /*Iterate over the clusters*/
            //printf("  [-] cluster info:\n");
            for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud->points[*pit]); 
              cloud_cluster->width  = cloud_cluster->points.size ();
              cloud_cluster->height = 1;
              cloud_cluster->is_dense = true;

              /*This cluster size*/
              cluster_size = cloud_cluster->points.size(); 
              /*The mean of the two edge points of a cluster*/
              cluster_x_mean = (cloud_cluster->points[0].x + cloud_cluster->points[cluster_size -1].x)/2;
              cluster_y_mean = (cloud_cluster->points[0].y + cloud_cluster->points[cluster_size -1].y)/2;
              //cluster_x_mean = cloud_cluster->points[cluster_size/2].x;
              //cluster_y_mean = cloud_cluster->points[cluster_size/2].y;

              /*cluster_to_robot_distance is distance from Robot to this cluster */
              cluster_to_robot_distance  = sqrt(pow((cluster_x_mean - odom_x),2) + pow((cluster_y_mean - odom_y),2));
              /*This cluster radius*/
              obstacle_radius = sqrt(pow( (cloud_cluster->points[0].x - cloud_cluster->points[cluster_size -1].x) ,2) + pow( (cloud_cluster->points[0].y - cloud_cluster->points[cluster_size -1].y),2))/2;
              //printf("    [-] cluster size: %d, center: (%f,%f), distance from robot: %f \n", cluster_size,cluster_x_mean,cluster_y_mean,cluster_to_robot_distance);
              /*Save this cluster's information into Cluster vector*/
              cluster_vector.push_back(cluster_struct(cluster_to_robot_distance,cluster_size,cluster_x_mean,cluster_y_mean,obstacle_radius,cloud_cluster->points[0].x,cloud_cluster->points[0].y,cloud_cluster->points[cluster_size -1].x,cloud_cluster->points[cluster_size -1].y));
            }
            
            /*=========STEP B ========== Is there Any Cluster found?????? =====================*/

            number_of_clusters = cluster_vector.size();
            printf("        # Number_of_clusters %d!\n",int(number_of_clusters));

            if(number_of_clusters == 0){
              /*========== There is no any cluster percieved as obstacle,so let the robot go to the Goal  =====================*/
              printf("# [*] No obstacle detected. Execute Global Planner. \n");

              /*Clear obstacle's history*/
              obstacle_vector.clear();

              /* Excute Global planner */
              robot_defelection_from_goal = atan2((goal_y - odom_y), (goal_x - odom_x));
              Spur_line_GL(goal_x, goal_y, robot_defelection_from_goal); 

              /* Log information into a file */
              if(previous_code_isnt_3 == 1 or Number_code_3 >= 10){

                /*Robot information*/
                printf("%d %f %f %s %s %s %s %s %s %s %s %s %s %s\n",6,odom_x,odom_y," "," "," "," "," "," "," "," "," "," "," "); 

                Number_code_3 = 0;
                string s = to_string(3) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(goal_x) + " " + to_string(goal_y) + "\n";
                myfile<<s; 

              }else{
                Number_code_3 ++;
              }

              previous_code_isnt_3 = 0;

            }else{
              /*========== There is(are) a cluster(s) =====================*/

              /*Find the nearest cluster*/
              nearest_cluster_index = 0;
              for(int c = 0; c < number_of_clusters; c++){
                //printf("  Cluster %d: Size %d\n", c + 1, cluster_vector[c].s);
                if(cluster_vector[c].d < cluster_vector[nearest_cluster_index].d){
                    nearest_cluster_index = c;
                }
              }

              /*Is this cluster part of the previous clusters?*/
              if(obstacle_vector.size() == 0){
                /* Save the mean of the closest cluster, its radius and time.*/
                obstacle_vector.push_back(obstacle_struct(cluster_vector[nearest_cluster_index].x,cluster_vector[nearest_cluster_index].y,finish_time_current_scan,cluster_vector[nearest_cluster_index].r));
                reset_kf = 1;
                /*Start decison time*/
                decision_starting_time = finish_time_current_scan;
               

              }else{
                /*Check the distance*/
                previous_x = obstacle_vector[obstacle_vector.size() - 1].x;
                previous_y = obstacle_vector[obstacle_vector.size() - 1].y;
                this_x = cluster_vector[nearest_cluster_index].x;
                this_y = cluster_vector[nearest_cluster_index].y;
                this_cluster_from_previous_d = sqrt(pow((this_x - previous_x),2) + pow((this_y - previous_y),2));
                obstacle_vector.clear();

                if(this_cluster_from_previous_d > 0.35){
                  /* Save the mean of the closest cluster, its radius and time.*/
                  obstacle_vector.push_back(obstacle_struct(cluster_vector[nearest_cluster_index].x,cluster_vector[nearest_cluster_index].y,finish_time_current_scan,cluster_vector[nearest_cluster_index].r)); 
                  /*This cluster is from another obstacle*/
                  reset_kf = 1;
                  

                }else{
                  obstacle_vector.push_back(obstacle_struct(cluster_vector[nearest_cluster_index].x,cluster_vector[nearest_cluster_index].y,finish_time_current_scan,cluster_vector[nearest_cluster_index].r)); 
                }

              }
              
              //printf("# Nearest cluster x:%f y:%f and Obstacle vector size: %d\n",cluster_vector[nearest_cluster_index].x,cluster_vector[nearest_cluster_index].y,int(obstacle_vector.size()));
              /*Clear cluster vector*/
              cluster_vector.clear(); 

              /*=================== START Kalman Filtering =====================*/
              if(reset_kf == 1){
                
                printf("\n\n");

                /*set kalman filter*/
                reset_kf = 0;

                /*Setting obstacle's intial position and velocity*/
                obstacle_Ux = obstacle_vector[0].x;
                obstacle_Uy = obstacle_vector[0].y;
                obstacle_Vx = 0;
                obstacle_Vy = 0;
                obstacle_speed = sqrt(pow(obstacle_Vx,2) + pow(obstacle_Vy,2));

                /*Intialization of Xk_1 and Pk_1*/
                Xk_1 <<  obstacle_Ux, 
                         obstacle_Uy,    
                         obstacle_Vx,    
                         obstacle_Vy;
                     
                Pk_1 <<  0.0016, 0.0,    0.0,    0.0,
                         0.0,    0.0016, 0.0,    0.0,
                         0.0,    0.0,    0.0004, 0.0,
                         0.0,    0.0,    0.0,    0.0004;

                Zk << obstacle_Ux, 
                      obstacle_Uy;

                finish_time_previous_scan = finish_time_current_scan;
   
              }else{
                /*Get the time of decision*/
                cluster_elapsed_time = (obstacle_vector[0].t - finish_time_previous_scan)/1000; 
                
                dt = cluster_elapsed_time;
                Fk << 1, 0,  dt, 0,
                      0, 1,  0,  dt,
                      0, 0,  1,  0,
                      0, 0,  0,  1;

                /*Prediction*/
                Xk = Fk * Xk_1;
                Pk = Fk * Pk_1 * Fk.transpose() + Q;

                /*Measurement*/
                Zk << obstacle_vector[0].x, 
                      obstacle_vector[0].y;

                /*Kalman gain*/
                K = Pk* Hk.transpose() * (Hk*Pk*Hk.transpose() + R).inverse();

                /*calaculate the optimal estimate*/
                X_new = Xk + K*(Zk - Hk * Xk);
                P_new = Pk - K*Hk*Pk;

                /*use X_new as the Obstacle's position and velocity*/
                obstacle_Ux = X_new(0);
                obstacle_Uy = X_new(1);
                obstacle_Vx = X_new(2);
                obstacle_Vy = X_new(3);
                obstacle_speed = sqrt(pow(obstacle_Vx,2) + pow(obstacle_Vy,2));

                /*Feed the new optimal estimate as a prior for next prediction*/
                Xk_1 = X_new;
                Pk_1 = P_new; 

                /*Set current estimated position as previous position to calculate velocity*/
                finish_time_previous_scan = finish_time_current_scan;

              }

            
              /*=========STEP C ========== If decision_time is greater than 950ms then proceed to a decision!! =====================*/
              decision_time = (finish_time_current_scan - decision_starting_time)/1000;

              if(decision_time > 1.35){

                previous_code_isnt_3 = 1;
                Number_of_decision ++;

                printf("# Decision: %d \n", Number_of_decision);
                printf("  # Reset kalman filter status: %d \n",reset_kf);

                /*=================== ROBOT'S POSITION AND VELOCITY=====================*/
                robot_Ux  = odom_x;
                robot_Uy  = odom_y;

                /*How would the robot turn to reach the goal if there were no obstacle - Global path planner*/
                robot_defelection_from_goal = atan2((goal_y - robot_Uy), (goal_x - robot_Ux)); 
                robot_Vx  = robot_speed * cos(robot_defelection_from_goal);
                robot_Vy  = robot_speed * sin(robot_defelection_from_goal);
                
                /*=================== OBSTACLE'S POSITION AND VELOCITY=====================*/
                obstacle_r = obstacle_vector[0].r;
                
                /*==================Time_CPA AND Distance_CPA CALACULATIONS=================*/
                time_CPA = (-(robot_Ux - obstacle_Ux) * (robot_Vx - obstacle_Vx) - (robot_Uy - obstacle_Uy) * (robot_Vy - obstacle_Vy))/(pow((robot_Vx - obstacle_Vx),2) + pow((robot_Vy - obstacle_Vy),2));
                distance_CPA = sqrt(pow((time_CPA*robot_Vx - time_CPA*obstacle_Vx + robot_Ux - obstacle_Ux),2) + pow((time_CPA*robot_Vy - time_CPA*obstacle_Vy + robot_Uy - obstacle_Uy),2));
                printf("  # Abslute Time: %f \n",finish_time_current_scan/1000);
                printf("  # Decision Time: %f \n",decision_time);
                printf("  # TCPA: %f  DCPA:%f\n",time_CPA,distance_CPA);

                /*Start decison time*/
                decision_starting_time = finish_time_current_scan;
                

                /*========================AVOIDANCE ALGORITHM===================*/

                /*======================== Is collision going to happen????? ===================*/
                if( time_CPA > 0 and distance_CPA <= robot_r + obstacle_r + safety_distance){ 

                  /* Declartion of important variables */
                  double q,Theta,Alpha,M,l;
                  calculate_via_again = 0;
         
                  /*For loop to calculate via point*/
                  for(cv = 1; cv <= 2; cv++){
                    double Actual_robot_obstacle_distance = sqrt(pow((robot_Ux - obstacle_Ux),2) + pow((robot_Uy - obstacle_Uy),2));

                    l = obstacle_r + robot_r + safety_distance;

                    if( calculate_via_again == 1){
                        /*viapoint has been found already*/
                        break;
                    }else if(calculate_via_again == 2){
                        /*repeate calculating via point by diminishing the distance*/
                        l = obstacle_r + safety_distance;
                    }

                    q = sqrt( pow(obstacle_Ux - robot_Ux , 2) + pow ( obstacle_Uy - robot_Uy, 2));
                    Theta = asin(l/q); 
                    Alpha = atan2((obstacle_Uy - robot_Uy),(obstacle_Ux - robot_Ux));
                    M = sqrt(pow(q,2) - pow(l,2));


                    /*Calculating the avoiding points.*/
                    double xs1,ys1,xs2,ys2;
                    xs1 = robot_Ux + M * cos(Alpha - Theta);
                    ys1 = robot_Uy + M * sin(Alpha - Theta);
                    xs2 = robot_Ux + M * cos(Alpha + Theta);
                    ys2 = robot_Uy + M * sin(Alpha + Theta);
                    //printf("    [-] xs1, ys1: (%f, %f) \n",xs1,ys1);
                    //printf("    [-] xs2, ys2: (%f, %f) \n",xs2,ys2);
            
                    /*The maximum distance the robot can cover/second*/
                    double r = 0.2;

                    /*Calculate the intersection between the avoiding line and reachable cone*/
                    double cx,cy,vx,vy;
                    double a, b, c; /*Coefficients of quadratic equation*/
                    double Tmp, Solution_1, Solution_2;

                    /*Intersection b/n avoiding line through s1 and reachable cone.*/
                    cx = xs1-robot_Ux; 
                    cy = ys1-robot_Uy;
                    vx = obstacle_Vx;
                    vy = obstacle_Vy;

                    a = pow(vx,2) + pow(vy,2) - pow(r,2);
                    b = 2*vx*cx + 2*vy*cy;
                    c = pow(cx,2) + pow(cy,2);

                    Tmp = b*b - 4*a*c;
                    Solution_1 = (-1*b - sqrt(Tmp))/(2*a);
                    Solution_2 = (-1*b + sqrt(Tmp))/(2*a);
                    //printf("  [+] calculating avoiding line:\n");
                    //printf("    [-] Intersection b/n reachable cone and avoiding line through S1\n");

                    if( (Solution_1 >= 0) and (!isnan(Solution_1)) and (!isinf(Solution_1))) {
                        viaPoints_vector.push_back(viaPoints_struct(xs1 + Solution_1 * obstacle_Vx, ys1 + Solution_1 * obstacle_Vy,1));
                        //printf("      [-] solution 1: %f  Via points through this: (%f,%f) \n", Solution_1,xs1 + Solution_1 * obstacle_Vx, ys1 + Solution_1 * obstacle_Vy);
                    }

                    if( (Solution_2 >= 0) and (!isnan(Solution_2)) and (!isinf(Solution_2)) ){
                        viaPoints_vector.push_back(viaPoints_struct(xs1 + Solution_2 * obstacle_Vx, ys1 + Solution_2 * obstacle_Vy,1));
                        //printf("      [-] solution 2: %f  Via points through this: (%f,%f) \n", Solution_2,xs1 + Solution_2 * obstacle_Vx, ys1 + Solution_2 * obstacle_Vy);
                    }
   
                    /*Intersection b/n avoiding line through s2 and reachable cone.*/
                    cx = xs2-robot_Ux; 
                    cy = ys2-robot_Uy;
                    vx = obstacle_Vx;
                    vy = obstacle_Vy;

                    a = pow(vx,2) + pow(vy,2) - pow(r,2);
                    b = 2*vx*cx + 2*vy*cy;
                    c = pow(cx,2) + pow(cy,2);

                    Tmp = b*b - 4*a*c;
                    Solution_1 = (-1*b - sqrt(Tmp))/(2*a);
                    Solution_2 = (-1*b + sqrt(Tmp))/(2*a);

                    //printf("    [-] Intersection b/n reachable cone and avoiding line through S2\n");

                    if( (Solution_1 >= 0) and (!isnan(Solution_1)) and (!isinf(Solution_1)) ){
                        viaPoints_vector.push_back(viaPoints_struct(xs2 + Solution_1 * obstacle_Vx, ys2 + Solution_1 * obstacle_Vy,2));
                        //printf("      [-] solution 1: %f  Via points through this: (%f,%f) \n", Solution_1,xs2 + Solution_1 * obstacle_Vx, ys2 + Solution_1 * obstacle_Vy);
                    }

                    if( (Solution_2 >= 0) and (!isnan(Solution_2)) and (!isinf(Solution_2)) ){
                        viaPoints_vector.push_back(viaPoints_struct(xs2 + Solution_2 * obstacle_Vx, ys2 + Solution_2 * obstacle_Vy,2));
                        //printf("      [-] solution 2: %f  Via points through this: (%f,%f) \n", Solution_2,xs2 + Solution_2 * obstacle_Vx, ys2 + Solution_2 * obstacle_Vy);
                    }

                    //printf("  [+] find out the via point with minimum time to goal:\n");
                    

                    double via_x, via_y, total_time_to_goal, time_from_Robot_viaPoint , time_from_viaPoint_goal;
                    
                    /* If there is an intersection b/n reachable cone and avoiding line */
                    if(viaPoints_vector.size() > 0){

                      calculate_via_again = 1;
                      
                      /*Final Position of the obstacle*/
                      double Obstacle_final_position_x = 0;
                      double Obstacle_final_position_y = 0;
                      s_info = "";

                      /* There is at least one solution - pop the first element and compare with rest one */
                      for(int vpts = 0; vpts < viaPoints_vector.size(); vpts++){
                        via_x = viaPoints_vector[vpts].via_x;
                        via_y =  viaPoints_vector[vpts].via_y;

                        time_from_Robot_viaPoint = 1/r * sqrt( pow((via_x - robot_Ux),2) + pow((via_y - robot_Uy),2));
                        time_from_viaPoint_goal = 1/r * sqrt( pow((goal_x - via_x),2) + pow(( goal_y - via_y),2));

                        total_time_to_goal = time_from_Robot_viaPoint + time_from_viaPoint_goal;

                        //printf("    [-] via point: (%f,%f) \n", via_x,via_y);
                        //printf("      [-] time to this via point: %f \n", time_from_Robot_viaPoint);
                        //printf("      [-] time from this via point to goal: %f \n", time_from_viaPoint_goal);
                        //printf("      [-] total time through this via point: %f \n", total_time_to_goal);
                      
                        select_viaPoints_vector.push_back(select_viaPoints_struct(via_x,via_y,time_from_Robot_viaPoint,total_time_to_goal,viaPoints_vector[vpts].s1s2));
                      
                        /* Temporary information to be logged*/ 
                        Obstacle_final_position_x = obstacle_Ux + time_from_Robot_viaPoint*obstacle_Vx;
                        Obstacle_final_position_y = obstacle_Uy + time_from_Robot_viaPoint*obstacle_Vy;
                        
                        s_info = s_info + to_string(Obstacle_final_position_x) + " " +  to_string(Obstacle_final_position_y) + " " + to_string(via_x) + " " + to_string(via_y) + " " + to_string(time_from_Robot_viaPoint) + " ";
              
                      }

                      /*Select the viapoint that optimizes the time to goal*/
                      double lambda;
                      if(select_viaPoints_vector.size() > 1){
                        /*Sort the viaPoints according the time to goal*/
                        for(int viai = 0; viai < select_viaPoints_vector.size(); viai ++){
                          for(int viaj = viai + 1; viaj < select_viaPoints_vector.size(); viaj ++){
                            if(select_viaPoints_vector[viai].t > select_viaPoints_vector[viaj].t){
                              tmp_select_viaPoints_vector[0] = select_viaPoints_vector[viai];
                              select_viaPoints_vector[viai] = select_viaPoints_vector[viaj];
                              select_viaPoints_vector[viaj] = tmp_select_viaPoints_vector[0];
                            }
                          }
                        }

                        //printf("    [+] Sorted viapoints \n");
                        for(int c=0;c < select_viaPoints_vector.size(); c++){
                          //printf("     [-] Via point %f %f Time:%f and s1s2:%f \n",select_viaPoints_vector[c].x,select_viaPoints_vector[c].y,select_viaPoints_vector[c].t,select_viaPoints_vector[c].s1s2);
                        }

                       /*check if the time difference between the first and the second is < 1 second*/
                       //printf("#    [#] Time difference is: %f \n", (select_viaPoints_vector[1].t - select_viaPoints_vector[0].t));
                       if((select_viaPoints_vector[1].t - select_viaPoints_vector[0].t) <= 1){
                          //printf("     [-] Previous S1s2: %f \n", previousS1s2);
                          if(previousS1s2 == select_viaPoints_vector[1].s1s2){
                            via_x = select_viaPoints_vector[1].x;
                            via_y = select_viaPoints_vector[1].y;
                            lambda = select_viaPoints_vector[1].t_via;
                            previousS1s2 = select_viaPoints_vector[1].s1s2;
                          }else{
                            via_x = select_viaPoints_vector[0].x;
                            via_y = select_viaPoints_vector[0].y;
                            lambda = select_viaPoints_vector[0].t_via;
                            previousS1s2 = select_viaPoints_vector[0].s1s2;
                          }
                       }else{
                          via_x = select_viaPoints_vector[0].x;
                          via_y = select_viaPoints_vector[0].y;
                          lambda = select_viaPoints_vector[0].t_via;
                          previousS1s2 = select_viaPoints_vector[0].s1s2;
                       }

                      }else{
                        via_x = select_viaPoints_vector[0].x;
                        via_y = select_viaPoints_vector[0].y;
                        lambda = select_viaPoints_vector[0].t_via;
                        previousS1s2 = select_viaPoints_vector[0].s1s2;
                      }

                      /*get angle of robot to viapoint*/
                      double Angle_To_ViaPoint = atan2((via_y - robot_Uy), (via_x - robot_Ux));
                      
                      printf("  # Try: %d detour/Via point found l(cylinder raduis): %f Actual robot-Obstacle distance: %f \n",cv,l,Actual_robot_obstacle_distance);
                      //printf("# Try: %d selected via point: (%f,%f) Time: %f Turn angle: %f \n",cv, via_x,  via_y, total_time_to_goal, Angle_To_ViaPoint*180/M_PI);        
                      Spur_line_GL(via_x, via_y, Angle_To_ViaPoint);
                      
                      /*Velocity estimation information*/
                      //printf("  [+] Velocity Estimation Information:\n");
                      printf("%d %f %f %f %f %f %f %f %f %f %f %f %f %d\n",0,robot_Ux,robot_Uy,Angle_To_ViaPoint,obstacle_Ux,obstacle_Uy,Zk(0),Zk(1),Xk(0),Xk(1),obstacle_Vx,obstacle_Vy,obstacle_speed,Number_of_decision); 

                      if(select_viaPoints_vector.size() > 1){
                        /*Save as code 4*/
                        string s = to_string(4) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(obstacle_Ux) + " " + to_string(obstacle_Uy)  + " " + to_string(obstacle_r) + " " + to_string(xs1) + " " + to_string(ys1) + " " + to_string(xs2) + " " + to_string(ys2) + " " + to_string(l) + " " + to_string(via_x) + " " + to_string(via_y)+ " " + to_string(goal_x) + " " + to_string(goal_y) + " " + s_info + "\n";
                        myfile<<s;

                      }else{
                        /*Save as code 0*/
                        string s = to_string(0) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(obstacle_Ux) + " " + to_string(obstacle_Uy)  + " " + to_string(obstacle_r) + " " + to_string(xs1) + " " + to_string(ys1) + " " + to_string(xs2) + " " + to_string(ys2) + " " + to_string(l) + " " + to_string(via_x) + " " + to_string(via_y)+ " " + to_string(goal_x) + " " + to_string(goal_y) + " " + s_info + "\n";
                        myfile<<s;

                      }                   
                      
                    }else{
                      printf("  # Try: %d Stop crossing Point Not Detected! l(cylinder raduis): %f, Actual robot-Obstacle distance: %f \n",cv,l,Actual_robot_obstacle_distance);

                      if(cv == 1){ 
                        /*Repeat the process of finding via point*/
                        calculate_via_again = 2;

                      }else{
                        /* Stop YP-spur Movement commands immediatly */ 
                        Spur_stop();

                        /*Velocity estimation information*/
                        //printf("  [+] Velocity Estimation Information:\n");
                        printf("%d %f %f %f %f %f %f %f %f %f %f %f %f %d\n",1,robot_Ux,robot_Uy,0.0,obstacle_Ux,obstacle_Uy,Zk(0),Zk(1),Xk(0),Xk(1),obstacle_Vx,obstacle_Vy,obstacle_speed,Number_of_decision); 

                        /* Log information into a file */ 
                        string s = to_string(1) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(obstacle_Ux) + " " + to_string(obstacle_Uy)  + " " + to_string(obstacle_r) + " " + to_string(goal_x) + " " + to_string(goal_y) + " " + to_string(l) + "\n";
                        myfile<<s;
                      }   
                    }

                    viaPoints_vector.clear();
                    select_viaPoints_vector.clear();

                  }
                  
                }else{ 
                  printf("  # No collison detected. Excute Global path planner! \n");
                  
                  /* Excute Global planner */
                  Spur_line_GL(goal_x, goal_y, robot_defelection_from_goal);

                  /*Velocity estimation information*/
                  //printf("  [+] Velocity Estimation Information:\n");
                  printf("%d %f %f %f %f %f %f %f %f %f %f %f %f %d\n",0,robot_Ux,robot_Uy,robot_defelection_from_goal,obstacle_Ux,obstacle_Uy,Zk(0),Zk(1),Xk(0),Xk(1),obstacle_Vx,obstacle_Vy,obstacle_speed,Number_of_decision); 

                  /* Log information into a file  */
                  string s = to_string(2) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(obstacle_Ux) + " " + to_string(obstacle_Uy)  + " " + to_string(obstacle_r) + " " + to_string(goal_x) + " " + to_string(goal_y) + "\n";
                  myfile<<s; 
                }/*End of else*/
              }//End of Obstacle vector size is 11
              
            }
           
          }else{ 

            /*=================== URG returns nothing... Execute Global path planner  =====================*/

            printf("# URG returns nothing. Execute Global Planner. \n");

            /*Clear obstacle's history*/
            obstacle_vector.clear();
          
            /* Excute Global planner */
            robot_defelection_from_goal = atan2((goal_y - odom_y), (goal_x - odom_x));
            Spur_line_GL(goal_x, goal_y, robot_defelection_from_goal); 

            /* Log information into a file */
            if(previous_code_isnt_3 == 1 or Number_code_3 >= 10){
              
              /*Robot information*/
              printf("%d %f %f %s %s %s %s %s %s %s %s %s %s %s\n",6,odom_x,odom_y," "," "," "," "," "," "," "," "," "," "," "); 
              
              Number_code_3 = 0;
              string s = to_string(3) + " " + to_string(odom_x) + " " + to_string(odom_y) + " " + to_string(robot_r) + " " + to_string(goal_x) + " " + to_string(goal_y) + "\n";
              myfile<<s; 
            }else{
              Number_code_3 ++;
            }
            previous_code_isnt_3 = 0;

          }
        }
    }
  
    /* Close the opened file.*/
    myfile.close();
  }
}


/* ========================================
 Main function thread accounts for Environment scanning and feeding the result to 
 avoidance algorithm thread 
========================================*/
int main(int argc, char** argv){
  /*Start avoidance_algorithm_thread*/
  std::thread threadObj(avoidance_algorithm_thread);
  //printf("[+s] Main thread function started. \n");

  /* set Ctrl-c function */
  signal(SIGINT, ctrl_c);  

  /*URG  variables */
  int ret;
  S2Port   *urg_port;   /* port */
  S2Sdd_t   urg_buff;   /* buffer */
  S2Scan_t *urg_data;   /* pointer to buffer */
  S2Param_t urg_param;  /* parameter */

   /* check argument */
  if( argc < 2 ) { 
    fprintf(stderr, "[*] ERORR: missing device operand\n");
    fprintf(stderr, "[*] USAGE: %s <device>\n", argv[0]);
    fprintf(stderr, "[*] e.g.: %s /dev/ttyACM0\n", argv[0]);
    return 1;
  }

  /* open port */
  urg_port = Scip2_Open(argv[1], B115200); 
  if(urg_port == 0) {
    fprintf(stderr, "[*] ERORR: cannot open %s\n", argv[1]);
    return 1;
  }
  //fprintf(stdout, "[*] Port opened\n");

  /* init buffer */ 
  S2Sdd_Init(&urg_buff); 

  /* get paramerter */
  Scip2CMD_PP(urg_port, &urg_param); 

  /* scan start */
  ret = Scip2CMD_StartMS(urg_port, urg_param.step_min, urg_param.step_max, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE );
  if(ret == 0){
    fprintf(stderr, "[*] ERROR: Scip2CMD_StartMS\n");
    return 1;
  }
  //fprintf(stdout, "[*] Scan started\n");
 
  /* open pipe to gnuplot */
  FILE *fp = popen("gnuplot -noraise", "w");  
  if(fp == NULL){
    fprintf(stderr, "[*] ERROR: popen\n");
    return 1;
  }
  //fprintf(stdout, "[*] Pipe opened\n");

  fputs("set terminal x11\n", fp);   /* drawing destination */
  fputs("set terminal qt size 1286, 728\n", fp);   /* set to maximum screen size */
  fputs("set grid\n", fp);  /* draw grid */
  fputs("set mouse\n", fp);  /* use mouse */
  fputs("set xlabel \"Robot X [m]\"\n", fp);  /* label of x-axis */
  fputs("set ylabel \"Robot Y [m]\"\n", fp);  /* label of y-axis */
  fputs("set xrange [-6:6] reverse\n", fp);  /* range of x-axis */
  fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  fputs("set size ratio -1\n", fp);  /* aspect ratio */
  fputs("unset key\n", fp);  /* hide graph legends */

  /* declared variables. */
  int i; /*Counter for fetching URG values...*/
  double x, y, rad,robot_vel,robot_omega; /*X, Y and rad values directly from URG, Robot's velocity, Robot's angular velocity*/
  double x_translated, y_translated; /*GL translated values of the above X and Y*/
  urg_vector.reserve(681);   /* Vector used to hold x_translated and y_translated*/
  urg_vector_copy.reserve(681); /* A vector that is a copr of dataset and used to transmit to the other thread*/

  /* main loop starting here */
  gIsShuttingDown = 0;
  
  /* Start Scan Time*/
  program_starting_time = high_resolution_clock::now();

  while(!gIsShuttingDown) { 
    ret = S2Sdd_Begin(&urg_buff, &urg_data);
    if(ret > 0){
      fputs("plot '-'\n", fp);

      /*get Odometry measurments*/
      Spur_get_pos_GL(&odom_x, &odom_y, &odom_theta);
      Spur_get_vel(&robot_vel,&robot_omega );

      /*Change Angle so that it will compensate URG time delay*/
      odom_theta = odom_theta - robot_omega * 0.055;
      //printf("Scanning thread sent data: \n");
     
      for(i = 0 ; i < urg_data->size ; i++){
        if(urg_data->data[i] < 20) { /* error code */
          continue;
        }
        rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
        x = urg_data->data[i] * cos(rad) / 1000.0;
        y = urg_data->data[i] * sin(rad) / 1000.0;
      
        /*change each point from FS->GL coordinate*/
        x_translated = x * cos(odom_theta) - y * sin(odom_theta) + odom_x;
        y_translated = x * sin(odom_theta) + y * cos(odom_theta) + odom_y;
        
        if(x_translated >= odom_x and x_translated <= odom_x + 4 and x_translated <= 7.5 and fabs(y_translated) <= 3.2 ){
           
           //printf("       # %f %f\n",x_translated,y_translated);
           //fprintf(fp, "%f %f\n", x_translated, y_translated);
           /*Store value into a vector*/
           urg_vector.push_back(urg_struct(x_translated,y_translated));
        }
      
      } /* End of For Loop../*/
      
      /* Copy the urg_vector to a vector so that the other thread will recieve them.*/
      urg_vector_copy = urg_vector; 
      
      /*Set scannedDataReady to True*/
      scannedDataReady = true;

      /*Clear vector for new Urg urg_vector.*/
      urg_vector.clear(); 
      
      fputs("e\n", fp);
      S2Sdd_End(&urg_buff);
      /* wait 90 ms (URG-04LX 1scan = 100 ms) */
      usleep(90000);
    }else if(ret == -1) {
      fprintf(stderr, "[*] ERROR: S2Sdd_Begin\n");
      break;
    } else {
      usleep(100);
    }
  }

  /* Wait untill the other thread finishes its operation.*/
  threadObj.join(); 
  return 0;
}