#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include "std_msgs/Float64MultiArray.h"
#include "tum_ics_skin_msgs/SkinCellDataArray.h" 
#include <cmath> 
#include <exo_control/exo_force_control.h>
#include <algorithm>  
#include "std_msgs/String.h"


double deg2rad(double degree){
    return (degree * 3.14159265359/180);
}



// ************************************* Gravity definition ******************************************
double gx1 = 0;
double gy1 = 0;
double gz1 = 0;

double gx1n = 0;
double gy1n = 0;

double magnitude_a = 0;

double a_beta = 0;

void chatterCallback_g(const tum_ics_skin_msgs::SkinCellDataArray &msg)
{
    if(msg.data[0].cellId == 1){ //Take the acceleration data from the skin patch near to the shoulder

        gx1 = msg.data[0].acc[0] ;
        gy1 = msg.data[0].acc[1] ;
        gz1 = msg.data[0].acc[2] ;

    }

    

    
    magnitude_a = sqrt(pow(gx1,2) + pow(gy1,2)); // Calculate the magnitud of the gravity in the plane 

    gx1 = gx1 / magnitude_a ; // Components normalized
    gy1 = gy1 / magnitude_a ;

    if(isnan(gx1) || isnan(gy1) || isnan(gz1)){ //Dealing with indeterminations
        gx1 = 1;
        gy1 = 0;
        gz1 = 0;
    }

    a_beta = M_PI - std::atan2(-0.5699293974 , -0.821671881044706) ; 

    // a_beta = M_PI - std::atan2(gy1 , gx1) ;

    gx1n = gx1 * std::cos(M_PI - a_beta) + gy1 * std::sin(M_PI - a_beta) ;
    gy1n = - gx1 * std::sin(M_PI - a_beta) + gy1 * std::cos(M_PI - a_beta);

    gx1 = gx1n;
    gy1 = gy1n;

}



// **************************** Force variables *************************************

double force_elbow_down = 0;
double force_elbow = 0;
double desired_force_elbow = 0;
double force_elbow_up = 0;

// **************************** Callback funtion to take the data from skin patches *****************

void chatterCallback_p(const tum_ics_skin_msgs::SkinCellDataArray &msg){

    // cells_numbers: (direction: cell with wire connection to cell without cell connection)
    // patch1: 1, 2, 3
    // patch2: 4, 14 ,15
    // patch3: 11, 12, 13
    // patch4: 5, 9 ,10
    // patch5: 6, 7, 8

    if(msg.data[0].cellId == 7){
        
        force_elbow_down = msg.data[0].force[2] ; // Force in the skin patch placed under the elbow
    }
    if(msg.data[0].cellId == 5){
        
        force_elbow_up = msg.data[0].force[2] ; // Force in the skin patch placed over the elbow
    }
}



/* Main function

This function has a lot of different functionalities which are commented inside itself.


Between those functionalities it could be found:

- Definition of the ROS nodes in charge of the communications (publishers and subscribers)
- Initialization of the physical variables of the exo
- Calibration routine for the force control, so the minimum force required to activate it can be defined
- Algorithm to define how the force and position control cooperate between themselves in order to give support to the user

 */

bool if_cont;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("loop");
    if (msg->data == "z") {
    ROS_INFO_STREAM("continue");
    if_cont = true;}
    else if_cont =  false;

}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "exo_control",
            ros::init_options::AnonymousName);

    ros::NodeHandle n;

    // *********************** Nodes for q position *****************************

    std::vector<double> q_ESP3 = {0, 0};
    std_msgs::Float64MultiArray msg_q_ESP3;
    ros::Publisher exo_control_pub_q_ESP3 = n.advertise<std_msgs::Float64MultiArray>("q_control_publisher", 1); 

    std::vector<double> q_state = {0, 0, 0};
    std_msgs::Float64 msg_q_state;
    ros::Publisher exo_control_pub_q_state = n.advertise<std_msgs::Float64>("q_state", 1); 


    // *********************** Skin-patch nodes for the elbow *****************************

    ros::Subscriber exo_control_sub_gravity1 = n.subscribe("patch1", 1, chatterCallback_g);

    ros::Subscriber exo_control_sub_skinpatch2 = n.subscribe("patch2", 1, chatterCallback_p);

    ros::Subscriber exo_control_sub_skinpatch3 = n.subscribe("patch3", 1, chatterCallback_p);


    // *********************** Skin-patch nodes for the wrist *****************************

    ros::Subscriber exo_control_sub_skinpatch4 = n.subscribe("patch4", 1, chatterCallback_p);

    ros::Subscriber exo_control_sub_skinpatch5 = n.subscribe("patch5", 1, chatterCallback_p);




    ros::Rate r(100);

    

    double delta_t = 1/(double)200; 

    // ************************** Parameters from yaml file ***************************************

    // ************************** Shoulder and elbow parameters ***********************************
    double L1;
    std::string ns="~L1";
    std::stringstream s;
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L1);
    double L2;
    ns="~L2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L2);
    double m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    double b1;
    ns="~b1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b1);
    double k1;
    ns="~k1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k1);
    double theta1;
    ns="~theta1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta1);
    double I211;
    ns="~I211";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I211);
    double I222;
    ns="~I222";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I222);    
    double I233;
    ns="~I233";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I233);
    double g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);




    
    

    //******************** Elbow Position Control *************************

    ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx1*g, gy1*g);
    Vector3d qEnd;
    double timeEnd = 0;
    
    

    // ************************** Dynamic system matrices ***************************************

    // ************************** 1 DOF ***************************************

    double m_matrix; 
    double c_matrix;
    double g_matrix;
    double b_matrix;

    // ************************** Initial conditions for dynamics ***************************************

    double q = deg2rad(90);
    double qd = 0;
    double qdd = 0;

    Vector3d g_vec(g, 0, 0);
    double tao = 0;


    // ****************************** Calibration routine **********************************************
    /* This calibration routine takes a certain number of samples equal to "n_values" and choose the 
        maximal value, in order to set that value as the minimum value required to activate the force control*/

    q_ESP3[0] = q;
    q_ESP3[1] = 0;
    
    msg_q_ESP3.data = q_ESP3;
    ROS_INFO_STREAM("start position");
    
    /* ros::spinOnce();
    r.sleep(); */
    ros::spinOnce();
    exo_control_pub_q_ESP3.publish(msg_q_ESP3);
    ros::spinOnce();
    /* ros::spinOnce();
    r.sleep(); */


    
    int n_values = 200; // Number of values to take in account in the calibration routine
    int counter = 0; // Counter to follow the number of values that were taken in account, so the loop stops when counter = n_values

    double force_elbow_down_calibrated = 0; // Variable for the minimum force value required to activate the force control downward
    double force_elbow_up_calibrated = 0; // Variable for the minimum force value required to activate the force control upward

    

    ROS_INFO_STREAM("Estimate calibrated values");

    while(ros::ok() && counter < n_values)
    {
        ros::spinOnce();

        if (force_elbow_down > force_elbow_down_calibrated) force_elbow_down_calibrated = force_elbow_down; // Take the maximum value
        if (force_elbow_up > force_elbow_up_calibrated) force_elbow_up_calibrated = force_elbow_up; // Take the maximum value
        counter +=1;

        r.sleep();
    }

    ROS_INFO_STREAM("Start control");
    q = deg2rad(11);
    q_ESP3[0] = q;
    q_ESP3[1] = 0;
    msg_q_ESP3.data = q_ESP3;
    exo_control_pub_q_ESP3.publish(msg_q_ESP3);
    

    

    

    // ************************** Initial conditions for force control ***************************************


    

    ExoControllers::ForceControl forceControl_elbow_intern(L2);
    forceControl_elbow_intern.init();


    // ************************** Variables for control algorithm ***************************************

    // Further explanation about how the algorithm works can be found below
    
    
    double q_cos [2]; // Array which stores the peaks of the generate trajectory
    q_cos [0] = deg2rad(10); // Minimum peak
    q_cos [1] = deg2rad(100); // Maximum peak
    
    double time [2]; // Array which stores the time when peaks where reached
    time[1] = 2.5;
    
    bool run_posControl = false; // Boolean which defines if position control is used or not


    double q_predicted = 0; // Next point to be followed by the position control

    double force_control = 0; // Stores the tau from force control
    double position_control = 0; // Stores the tau from position control

    double q_temp_max = 0; // Stores maximum value to the define the maximum peak

    
    bool new_tr = false; // Defines if a new trajectory is going to be generate

    double q_tot; // Variable mess if q is still in the desire range of 10 - 100 degrees

    while(ros::ok())
    {
        
        ros::spinOnce();

        /* This algorithm defines first the variables related to the dynamic of the system in order to model the system */

        // ************************** Definition of the g vector with the lectures of the skin-patch near to the shoulder ************************** 

        g_vec[0] = gx1 * g;
        g_vec[1] = gy1 * g;


        // ************************** Definition of the dynamic matrices for 1 DOF ************************** 

        m_matrix = I233 + L2*L2*m2/4;
        c_matrix = 0;
        g_matrix = - L2*g_vec[0]*m2*sin(q)/2 + L2*g_vec[1]*m2*sin(q)/2 ;
        b_matrix = b1 ;






        /* Defines time[0], which is the time when the minimum peak of the sinusoidal wave is reached */
        
        if(q - deg2rad(12) < 0.001)
        {
            time[0] = ros::Time::now().toSec();
        }

        
        /* Then comes a routine where is determined if the force control is activated or not */

        if(force_elbow_up > force_elbow_up_calibrated + 0.01 )
        {
            force_elbow = force_elbow_up;
            desired_force_elbow = force_elbow_up_calibrated;
        }

        else if(force_elbow_down > force_elbow_down_calibrated + 0.01 )
        {
            force_elbow = -force_elbow_down;
            desired_force_elbow = -force_elbow_down_calibrated;

            /* Inside of this routine there is an algorithm in charge defining a new maximal peak 
            when the user applies a force downwards, here is defined time[1] too, as the time between 
            two peaks, half a period. What is going to be used as a value related to the frequence of 
            the generated trajectory */

            if (q > q_temp_max /* && !new_tr */){
                q_temp_max = q;
            } else if (q < q_temp_max - deg2rad(5)) {

                q_cos[1] = q_temp_max;
                time[1] = ros::Time::now().toSec() - time[0];
                run_posControl = true;

                q_temp_max = q + deg2rad(1); 
                new_tr = true;
            
            }
            
            
        }
        else
        {
            force_elbow = 0;
            desired_force_elbow = 0;
        }

        

        /* Then if the two peaks required to generate a new trajectory were defined, the run_posControl 
        boolean was setted to true and the system is prepared to run the position control */
        
        if(run_posControl) {

            if((abs(q - qEnd[0]) < 0.001) || new_tr )
            {
                
                if(abs(q_predicted - q_cos[0]) < 0.0001)
                {
                    q_predicted = q_cos[1];
                }
                else
                {
                    q_predicted = q_cos[0];
                }

                qEnd << std::min(1.6, q_predicted),0,0;

                //In case there was a problem with the time measurement, it is restringed to a range of 1 - 3 seconds

                if (time[1] < 1) time[1] = 1;

                if (time[1] > 3) time[1] = 3;

                //Here the time in which the next peak is wanted to be reached is defined

                timeEnd = time[1];
                
                // Here the new trajectory is defined
                posControl.init(qEnd,timeEnd);
                new_tr = false;

                
            }

            force_control = 10 * forceControl_elbow_intern.update(force_elbow, desired_force_elbow);
            position_control = 0.01 * posControl.update(delta_t,q,qd,qdd);

            // In case the position control disturbs the normal behavior of the force control it is shutted down

            if(force_control * position_control < 0)
            {
                tao = force_control +  g_matrix;
                //q_temp_max = 0;
            } else tao = force_control + position_control +  g_matrix;
            
            ROS_WARN_STREAM("force control : "<<forceControl_elbow_intern.update(force_elbow, desired_force_elbow));
            ROS_WARN_STREAM("pose control : "<<posControl.update(delta_t,q,qd,qdd));
            
            
        }
        else 
        {
            tao = 10 * forceControl_elbow_intern.update(force_elbow, desired_force_elbow) +  g_matrix;
            ROS_WARN_STREAM("force control : "<<forceControl_elbow_intern.update(force_elbow, desired_force_elbow));
            
        }
        
         
                
        // *********************************  Here qdd is calculated from tau and then qd and q are obtained by integrating


        qdd =(tao - g_matrix - (b_matrix + c_matrix)* qd)/m_matrix;
        
        qd = delta_t*qdd + qd;
        
        
        q_tot = delta_t*qd + q;
        
        if (q_tot >= 1.7 || q_tot <= 0.17 ){
            qdd = 0;
            qd = 0;
            
        }
        else {
            q = q_tot;
        }

        ROS_WARN_STREAM("time: "<<time[0]);
        ROS_WARN_STREAM("time 1: "<<time[1]);
        ROS_WARN_STREAM("q: "<<q);

        ROS_WARN_STREAM("q_0: "<<q_cos[0]);
        ROS_WARN_STREAM("q_1: "<<q_cos[1]);
        ROS_WARN_STREAM("force_d: "<<force_elbow_down);
        ROS_WARN_STREAM("force_u: "<<force_elbow_up);
        


        // ********************************* Preparation of the data to share with the ESP32

        q_ESP3[0] = q;
        q_ESP3[1] = 0;
        msg_q_ESP3.data = q_ESP3;
        exo_control_pub_q_ESP3.publish(msg_q_ESP3);


        // *********************************  Preparation of the state variables to publish

        q_state[0] = q* 180 / M_PI;
        q_state[1] = qd* 180 / M_PI;
        q_state[2] = qdd* 180 / M_PI;

        msg_q_state.data = q_state[0];

        exo_control_pub_q_state.publish(msg_q_state);

        r.sleep();
    }
    
    
    return 0; 

}