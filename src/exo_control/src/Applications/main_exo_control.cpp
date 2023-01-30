#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include "std_msgs/Float64MultiArray.h"
#include "tum_ics_skin_msgs/SkinCellDataArray.h" 
#include <cmath> 
#include <exo_control/exo_force_control.h>
#include <algorithm>  


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
    if(msg.data[0].cellId == 2){ //Take the acceleration data from the skin patch near to the shoulder

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



// **************************** Proximity and force variables *************************************
Vector4d prox(0, 0, 0, 0);

double prox_filtered = 0;
double force_elbow_down = 0;
double force_elbow = 0;
double desired_force_elbow = 0;
double force_elbow_up = 0;
double force_wrist = 0;

// **************************** Callback funtion to take the data from skin patches *****************

void chatterCallback_p(const tum_ics_skin_msgs::SkinCellDataArray &msg){

    // cells_numbers: (direction: cell with wire connection to cell without cell connection)
    // patch1: 1, 2, 3
    // patch2: 4, 14 ,15
    // patch3: 11, 12, 13
    // patch4: 5, 9 ,10
    // patch5: 6, 7, 8

    if (msg.data[0].prox[0] > 0.7){
        prox_filtered = msg.data[0].prox[0];
    }else{
        prox_filtered = 0;
    }

    switch (msg.data[0].cellId){
        case 6: // Positive force
            prox[0] = prox_filtered ;
            break;
        
        case 8: // Negative force
            prox[1] = prox_filtered ;
            break;

        case 12:
            prox[2] = prox_filtered ;
            break;

        case 9:
            prox[3] = prox_filtered ;
            break;
        
    }

    if(msg.data[0].cellId == 15){
        
        force_elbow_down = msg.data[0].force[2] ;
    }
    if(msg.data[0].cellId == 13){
        
        force_elbow_up = msg.data[0].force[2] ;
    }
    if(msg.data[0].cellId == 9){
        
        force_wrist = msg.data[0].force[2] ;
    }
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
    std_msgs::Float64MultiArray msg_q_state;
    ros::Publisher exo_control_pub_q_state = n.advertise<std_msgs::Float64MultiArray>("q_state", 1); 


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




    // ******************************* Wrist parameters *********************

    double L3;
    ns="~L3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L3);
    double m3;
    ns="~m3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m3);
    double b2;
    ns="~b2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b2);
    double k2;
    ns="~k2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k2);
    double theta2;
    ns="~theta2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta2);
    double I311;
    ns="~I311";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I311);
    double I322;
    ns="~I322";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I322);
    double I333;
    ns="~I333";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),I333);
    
    

    //******************** Elbow Position Control *************************

    ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx1*g, gy1*g);
    Vector3d qEnd;
    double timeEnd;
    
    

    // ************************** Dynamic system matrices ***************************************

    // ************************** 2 DOF ***************************************

    // Matrix3d m_matrix = Matrix3d :: Zero () ;
    // Matrix3d c_matrix = Matrix3d :: Zero ();
    // //MatrixXd g_matrix(1,3);
    // Vector3d g_matrix = Vector3d :: Zero ();
    // Matrix3d b_matrix = Matrix3d :: Zero ();

    // ************************** 1 DOF ***************************************

    double m_matrix; 
    double c_matrix;
    double g_matrix;
    double b_matrix;

    // ************************** Initial conditions for dynamics ***************************************

    Vector3d q(0, deg2rad(80), deg2rad(90));
    Vector3d qd(0, 0, 0);
    Vector3d qdd(0, 0, 0);

    Vector3d g_vec(g, 0, 0);
    Vector3d tao(0, 0, 0);


    // ****************************** Calibration routine **********************************************
    int counter = 0;
    double force_elbow_down_calibrated = 0;
    double force_elbow_up_calibrated = 0;


    while(ros::ok() && counter < 20)
    {
        ros::spinOnce();

        force_elbow_down_calibrated += force_elbow_down;
        force_elbow_up_calibrated += force_elbow_down;
        counter +=1;
        
        
        if(counter == 20)
        {
            force_elbow_down_calibrated /= 20;
            force_elbow_up_calibrated /= 20;
        }

        ROS_WARN_STREAM("count: "<<counter);

        r.sleep();

    }

    // ************************** Initial conditions for force control ***************************************

    double Ws = 0;

    Vector2d Ws_elbow(0, 0);
    Vector2d Wds_elbow( force_elbow_down_calibrated, force_elbow_up_calibrated);

    ExoControllers::ForceControl forceControl_elbow_intern(L2, L3);
    forceControl_elbow_intern.init(Wds_elbow[0]);

    ExoControllers::ForceControl forceControl_elbow_extern(L2, L3);
    forceControl_elbow_extern.init(Wds_elbow[1]);

    Vector2d Ws_wrist(0, 0);
    Vector2d Wds_wrist( - 0.07, 0);

    ExoControllers::ForceControl forceControl_wrist_intern(L2, L3);
    forceControl_wrist_intern.init( Wds_wrist[0]);

    ExoControllers::ForceControl forceControl_wrist_extern(L2, L3);
    forceControl_wrist_extern.init(Wds_wrist[1]);

    

    double q_tot[2]; // Variable to test the range of the joints
    bool change_direction = false;
    bool up = true;
    bool down = true;
    
    bool still = false;
    


    while(ros::ok())
    {
        
        ros::spinOnce();

        // ************************** Definition of the g vector with the lectures of the skin-patch near to the shoulder ************************** 

        g_vec[0] = gx1 * g;
        g_vec[1] = gy1 * g;

        // ************************** Definition of the dynamic matrices for 2 DOF ************************** 



        // m_matrix << I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + pow(L1,2)*(m2 + m3) + L1*L2*cos(q[1])*(m2 + 2*m3) + 2*L1*L3*m3*cos(q[1])*cos(q[2]) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + L1*L2*m2*cos(q[1])/2 + L1*m3*cos(q[1])*(L2 + L3*cos(q[2])) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), -L1*L3*m3*sin(q[1])*sin(q[2]),
        //             I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + L1*L2*m2*cos(q[1])/2 + L1*m3*cos(q[1])*(L2 + L3*cos(q[2])) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), 0,
        //             -L1*L3*m3*sin(q[1])*sin(q[2]), 0, I333 + pow(L3,2) * m3;

        // c_matrix << 0, 0, 0,
        //             0, (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[2], (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1],
        //             0, -(I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1], 0;
        // g_matrix << 0,
        //             L2*m2/2*(-g_vec[0]*sin(q[1]) + g_vec[1]*cos(q[1])) - g_vec[0]*m3*(L2 + L3 * cos(q[2]))*sin(q[1]) + g_vec[1]*m3*(L2 + L3 * cos(q[2]))*cos(q[1]) - k1*(theta1 - q[1]),
        //             -m3*L3*sin(q[2])*(g_vec[0]*cos(q[1]) + g_vec[1]*sin(q[1])) - k2*(theta2 - q[2]);
        // b_matrix << 0,0,0,
        //             0,b1,0,
        //             0,0,b2;

        // ************************** Definition of the dynamic matrices for 1 DOF ************************** 

        m_matrix = I233 + L2*L2*m2/4;
        c_matrix = 0;//-1.5*L1*L2*m2*sin(q1)*qd1;
        g_matrix = - L2*g_vec[0]*m2*sin(q[1])/2 + L2*g_vec[1]*m2*sin(q[1])/2 ;//TODO 
        b_matrix = b1 ;//TODO



        
        // ****************** Position control elbow *****************************
        
        // tao[1] = posControl.update(delta_t,q[1],qd[1],qdd[1]) + g_matrix;

        
        
        // if(force_elbow_down > force_elbow_down_calibrated)
        // {

        //     // ****************** Down force control elbow *****************************

        //     Ws_elbow[0] = - force_elbow_down; 

        //     tao[1] = forceControl_elbow_intern.update(Ws_elbow[0], 3, q[2]) + g_matrix;

        // }
        // else if(force_elbow_up > force_elbow_up_calibrated)
        // {
        // // ****************** Up force control elbow *****************************

        //     Ws_elbow[1] = force_elbow_up; 

        //     tao[1] = forceControl_wrist_extern.update(Ws_elbow[1], 3, q[2]) + g_matrix;
        // }
        // else
        // {
        //     tao[1] = g_matrix;
        // }




        if(force_elbow_down > force_elbow_down_calibrated + 0.01  && (q[1] > deg2rad(10)))
        {
            

            down = true;

            if(up == true) 
            {
                change_direction = true;
                up = false;
            }
            force_elbow = - force_elbow_down;
            desired_force_elbow = - force_elbow_down_calibrated;

            if((change_direction || still || abs(q[1] - qEnd[0]) < 0.05))
            /* if(change_direction || still || !posControl.get_m_startFlag()) */
            {
                timeEnd = 1.5;
                if (q[1] - deg2rad(30) > deg2rad(10))
                    qEnd << q[1] - deg2rad(30),0.0,0.0;
                else
                    qEnd << deg2rad(10),0.0,0.0;

                
                posControl.init(qEnd,timeEnd);
                
            } 
            still = false;
            ROS_WARN_STREAM("condition: "<<1);

        }
        else if(force_elbow_up > force_elbow_up_calibrated + 0.01 && q[1] < deg2rad(95))
        {
            

            up = true;

            if(down == true) 
            {
                change_direction = true;
                down = false;
            }

            force_elbow = force_elbow_up;
            desired_force_elbow = force_elbow_up_calibrated;

            if((change_direction || still || (abs(q[1] - qEnd[0]) < 0.05)))
            {
                timeEnd = 1.5;
                if (q[1] + deg2rad(30) < deg2rad(100))
                    qEnd << q[1] + deg2rad(30),0.0,0.0;
                else
                    qEnd << deg2rad(95 ),0.0,0.0; 


                posControl.init(qEnd,timeEnd);
            }
            still = false;
            ROS_WARN_STREAM("condition: "<<2);
        }
        else
        {
            if(still == false)
            {
                timeEnd = 0.5;
                qEnd << q[1],qd[1],qdd[1];
                posControl.init(qEnd,timeEnd);
                still = true;

                force_elbow = desired_force_elbow;
            }
            
            ROS_WARN_STREAM("condition: "<<3);
        }

        

        tao[1] = 5 * forceControl_elbow_intern.update(force_elbow, desired_force_elbow, 3, q[2]) +  g_matrix  + posControl.update(delta_t,q[1],qd[1],qdd[1]);

        ROS_WARN_STREAM("force control : "<<forceControl_elbow_intern.update(force_elbow, desired_force_elbow, 3, q[2]));

        // ****************** Intern force control wrist *****************************

        /*
        

        if (prox[0] != 0 || prox[1] != 0){
            if(prox[0] != 0){
                Ws_wrist[1] =  -prox[0]/10;
            }else if(prox[1] != 0){
                Ws_wrist[1] = prox[1]/10;
            }else{
                Ws_wrist[1] = 0;
            }

            tao[2] = forceControl_wrist_extern.update(Ws_wrist[1], 2, q[2]) + g_matrix[2];

        }else{

        // ****************** Extern force control wrist *****************************
            if(prox[3] != 0){
                Ws_wrist[0] = - force_wrist; 
            }else{
                Ws_wrist[0] = - 0.07; 
            }
            

            tao[2] = forceControl_wrist_intern.update(Ws_wrist[0], 2, q[2]) + g_matrix[2];

        }
        

        */
        
        
        


        
        // *********************************  Here qdd is calculated from tau and then qd and q are obtained by integrating

        qd[0] = 0;
        qdd[0] = 0;

        qdd[1] =(tao[1] - g_matrix - (b_matrix + c_matrix)* qd[1])/m_matrix;
        
        qd = delta_t*qdd + qd;
        
        
        q_tot[0] = delta_t*qd[1] + q[1];
        q_tot[1] = delta_t*qd[2] + q[2];
        
        if (q_tot[0] >= 1.7 || q_tot[0] <= 0.17 ){
            qdd[1] = 0;
            qd[1] = 0;
        }
        else {
            q[1] = q_tot[0];
        }
        
        if (q_tot[1] >= 1.7 || q_tot[1] <= 0.6 ){ //other limitations
            qdd[2] = 0;
            qd[2] = 0;
        }
        else {
            q[2] = q_tot[1];
        }
        
        // ROS_WARN_STREAM("up force: "<<force_elbow_up);
        // ROS_WARN_STREAM("down force: "<<force_elbow_down);

        // ROS_WARN_STREAM("tao: "<<tao[1]);
        // ROS_WARN_STREAM("tao: "<<tao[1]);
        ROS_WARN_STREAM("qdd: "<<qdd[1]);
        ROS_WARN_STREAM("qd: "<<qd[1]);
        ROS_WARN_STREAM("q: "<<q[1]);


        // ********************************* Preparation of the data to share with the ESP32

        q_ESP3[0] = q[1];
        q_ESP3[1] = q[2];
        msg_q_ESP3.data = q_ESP3;
        exo_control_pub_q_ESP3.publish(msg_q_ESP3);


        // *********************************  Preparation of the state variables to publish

        q_state[0] = q[1]* 180 / M_PI;
        q_state[1] = qd[1]* 180 / M_PI;
        q_state[2] = qdd[1]* 180 / M_PI;

        msg_q_state.data = q_state;

        exo_control_pub_q_state.publish(msg_q_state);

        change_direction = false;

        r.sleep();
    }
    
    
    return 0; 

}
