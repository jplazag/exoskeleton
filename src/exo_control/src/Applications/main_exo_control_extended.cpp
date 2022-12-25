#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <exo_control/exo_pos_control.h>
#include "std_msgs/Float64MultiArray.h"
#include "tum_ics_skin_msgs/SkinCellDataArray.h" 
#include <cmath> 
#include <exo_control/exo_force_control.h>

double deg2rad(double degree){
    return (degree * 3.14159265359/180);
}

double gx1 = 0;
double gy1 = 0;
double gz1 = 0;

double gx1n = 0;
double gy1n = 0;

double magnitude_a;

double a_beta;

void chatterCallback_g(const tum_ics_skin_msgs::SkinCellDataArray &msg)
{
    if(msg.data[0].cellId == 2){

        gx1 = msg.data[0].acc[0] ;
        gy1 = msg.data[0].acc[1] ;
        gz1 = msg.data[0].acc[2] ;

    }
    
    magnitude_a = sqrt(pow(gx1,2) + pow(gy1,2));

    gx1 = gx1 / magnitude_a ;
    gy1 = gy1 / magnitude_a ;

    a_beta = M_PI - std::atan2(-0.5699293974 , -0.821671881044706) ;

    gx1n = gx1 * std::cos(M_PI - a_beta) + gy1 * std::sin(M_PI - a_beta) ;
    gy1n = - gx1 * std::sin(M_PI - a_beta) + gy1 * std::cos(M_PI - a_beta);

    gx1 = gx1n;
    gy1 = gy1n;

}




double prox;

void chatterCallback_p(const tum_ics_skin_msgs::SkinCellDataArray &msg){    
    if (msg.data[0].prox[0] > 0.1){
        prox = msg.data[0].prox[0] ;
    } else {
        prox = 0;
    }       
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "exo_control",
            ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Publisher exo_control_pub_q1 = n.advertise<std_msgs::Float64>("q_control_publisher", 1000);
    std::vector<double> q_state = {0, 0, 0};

    ros::Publisher exo_control_pub_q_state = n.advertise<std_msgs::Float64MultiArray>("q_state", 1000); 
    ros::Subscriber exo_control_sub_gravity = n.subscribe("patch1", 1000, chatterCallback_g);

    std::vector<double> gravity = {0, 0, 0};

    ros::Publisher exo_control_pub_gravity = n.advertise<std_msgs::Float64MultiArray>("g", 1000); 

    ros::Subscriber exo_control_sub_proximity = n.subscribe("patch2", 1000, chatterCallback_p);
    ros::Rate r(200);

    double delta_t = 1/(double)200; 

    // load your params
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
    double L3;
    ns="~L3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),L3);
    double m2;
    ns="~m2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m2);
    double m3;
    ns="~m3";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),m3);
    double b1;
    ns="~b1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b1);
    double b2;
    ns="~b2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),b2);
    double k1;
    ns="~k1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k1);
    double k2;
    ns="~k2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),k2);
    double theta1;
    ns="~theta1";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta1);
    double theta2;
    ns="~theta2";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),theta2);
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
    double g;
    ns="~g";
    s.str("");
    s<<ns;
    ros::param::get(s.str(),g);

    //double tao = 0; 
    
    //init params 
    //double q1 = deg2rad(90); 
    //double qd1 = 0;
    //double qdd1 = 0; 

    Vector3d q;
    Vector3d qd;
    Vector3d qdd;
    q[0] = 0;
    qd[0] = 0;
    qdd[0] = 0;
    Vector3d g_vec;
    g_vec[2] = 0;
    Vector3d tao;
    tao[0] = 0;

    // static g 
    //double gx = g;
    //double gy = 0;
    //double gz = 0;


    Matrix3d m_matrix;
    Matrix3d c_matrix;
    //MatrixXd g_matrix(1,3);
    Vector3d g_matrix;
    Matrix3d b_matrix;

    
    /*double m_matrix; 
    double c_matrix;
    double g_matrix;
    double b_matrix;*/

    /*ExoControllers::PosControl posControl(L1, L2, L3, m2, m3, b1, b2, k1, k2, theta1, theta2, gx, gy, gz,
     I211, I222, I233, I311, I322, I333);
    Vector3d qEnd;
    qEnd << deg2rad(45),0.0,0.0;
    double timeEnd = 2.5;
    posControl.init(qEnd,timeEnd);*/

    ExoControllers::ForceControl forceControl(L2);
    double W_des = 0;
    double Ws = 0;
    forceControl.init(W_des);

    std_msgs::Float64 msg;
    std_msgs::Float64MultiArray msg_q_state;
    std_msgs::Float64MultiArray msg_gravity;

    double q_tot[2];
    
    while(ros::ok())
    {        
        ros::spinOnce();
        //m_matrix = I233 + L2*L2*m2/4;
        //c_matrix = 0;
        //g_matrix = -L2*gx*m2*sin(q1)/2 + L2*gy*m2*cos(q1)/2 - k1*(theta1-q1); 
        //b_matrix = b1;
        //I233 - I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + L1 * L2 * cos(q[1]) * (m2 /2 + m3 * cos)
        // calculate qdd1 and integrate 
        //tao = posControl.update(delta_t,q1,qd1,qdd1);
        //ROS_INFO_STREAM("Deb1");
        g_vec[0] = gx1 * g;
        g_vec[1] = gy1 * g;
        //ROS_INFO_STREAM("Deb2");
        m_matrix << 1, 2, 3,
                    0, I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), 0,
                    0, 0, I333 + pow(L3,2) * m3;

        c_matrix << 0, 0, 0,
                    0, (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[2], (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1],
                    0, -(I311 * cos(q[2]) + I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1], 0;
        g_matrix << 0,
                    L2*m2/2*(-g_vec[0]*sin(q[1]) + g_vec[1]*cos(q[1])) - g_vec[0]*m3*(L2 + L3 * cos(q[2]))*sin(q[1]) + g_vec[1]*m3*(L2 + L3 * cos(q[2]))*cos(q[1]) - k1*(theta1 - q[1]),
                    -m3*L3*sin(q[2])*(g_vec[0]*cos(q[1]) + g_vec[1]*sin(q[1])) - k2*(theta2 - q[2]);
        b_matrix << 0,0,0,
                    0,b1,0,
                    0,0,b2;
        //ROS_INFO_STREAM("Deb3");
        Ws = -prox;
        tao[1] = forceControl.update(Ws) + g_matrix[1];
        
        qdd= m_matrix.inverse() * (tao - g_matrix - (b_matrix + c_matrix) * qd);
        
        qd = delta_t*qdd + qd;

        

        q_tot[0] = delta_t*qd[1] + q[1];
        q_tot[1] = delta_t*qd[2] + q[2];
        //ROS_INFO_STREAM("Deb4");
        
        if (q_tot[0] >= 1.7 || q_tot[0] <= 0.17 ){
            qdd[1] = 0;
            qd[1] = 0;
        }
        else {
            q[1] = q_tot[0];
        }
        
        if (q_tot[1] >= 1.7 || q_tot[1] <= 0.17 ){ //other limitations
            qdd[2] = 0;
            qd[2] = 0;
        }
        else {
            q[2] = q_tot[1];
        }

        /*ROS_WARN_STREAM("qdd1 "<<qdd1);
        ROS_WARN_STREAM("qd1 "<<qd1);
        ROS_WARN_STREAM("q1 "<<q1);*/



        /*msg.data = q1; 

        exo_control_pub_q1.publish(msg);

        q_state[0] = q1 ;
        q_state[1] = qd1;
        q_state[2] = qdd1;

        msg_q_state.data = q_state;

        exo_control_pub_q_state.publish(msg_q_state);*/
        
        gravity[0] = prox;
        gravity[1] = gy1;
        gravity[2] = magnitude_a;

        msg_gravity.data = gravity;

        exo_control_pub_gravity.publish(msg_gravity);

        //
        r.sleep();
    }
    
    return 0; 

}
