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

double magnitude_a = 0;

double a_beta = 0;

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

    if(isnan(gx1) || isnan(gy1) || isnan(gz1)){
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




Vector4d prox(0, 0, 0, 0);

double prox_filtered = 0;
double force_elbow = 0;
double force_wrist = 0;

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
        
        force_elbow = msg.data[0].force[2] ;
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

    std::vector<double> q_ESP3 = {0, 0};
    // double q_ESP3;

    ros::Publisher exo_control_pub_q_ESP3 = n.advertise<std_msgs::Float64MultiArray>("q_control_publisher", 1); 
    // ros::Publisher exo_control_pub_q_ESP3 = n.advertise<std_msgs::Float64>("q_control_publisher", 1000);

    std::vector<double> q_state = {0, 0, 0};

    ros::Publisher exo_control_pub_q_state = n.advertise<std_msgs::Float64MultiArray>("q_state", 1); 

    ros::Subscriber exo_control_sub_gravity1 = n.subscribe("patch1", 1, chatterCallback_g);

    ros::Subscriber exo_control_sub_skinpatch2 = n.subscribe("patch2", 1, chatterCallback_p);

    ros::Subscriber exo_control_sub_skinpatch3 = n.subscribe("patch3", 1, chatterCallback_p);


    // *********************** Nodes for the wrist *****************************

    ros::Subscriber exo_control_sub_skinpatch4 = n.subscribe("patch4", 1, chatterCallback_p);

    ros::Subscriber exo_control_sub_skinpatch5 = n.subscribe("patch5", 1, chatterCallback_p);



    //*************************** q_des Publisher ******************************
    // ros::Publisher exo_control_pub_q_desired = n.advertise<std_msgs::Float64MultiArray>("q_des", 1000);


    ros::Rate r(100);

    

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
    
    

    Vector3d q(0, deg2rad(10), deg2rad(90));
    Vector3d qd(0, 0, 0);
    Vector3d qdd(0, 0, 0);

    Vector3d g_vec(g, 0, 0);
    Vector3d tao(0, 0, 0);

    //******************** Elbow Position Control *************************

    ExoControllers::PosControl posControl(L1, L2, m2, b1, k1, theta1, gx1*g, gy1*g);
    Vector3d qEnd;
    qEnd << deg2rad(40),0.0,0.0;
    double timeEnd = 2.5;
    posControl.init(qEnd,timeEnd);

    Matrix3d m_matrix = Matrix3d :: Zero () ;
    Matrix3d c_matrix = Matrix3d :: Zero ();
    //MatrixXd g_matrix(1,3);
    Vector3d g_matrix = Vector3d :: Zero ();
    Matrix3d b_matrix = Matrix3d :: Zero ();

    // double m_matrix; 
    // double c_matrix;
    // double g_matrix;
    // double b_matrix;



    


    


    double Ws = 0;

    Vector2d Ws_elbow(0, 0);
    Vector2d Wds_elbow( - 0.07, 0);

    ExoControllers::ForceControl forceControl_elbow_intern(L2, L3);
    forceControl_elbow_intern.init(Ws_elbow[0]);

    ExoControllers::ForceControl forceControl_elbow_extern(L2, L3);
    forceControl_elbow_extern.init(Ws_elbow[1]);

    Vector2d Ws_wrist(0, 0);
    Vector2d Wds_wrist( - 0.07, 0);

    ExoControllers::ForceControl forceControl_wrist_intern(L2, L3);
    forceControl_wrist_intern.init( Wds_wrist[0]);

    ExoControllers::ForceControl forceControl_wrist_extern(L2, L3);
    forceControl_wrist_extern.init(Wds_wrist[1]);

    





    std_msgs::Float64MultiArray msg_q_ESP3;

    std_msgs::Float64MultiArray msg_q_state;

    double q_tot[2];



    Matrix3d q_neu;

    

    while(ros::ok())
    {
        ros::spinOnce();

        g_vec[0] = gx1 * g;
        g_vec[1] = gy1 * g;

        m_matrix << I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + pow(L1,2)*(m2 + m3) + L1*L2*cos(q[1])*(m2 + 2*m3) + 2*L1*L3*m3*cos(q[1])*cos(q[2]) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + L1*L2*m2*cos(q[1])/2 + L1*m3*cos(q[1])*(L2 + L3*cos(q[2])) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), -L1*L3*m3*sin(q[1])*sin(q[2]),
                    I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + L1*L2*m2*cos(q[1])/2 + L1*m3*cos(q[1])*(L2 + L3*cos(q[2])) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), I233 + I311 * pow(sin(q[2]),2) + I322 * pow(cos(q[2]),2) + pow(L2,2) * (m2 / 4 + m3) + L3 * m3 * cos(q[2]) * (2 * L2 + L3 * cos(q[2])), 0,
                    -L1*L3*m3*sin(q[1])*sin(q[2]), 0, I333 + pow(L3,2) * m3;

        c_matrix << 0, 0, 0,
                    0, (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[2], (I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1],
                    0, -(I311 * cos(q[2]) - I322 * cos(q[2]) - L2 * L3 * m3 - pow(L3,2) * m3 * cos(q[2])) * sin(q[2]) * qd[1], 0;
        g_matrix << 0,
                    L2*m2/2*(-g_vec[0]*sin(q[1]) + g_vec[1]*cos(q[1])) - g_vec[0]*m3*(L2 + L3 * cos(q[2]))*sin(q[1]) + g_vec[1]*m3*(L2 + L3 * cos(q[2]))*cos(q[1]) - k1*(theta1 - q[1]),
                    -m3*L3*sin(q[2])*(g_vec[0]*cos(q[1]) + g_vec[1]*sin(q[1])) - k2*(theta2 - q[2]);
        b_matrix << 0,0,0,
                    0,b1,0,
                    0,0,b2;


        // m_matrix = I233 + L2*L2*m2/4;
        // c_matrix = 0;//-1.5*L1*L2*m2*sin(q1)*qd1;
        // g_matrix = - L2*g_vec[0]*m2*sin(q[1])/2 + L2*g_vec[1]*m2*sin(q[1])/2 ;//TODO 
        // b_matrix = b1 * qd[1];//TODO

        
        // ****************** Position control elbow *****************************
        
        // tao[1] = posControl.update(delta_t,q[1],qd[1],qdd[1]) + g_matrix;

        

        
        // ****************** Intern force control elbow *****************************

        Ws_elbow[0] = - force_elbow; //- prox_1;

        tao[1] = forceControl_elbow_intern.update(Ws_elbow[0], 3, q[2]) + g_matrix[1];

        // ****************** Extern force control elbow *****************************

        Ws_elbow[1] = 0; //- force_elbow; //- prox_1;

        tao[1] = forceControl_wrist_extern.update(Ws_elbow[1], 3, q[2]) + g_matrix[1];

        // ****************** Intern force control wrist *****************************


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
        

        
        
        
        


        
        // calculate qdd1 and integrate 

        qd[0] = 0;
        qdd[0] = 0;

        qdd= m_matrix.inverse() * (tao - g_matrix - (b_matrix + c_matrix) * qd);

        // qdd[1] =(tao[1] - g_matrix - b_matrix)/m_matrix;
        
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
        
        if (q_tot[1] >= 1.7 || q_tot[1] <= 0.6 ){ //other limitations
            qdd[2] = 0;
            qd[2] = 0;
        }
        else {
            q[2] = q_tot[1];
        }
        
        
        ROS_WARN_STREAM("tao: "<<tao[1]);
        ROS_WARN_STREAM("qdd: "<<qdd[1]);
        ROS_WARN_STREAM("qd: "<<qd[1]);
        ROS_WARN_STREAM("q: "<<q[1]);

        q_ESP3[0] = q[1];
        q_ESP3[1] = q[2];

        

        msg_q_ESP3.data = q_ESP3;
        exo_control_pub_q_ESP3.publish(msg_q_ESP3);

        q_state[0] = q[1];
        q_state[1] = qd[1];
        q_state[2] = qdd[1];

        msg_q_state.data = q_state;

        exo_control_pub_q_state.publish(msg_q_state);





        // std_msgs::Float64MultiArray msg_q_desired;

        // // q_desired[0] = m_q_des(1,0)*180/3.14159;
        // // q_desired[1] = m_q_des(1,1)*180/3.14159;
        // // q_desired[2] = m_q_des(1,2)*180/3.14159;

        // // msg_q_desired.data = q_desired;

        // msg_q_state.data = msg_q_state.data * 180/3.14159;

        // exo_control_pub_q_desired.publish(msg_q_state);

        // ros::spinOnce();
        r.sleep();
    }
    
    
    return 0; 

}
