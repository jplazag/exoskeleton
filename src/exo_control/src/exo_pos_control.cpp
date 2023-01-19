#include <exo_control/exo_pos_control.h>

namespace ExoControllers{

    PosControl::PosControl(double L1, double L2, double L3, double m2, double m3, double b1, double b2, double k1, double k2, 
                            double theta1, double theta2, double gx, double gy, double gz, double I211, double I222, 
                            double I233, double I311, double I322, double I333)    
    {
        ROS_INFO_STREAM("Position Controller Created");
        
        std:string ns = "~pos_ctrl";
        std::stringstream s; 
        
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("pos m_kp: \n"<<m_kp);

        s.str("");
        s<<ns<<"/ki";
        ros::param::get(s.str(),m_ki);
        ROS_WARN_STREAM("pos m_ki: \n"<<m_ki);

        s.str("");
        s<<ns<<"/kd";
        ros::param::get(s.str(),m_kd);
        ROS_WARN_STREAM("pos m_kd: \n"<<m_kd);
        
        m_I211 = I211;
        m_I222 = I222;
        m_I233 = I233;
        m_I311 = I311;
        m_I322 = I322;
        m_I333 = I333;
        m_L1 = L1;
        m_L2 = L2;
        m_L3 = L3;
        m_m2 = m2;
        m_m3 = m3;
        m_b1 = b1;
        m_b2 = b2;
        m_k1 = k1;
        m_k2 = k2;
        m_theta1 = theta1;
        m_theta2 = theta2;
        m_gx = gx;
        m_gy = gy;
        m_gz = gz;

        m_q_des << 0, 0, 0,
                    0, 0, 0,
                    0, 0, 0;

        m_tao = Vector3d::Zero();

        m_startFlag = false;

        m_timeStart = 0.0;
        m_qStart = Matrix3d::Zero(); 
        m_qEnd = Matrix3d::Zero(); 
        m_timeEnd = 0.0;
        q_desired = {0, 0, 0};
        exo_control_pub_q_desired = n.advertise<std_msgs::Float64MultiArray>("q_des", 1000); 
    }

    PosControl::~PosControl()
    {
    }


    bool PosControl::get_m_startFlag(){

        return m_startFlag;
    }

    // 3. Trajectory generation
    void PosControl::trajGen(double qi, double qf, double qpi,
                             double qpf, double qppi, double qppf,
                             double t0, double tf, double tc,
                             bool startFlag, int q_ind)
    {
      MatrixXd static aq1(2,6);
      VectorXd aq(6);
      VectorXd static q(6);
      MatrixXd static T(6,6);
      VectorXd p(6);
      VectorXd v(6);
      VectorXd a(6);

      if(!startFlag){
        q << qi,qf,qpi,qpf,qppi,qppf;

        T <<     1, t0, pow(t0,2), pow(t0,3),   pow(t0,4),    pow(t0,5),
                 1, tf, pow(tf,2), pow(tf,3),   pow(tf,4),    pow(tf,5),
                 0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3),   5*pow(t0,4), 
                 0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3),   5*pow(tf,4),
                 0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3), 
                 0, 0, 2, 6*tf, 12*pow(tf,2), 20*pow(tf,3);     


        aq1.block<1,6>(q_ind-1,0) = T.colPivHouseholderQr().solve(q);
      }

      if(tc<=tf)
      {
        aq = aq1.block<1,6>(q_ind-1,0);
        p << 1, tc, pow(tc,2), pow(tc,3), pow(tc,4), pow(tc,5);
        v << 0, 1, 2*tc, 3*pow(tc,2), 4*pow(tc,3), 5*pow(tc,4);
        a << 0, 0, 2, 6*tc, 12*pow(tc,2), 20*pow(tc,3);
        m_q_des(q_ind, 0) = p.transpose()*aq;
        m_q_des(q_ind, 1) = v.transpose()*aq;
        m_q_des(q_ind, 2) = a.transpose()*aq;
      }
      else{ 
        m_q_des(q_ind, 0) = qf;
        m_q_des(q_ind, 1) = qpf;
        m_q_des(q_ind, 2) = qppf;
        m_startFlag = false;
      }
    }

    // 4. regressor
    Vector3d PosControl::YrTheta(Vector3d q, Vector3d qd, Vector3d qdr, Vector3d qddr)
    {
        Vector3d YrTheta;
        YrTheta << 0,
        m_L2*m_m2/2*(-m_gx*sin(q[1]) + m_gy*cos(q[1])) - m_gx*m_m3*(m_L2 + m_L3 * cos(q[2]))*sin(q[1]) + m_gy*m_m3*(m_L2 + m_L3 * cos(q[2]))*cos(q[1]) - m_k1*(m_theta1 - q[1]) + (m_b1 + (m_I311 * cos(q[2]) - m_I322 * cos(q[2]) - m_L2 * m_L3 * m_m3 - pow(m_L3,2) * m_m3 * cos(q[2])) * sin(q[2]) * qd[2]) * qdr[1] + ((m_I311 * cos(q[2]) - m_I322 * cos(q[2]) - m_L2 * m_L3 * m_m3 - pow(m_L3,2) * m_m3 * cos(q[2])) * sin(q[2]) * qd[1]) * qdr[2] + (m_I233 + m_I311 * pow(sin(q[2]),2) + m_I322 * pow(cos(q[2]),2) + pow(m_L2,2) * (m_m2 / 4 + m_m3) + m_L3 * m_m3 * cos(q[2]) * (2 * m_L2 + m_L3 * cos(q[2]))) * qddr[1],
        -m_m3*m_L3*sin(q[2])*(m_gx*cos(q[1]) + m_gy*sin(q[1])) - m_k2*(m_theta2 - q[2]) + (m_I333 + pow(m_L3,2) * m_m3) * qddr[2] + m_b2 * qdr[2] + (-(m_I311 * cos(q[2]) - m_I322 * cos(q[2]) - m_L2 * m_L3 * m_m3 - pow(m_L3,2) * m_m3 * cos(q[2])) * sin(q[2]) * qd[1]) * qdr[1];

        return YrTheta;
    }


    bool PosControl::init(Matrix3d qEnd, double timeEnd)
    {
        m_qEnd = qEnd;
        m_timeEnd = timeEnd;

        m_startFlag = false;

        return true;
    }

    // 5. PD controller 
    Vector3d PosControl::update(double delta_t, Vector3d q, Vector3d qd, Vector3d qdd)
    {   
        
        if(!m_startFlag)
        {
            m_timeStart = ros::Time::now().toSec();
            m_qStart << q[0],qd[0],qdd[0],
                        q[1],qd[1],qdd[1],
                        q[2],qd[2],qdd[2];
            
            trajGen(m_qStart(1, 0),m_qEnd(1, 0),m_qStart(1, 1),m_qEnd(1, 1), //gen traj to initpos
                m_qStart(1, 2),m_qEnd(1, 2),0.0,m_timeEnd,0.0,m_startFlag, 1);

            trajGen(m_qStart(2, 0),m_qEnd(2, 0),m_qStart(2, 1),m_qEnd(2, 1), //gen traj to initpos
                m_qStart(2, 2),m_qEnd(2, 2),0.0,m_timeEnd,0.0,m_startFlag, 2);
            m_startFlag = true;
        }
        else
        {
\

            trajGen(m_qStart(1, 0),m_qEnd(1, 0),m_qStart(1, 1),m_qEnd(1, 1), //gen traj to initpos
                m_qStart(1, 2),m_qEnd(1, 2),0.0,m_timeEnd,ros::Time::now().toSec()-m_timeStart,m_startFlag, 1);

            trajGen(m_qStart(2, 0),m_qEnd(2, 0),m_qStart(2, 1),m_qEnd(2, 1), //gen traj to initpos
                m_qStart(2, 2),m_qEnd(2, 2),0.0,m_timeEnd,ros::Time::now().toSec()-m_timeStart,m_startFlag, 2);
        }
        
        Vector3d deltaQ = q - m_q_des.col(0);
        Vector3d deltaQd = qd - m_q_des.col(1);

        Vector3d qdr = m_q_des.col(1) - m_kp*deltaQ;
        Vector3d qddr = m_q_des.col(2) - m_kp*deltaQd; 

        Vector3d Sq = qd - qdr;
        m_tao = -m_kd*Sq;// + YrTheta(q, qd, qdr, qddr);

        ROS_WARN_STREAM("m_tao: "<<m_tao);

        //std_msgs::Float64MultiArray msg_q_desired;

        /* q_desired[0] = m_q_des;
        q_desired[1] = m_qd_des;
        q_desired[2] = m_qdd_des;

        msg_q_desired.data = q_desired;

        exo_control_pub_q_desired.publish(msg_q_desired); */

        return m_tao;
    }

}