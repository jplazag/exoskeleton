#ifndef EXO_ROBOT_POSCONTROL_H
#define EXO_ROBOT_POSCONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"
#include <vector>

using namespace Eigen;
using std::string;

namespace ExoControllers{

  class PosControl{
      private:
        ros::NodeHandle n;
        Matrix3d m_q_des;
        double m_kp;
        double m_kd;
        double m_ki;
        Vector3d m_tao;
        double m_I211;
        double m_I222;
        double m_I233;
        double m_I311;
        double m_I322;
        double m_I333;
        double m_L1;
        double m_L2;
        double m_L3;
        double m_m2;
        double m_m3;
        double m_b1; 
        double m_b2; 
        double m_k1;
        double m_k2;
        double m_theta1;
        double m_theta2;
        double m_gx;
        double m_gy;
        double m_gz;

        bool m_startFlag;

        double m_timeStart;
        Matrix3d m_qStart;
        Matrix3d m_qEnd;
        double m_timeEnd;

        std::vector<double> q_desired;

        ros::Publisher exo_control_pub_q_desired; 

      public:
        PosControl(double L1, double L2, double L3, double m2, double m3, double b1, double b2, double k1, double k2, 
                            double theta1, double theta2, double gx, double gy, double gz, double I211, double I222, 
                            double I233, double I311, double I322, double I333);
        ~PosControl();

        bool get_m_startFlag();

        void trajGen(double qi, double qf, double qpi,
                             double qpf, double qppi, double qppf,
                             double t0, double tf, double tc,
                             bool startFlag, int q_ind);

        Vector3d YrTheta(Vector3d q, Vector3d qd, Vector3d qdr, Vector3d qddr);
        
        bool init(Matrix3d qEnd, double timeEnd);

        Vector3d update(double delta_t, Vector3d q, Vector3d qd, Vector3d qdd);



  };

}

#endif