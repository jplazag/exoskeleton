#ifndef EXO_ROBOT_FORCECONTROL_H
#define EXO_ROBOT_FORCECONTROL_H

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace Eigen;
using std::string;

namespace ExoControllers{
    
    class ForceControl{
        private:
            ros::NodeHandle n;
            double m_L2;
            double m_L3;
            double m_kp;
            bool m_startFlag;
            double m_W_des;
            double m_tao;
            
        public:
            ForceControl(double L2, double L3);
            ~ForceControl();
            bool init(double W_des);
            double update(double Ws, double Wds, int n_func, double q3);

    };
}

#endif