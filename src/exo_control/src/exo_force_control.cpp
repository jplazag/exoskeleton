/* Force Control
This code file holds all the methods used to implement a force control, which reacts to the pressure applied to the skin patch sensors attached to the
exoskeleton

Methods:

ForceControl: Constructor of the ForceControl object which takes the lenght of the exo's links
init: set the start flag to false in order to begin with the control
update: generate the tao based on the admittance control, which uses the applied and desired force with the respectiv control constant.
 */

#include<exo_control/exo_force_control.h>


namespace ExoControllers{

    ForceControl::ForceControl(double L2)
    {
        ROS_INFO_STREAM("Force Controller Created");
        
        std::string ns="~force_ctrl";
        std::stringstream s;
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("force m_kp: \n"<<m_kp);

        m_L2 = L2;
        m_startFlag = false;
        m_tao = 0;
    }

    ForceControl::~ForceControl()
    {
    }

    bool ForceControl::init()
    {
        m_startFlag = false;        
        return true;
    }

    double ForceControl::update(double Ws, double Wds)
    {
        if(!m_startFlag)
        {
            m_startFlag = true;
        }

        
        m_tao = m_L2 * m_kp * (Ws - Wds); //Elbow without wrist
                

        

        return m_tao;
    }

}
