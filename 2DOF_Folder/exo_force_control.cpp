#include<exo_control/exo_force_control.h>


namespace ExoControllers{

    ForceControl::ForceControl(double L2, double L3)
    {
        ROS_INFO_STREAM("Force Controller Created");
        
        std::string ns="~force_ctrl";
        std::stringstream s;
        s.str("");
        s<<ns<<"/kp";
        ros::param::get(s.str(),m_kp);
        ROS_WARN_STREAM("force m_kp: \n"<<m_kp);

        m_L2 = L2;
        m_L3 = L3;
        m_startFlag = false;
        m_tao = 0;
    }

    ForceControl::~ForceControl()
    {
    }

    bool ForceControl::init(double W_des)
    {
        m_W_des = W_des;
        m_startFlag = false;        
        return true;
    }

    double ForceControl::update(double Ws, double Wds, int n_func, double q3)
    {
        if(!m_startFlag)
        {
            m_startFlag = true;
        }

        switch(n_func){
            case 1:
                m_tao = (m_L2 + m_L3 * std::cos(q3) - 0.2 * std::cos(q3)) * m_kp * (Ws - Wds); //Elbow with wrist
                break;
            case 2:
                m_tao = - m_L3 * m_kp * (Ws - Wds);
                break;
            case 3:
                m_tao = m_L2 * m_kp * (Ws - Wds); //Elbow without wrist
                break;
            
        }

        

        return m_tao;
    }

}
