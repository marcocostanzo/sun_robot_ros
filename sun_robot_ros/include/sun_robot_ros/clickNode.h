
#include <sun_robot_lib/Robot.h>


namespace sun{

class clickNode
{
private:
    
    //! Robot object
    std::shared_ptr<Robot> robot_;

    //! state
    TooN::Vector<> qDH_k_;
    UnitQuaternion quat_k_1;

    //! Params
    TooN::Vector< 6, int > cartesian_mask_;
    double clik_gain_; // should be gain/Ts
    double Ts_;
    double second_obj_gain_;  // should be gain/Ts
    TooN::Vector<> jointDH_target_second_obj_;
    TooN::Vector<> joint_weights_second_obj_;

    //! Cbs
    std::function<void(TooN::Vector<>, TooN::Vector<>)> joint_publish_fcn_;

public:
    clickNode(/* args */);
    ~clickNode();


//Note: for velocity mode it is sufficient clik_gain_=0;
void clik_core(
    const TooN::Vector<3>& pd_k, 
    const UnitQuaternion& quat_d_k, 
    const TooN::Vector<>& dpd_k,
    const TooN::Vector<>& omega_d_k
    )
{

    TooN::Vector<> dqDH_k = TooN::Zeros(robot_->getNumJoints());
    TooN::Vector<6> error_k;
    
    qDH_k_ = //<- qDH at time k+1
        robot_->clik(   
            qDH_k_, //<- qDH now, time k
            pd_k, // <- desired position
            quat_d_k, // <- desired quaternion
            quat_k_1,// <- quaternion at last time (to ensure continuity)
            dpd_k, // <- desired translational velocity
            omega_d_k, //<- desired angular velocity
            cartesian_mask_,// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
            clik_gain_,// <- CLIK Gain
            Ts_,// <- Ts, sampling time
            second_obj_gain_, // <- Gain for second objective  
            jointDH_target_second_obj_, //<- target for joint position (used into the second objective obj)
            joint_weights_second_obj_, // <- weights for joints in the second objective       
            //Return Vars
            dqDH_k, // <- joints velocity at time k+1
            error_k, //<- error vector at time k
            quat_k_1 // <- Quaternion at time k (usefull for continuity in the next call of these function)
    );

    TooN::Vector<> qR = robot_->joints_DH2Robot( qDH_k_ );
    TooN::Vector<> dqR = robot_->jointsvel_DH2Robot( dqDH_k );

    safety_check(qR, dqR);

    joint_publish_fcn_( qR, dqR );

}

};

} // Namespace sun