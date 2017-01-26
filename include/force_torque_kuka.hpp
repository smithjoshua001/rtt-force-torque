/* Author: Joshua Smith
 * Date:
 *
 * Description:
 */

#ifndef FTKukaCOMPONENT_HPP
#define FTKukaCOMPONENT_HPP

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <stdio.h>
#include <Eigen/src/Core/DenseBase.h>
#include <string>
#include <XBotCoreModel.h>
#include <rst-rt/dynamics/Wrench.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jacboian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#define ROBOT_DOF_SIZE 7

class FTKuka: public RTT::TaskContext {
public:
    FTKuka(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void setFTandTip(std::string ft_name,std::string tip_name);
    void setBaseandEndEff(std::string base, std::string endeff);

private:
	// Declare ports and their datatypes

    rstrt::dynamics::Wrench wrench_data;
    RTT::OutputPort<rstrt::dynamics::Wrench> wrench_port;
    
    RTT::InputPort<rstrt::dynamics::JointTorques> in_ext_torques_port;
    RTT::InputPort<rstrt::robot::JointState> in_robot_status_port;

    RTT::FlowStatus in_ext_torques_flow;
    RTT::FlowStatus in_in_robot_status_flow;

    rstrt::dynamics::JointTorques in_ext_torques_var;
    rstrt::robot::JointState in_robot_status_var;

    XBot::XBotCoreModel xbot_model;
    std::string xml_model,ft_name, tip_name,name,base_name,end_eff_name;
    bool _models_loaded;
    KDL::Tree robot_tree,robot_tree2;
    KDL::Chain ftChain_KDL,armChain_KDL;
    
    Eigen::Matrix3f tip_ft_rotation;
    Eigen::Vector3f tip_ft_translation;
    KDL::Frame tip_ft_frame;
    Eigen::VectorXf F;

    KDL::JntArrayVel kdl_jointstate;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;

    KDL::Jacobian jac_kdl;
    Eigen::MatrixXf jac;
    
};

#endif // FTKukaCOMPONENT_HPP
