/* Author: Joshua Smith
 * Date:
 *
 * Description:
 */

#ifndef FTCOMATICOMPONENT_HPP
#define FTCOMATICOMPONENT_HPP

// RTT header files. Might missing some or some be unused
#include <rtt/RTT.hpp>
#include <comedilib.h>
#include <stdio.h>
#include <Eigen/src/Core/DenseBase.h>
#include <string>
#include <XBotCoreModel.h>
#include <rst-rt/dynamics/Wrench.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

#define ROBOT_DOF_SIZE 7

class FTComAti: public RTT::TaskContext {
public:
    FTComAti(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    void setFTandTip(std::string ft_name,std::string tip_name);

private:
	// Declare ports and their datatypes

    int subdev;
    Eigen::VectorXi chan;
    Eigen::VectorXf voltage_data;
    Eigen::MatrixXf calibration_matrix;
    rstrt::dynamics::Wrench wrench_data;
    int range;
    int aref;
    lsampl_t data,maxdata;
    int retval;
    comedi_t *dev;
    comedi_range* range_info;
    int i;

    XBot::XBotCoreModel xbot_model;
    std::string xml_model,ft_name, tip_name,name;
    bool _models_loaded;
    KDL::Tree robot_tree;
    KDL::Chain ftChain_KDL;
    Eigen::Matrix3f tip_ft_rotation;
    Eigen::Vector3f tip_ft_translation;
    KDL::Frame tip_ft_frame;
    
};

#endif // FTCOMATICOMPONENT_HPP
