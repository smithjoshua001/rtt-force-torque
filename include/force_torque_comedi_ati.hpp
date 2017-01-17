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

private:
	// Declare ports and their datatypes

    int subdev;
    Eigen::VectorXi chan;
    Eigen::VectorXf voltage_data;
    Eigen::MatrixXf calibration_matrix;
    Eigen::VecotrXf force_data;
    int range;
    int aref;
    lsampl_t data,maxdata;
    int retval;
    comedi_t *dev;
    comedi_range* range_info;
    int i;
};

#endif // FTCOMATICOMPONENT_HPP
