/* Author: Joshua Smith
 * Date:
 *
 * Description:
 */

#include <force_torque_comedi_ati.hpp>
#include <rtt/Component.hpp>

FTKuka::FTKuka(std::string const & name) :
		RTT::TaskContext(name) {
	// constructor:
	
}

bool FTKuka::configureHook() {
	// intializations and object creations go here. Each component should run this before being able to run
	in_ext_torques_port.setName("in_ext_torques");
	in_ext_torques_flow = RTT::NoData;
	ports()->addPort(in_ext_torques_port);
	in_robot_status_port.setName("in_robot_status");
	in_robot_status_flow = RTT::NoData;
	ports()->addPort(in_robot_status_port);
	wrench_port.setName("out_wrench");
	wrench_data = rstrt::dynamics::Wrench();
	wrench_data.force.setZero();
	wrench_data.torque.setZero();
	wrench_port.setDataSample(wrench_data);
	ports()->addPort(wrench_port);
	
	return true;
}

bool FTKuka::startHook() {
	// this method starts the component
	return true;
}

void FTKuka::updateHook() {
	in_robot_status_flow = in_robot_status_port.read(in_robot_status_var);
	in_ext_torques_flow = in_ext_torques_port.read(in_ext_torques_var);
	if(in_robot_status_flow==RTT::NoData||in_ext_torques_flow==RTT::NoData){
		return;
	}

	kdl_jointstate.q.data = in_robot_status_var.angles.cast<double>();
	jnt_to_jac_solver->JntToJac(kdl_jointstate.q,jac_kdl,armChain_KDL.getNrOfSegments());
	jac = jac_kdl.data.cast<float>().transpose();
	F = jac.transpose*(jac*jac.transpose()).inverse()*in_ext_torques_var.torques;
	wrench_data.forces = F.head<3>();
	wrench_data.torques = F.tail<3>();
	wrench_port.write(wrench_data);
	
	// this is the actual body of a component. it is called on each cycle
	
	/*
	wrench_data.forces= tip_ft_rotation*wrench_data.forces;
	wrench_data.torques= tip_ft_rotation*wrench_data.torques + tip_ft_translation.cross(wrench_data.forces);
	*/

}

void FTKuka::stopHook() {
	// stops the component (update hook wont be  called anymore)
}

void FTKuka::cleanupHook() {
	// cleaning the component data
}
bool FTKuka::loadURDFAndSRDF(const std::string &URDF_path,
		const std::string &SRDF_path) {
	if (!_models_loaded) {
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();
		RTT::log(RTT::Info)<<"CHECKING EXISTANCE OF URDF AND SRDF"<< RTT::endlog();
		assert(exists_test(_urdf_path) == true);
		assert(exists_test(_srdf_path) == true);
		RTT::log(RTT::Info)<<"URDF AND SRDF EXISTS"<< RTT::endlog();
		_models_loaded = xbot_model.init(_urdf_path, _srdf_path);
		RTT::log(RTT::Info)<<"XBOT INITIALISED"<< RTT::endlog();
		for (unsigned int i = 0; i < xbot_model.get_chain_names().size(); ++i) {
			std::vector<std::string> enabled_joints_in_chain_i;
			xbot_model.get_enabled_joints_in_chain(
					xbot_model.get_chain_names()[i], enabled_joints_in_chain_i);
			for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
				RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j]
						<< RTT::endlog();
		}
		xml_model = "";
		std::fstream xml_file(URDF_path.c_str(), std::fstream::in);
		if (xml_file.is_open()) {
			while (xml_file.good()) {
				std::string line;
				std::getline(xml_file, line);
				xml_model += (line + "\n");
			}
			xml_file.close();
		}
		if (!p.initTreeAndChainFromURDFString(xml_model, tip_name, ft_name,
			robot_tree, ftChain_KDL)) {
			RTT::log(RTT::Error) << "[ DLW " << this->getName()
				<< "] URDF could not be parsed !" << RTT::endlog();

			// TODO add proper error handling!
			return false;
		}
		unsigned int segments = ftChain_KDL.getNrOfSegments();
		tip_ft_frame = KDL::Frame::Identity();
		for(int i =0; i<segments;i++){
			KDL::Segment tempSeg = ftChain_KDL.getSegment(i);
			tip_ft_frame= tempSeg.getFrameToTip()*tip_ft_frame;
		}
		tip_ft_rotation = Eigen::Map<Eigen::Matrix3d>(tip_ft_frame.M.data).cast<float>();
		tip_ft_translation = Eigen::Map<Eigen::Vector3d>(tip_ft_frame.p.data).cast<float>();
		if (!p.initTreeAndChainFromURDFString(xml_model, base_name, end_eff_name,
			robot_tree2,armChain_KDL)) {
			RTT::log(RTT::Error) << "[ DLW " << this->getName()
				<< "] URDF could not be parsed !" << RTT::endlog();

			// TODO add proper error handling!
			return false;
		}
		jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(armChain_KDL));
		int DOF = armChain_KDL.getNrOfJoints();
                in_robot_status_var.angles.resize(DOF);
		in_robot_status_var.velocities.resize(DOF);
		in_robot_status_var.torques.resize(DOF);
		in_robot_status_var.angles.setZero();
		in_robot_status_var.velocities.setZero();
		in_robot_status_var.torques.setZero();
				
		jac_kdl.resize(DOF);
                jac.resize(DOF,6);
		in_ext_torques_var.torques.resize(DOF);
		in_ext_torques_var.torques.setZero();
		F.resize(6);
		F.setZero();

	} else
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!"
				<< RTT::endlog();

	return _models_loaded;
}

void FTKuka::setFTandTip(std::string ft_name,std::string tip_name){
	this->ft_name = ft_name;
	this->tip_name = tip_name;
}
void FTKuka::setBaseandEndEff(std::string base, std::string endeff){
	base_name = base;
	end_eff_name = endeff;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(FTKuka)
