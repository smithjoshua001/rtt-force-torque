/* Author: Joshua Smith
 * Date:
 *
 * Description:
 */

#include <force_torque_comedi_ati.hpp>
#include <rtt/Component.hpp>

FTComAti::FTComAti(std::string const & name) :
		RTT::TaskContext(name) {
	// constructor:
	subdev = 0;
	chan.resize(6);
	chan.setLinSpaced(6, 0, 5);
	range = 0;
	aref = AREF_DIFF;
}

bool FTComAti::configureHook() {
	// intializations and object creations go here. Each component should run this before being able to run
	dev = comedi_open("/dev/comedi0"); //may need to be changed
	if (dev == NULL) {
		comedi_perror("comedi_open");
		return false;
	}
	return true;
}

bool FTComAti::startHook() {
	// this method starts the component
	return true;
}

void FTComAti::updateHook() {
	// this is the actual body of a component. it is called on each cycle
	for (i = 0; i < 6; i++) {
		retval = comedi_data_read(dev, subdev, chan[i], range, aref, &data);
		if (retval < 0) {
			comedi_perror("comedi_data_read");
			return;
		}
		comedi_set_global_oor_behavior (COMEDI_OOR_NAN);
		range_info = comedi_get_range(dev, subdev, chan[i], range);
		maxdata = comedi_get_maxdata(dev, subdev, chan[i]);
		voltage_data[i] = comedi_to_phys(data, range_info, maxdata);
	}
	RTT::log(RTT::Error)<<voltage_data<<RTT::endlog();
	/*
	wrench_data.forces= tip_ft_rotation*wrench_data.forces;
	wrench_data.torques= tip_ft_rotation*wrench_data.torques + tip_ft_translation.cross(wrench_data.forces);
	*/

}

void FTComAti::stopHook() {
	// stops the component (update hook wont be  called anymore)
}

void FTComAti::cleanupHook() {
	free(chan);
	// cleaning the component data
}
bool FTComAti::loadURDFAndSRDF(const std::string &URDF_path,
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
		
		
	} else
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!"
				<< RTT::endlog();

	return _models_loaded;
}

void FTComAti::setFTandTip(std::string ft_name,std::string tip_name){
	this->ft_name = ft_name;
	this->tip_name = tip_name;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(FTComAti)
