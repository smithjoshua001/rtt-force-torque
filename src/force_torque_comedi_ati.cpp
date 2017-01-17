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
	aref = AREF_GROUND;
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

}

void FTComAti::stopHook() {
	// stops the component (update hook wont be  called anymore)
}

void FTComAti::cleanupHook() {
	free(chan);
	// cleaning the component data
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(FTComAti)
