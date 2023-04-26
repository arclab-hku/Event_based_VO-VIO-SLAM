#ifndef __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__
#define __DYNAMIC_RECONFIGURE__CONFIG_TOOLS__

#include <dv_ros_messaging/messaging.hpp>

#include "Config.h"
#include "ConfigDescription.h"
#include "Group.h"
#include "ParamDescription.h"
#include "Reconfigure.h"

#include <cstring>
#include <string>
#include <vector>

namespace dynamic_reconfigure {

class ConfigTools {
	using Allocator = boost::container::allocator<void>;

public:
	static DV_ROS_MSGS(dynamic_reconfigure::Config)::_bools_type &getVectorForType(
		DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const bool /*val*/) {
		return set.bools;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::Config)::_ints_type &getVectorForType(
		DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const int /*val*/) {
		return set.ints;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::Config)::_strs_type &getVectorForType(
		DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const std::string & /*val*/) {
		return set.strs;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::Config)::_doubles_type &getVectorForType(
		DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const double /*val*/) {
		return set.doubles;
	}

	static const DV_ROS_MSGS(dynamic_reconfigure::Config)::_bools_type &getVectorForType(
		const DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const bool /*val*/) {
		return set.bools;
	}

	static const DV_ROS_MSGS(dynamic_reconfigure::Config)::_ints_type &getVectorForType(
		const DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const int /*val*/) {
		return set.ints;
	}

	static const DV_ROS_MSGS(dynamic_reconfigure::Config)::_strs_type &getVectorForType(
		const DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const std::string & /*val*/) {
		return set.strs;
	}

	static const DV_ROS_MSGS(dynamic_reconfigure::Config)::_doubles_type &getVectorForType(
		const DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const double /*val*/) {
		return set.doubles;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::BoolParameter) makeKeyValuePair(const std::string &name, const bool val) {
		DV_ROS_MSGS(dynamic_reconfigure::BoolParameter) param;
		param.name  = name;
		param.value = val;
		return param;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::IntParameter) makeKeyValuePair(const std::string &name, const int val) {
		DV_ROS_MSGS(dynamic_reconfigure::IntParameter) param;
		param.name  = name;
		param.value = val;
		return param;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::StrParameter)
		makeKeyValuePair(const std::string &name, const std::string &val) {
		DV_ROS_MSGS(dynamic_reconfigure::StrParameter) param;
		param.name  = name;
		param.value = val;
		return param;
	}

	static DV_ROS_MSGS(dynamic_reconfigure::DoubleParameter)
		makeKeyValuePair(const std::string &name, const double val) {
		DV_ROS_MSGS(dynamic_reconfigure::DoubleParameter) param;
		param.name  = name;
		param.value = val;
		return param;
	}

	template<class T>
	static void appendParameter(dynamic_reconfigure::Config &set, const std::string &name, const T &val) {
		getVectorForType(set, val).push_back(makeKeyValuePair(name, val));
	}

	template<class VT, class T>
	static bool getParameter(const VT &vec, const std::string &name, T &val) {
		for (typename VT::const_iterator i = vec.begin(); i != vec.end(); ++i)
			if (strcmp(i->name.c_str(), name.c_str()) == 0) {
				val = i->value;
				return true;
			}
		return false;
	}

	template<class T>
	static bool getParameter(const DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const std::string &name, T &val) {
		return getParameter(getVectorForType(set, val), name, val);
	}

	template<class T>
	static void appendGroup(
		DV_ROS_MSGS(dynamic_reconfigure::Config) & set, const std::string &name, int id, int parent, const T &val) {
		DV_ROS_MSGS(dynamic_reconfigure::GroupState) msg;
		msg.name   = name;
		msg.id     = id;
		msg.parent = parent;
		msg.state  = val.state;
		set.groups.push_back(msg);
	}

	template<class T>
	static bool getGroupState(const DV_ROS_MSGS(dynamic_reconfigure::Config) & msg, const std::string &name, T &val) {
		for (const auto &i : msg.groups)
			if (std::strcmp(i.name.c_str(), name.c_str()) == 0) {
				val.state = i.state;
				return true;
			}
		return false;
	}

	static int size(DV_ROS_MSGS(dynamic_reconfigure::Config) & msg) {
		return msg.bools.size() + msg.doubles.size() + msg.ints.size() + msg.strs.size();
	}

	static void clear(DV_ROS_MSGS(dynamic_reconfigure::Config) & msg) {
		msg.bools.clear();
		msg.ints.clear();
		msg.strs.clear();
		msg.doubles.clear();
		msg.groups.clear();
	}
};

} // namespace dynamic_reconfigure

#endif
