#pragma once

#include "Singleton.h"
#include "Typedef.h"

using namespace boost::property_tree;
using namespace boost::property_tree::xml_parser;

class Sensor: public Singleton<Sensor>
{
public:
	inline double getX() { return mX; }
	inline double getY() { return mY; }
	inline double getZ() { return mZ; }
	inline double getPitch() { return mPitch; }
	inline double getYaw() { return mYaw; }
	inline double getRoll() { return mRoll; }

	inline double getMinX() { return mMinX; }
	inline double getMaxX() { return mMaxX; }
	inline double getMinY() { return mMinY; }
	inline double getMaxY() { return mMaxY; }
	inline double getMinZ() { return mMinZ; }
	inline double getMaxZ() { return mMaxZ; }

	void setX(double x) { mX = x; }
	void setY(double y) { mY = y; }
	void setZ(double z) { mZ = z; }
	void setPitch(double pitch) { mPitch = pitch; }
	void setYaw(double yaw) { mYaw = yaw; }
	void setRoll(double roll) { mRoll = roll; }

	inline void setPopsture(double x, double y, double z, double pitch, double yaw, double roll) {
		mX = x;
		mY = y;
		mZ = z;
		mPitch = pitch;
		mYaw = yaw;
		mRoll = roll;
	}

	inline void setRange(double minX, double maxX, double minY, double maxY, double minZ, double maxZ) {
		mMinX = minX;
		mMaxX = maxX;
		mMinY = minY;
		mMaxY = maxY;
		mMinZ = minZ;
		mMaxZ = maxZ;
	}

	bool loadSensorPostureData()
	{
		ptree pt;
		read_xml("Sensor_Setting.xml", pt);
		mX = pt.get_optional<double>("sensor_posture.x").get();
		mY = pt.get_optional<double>("sensor_posture.y").get();
		mZ = pt.get_optional<double>("sensor_posture.z").get();
		mPitch = pt.get_optional<double>("sensor_posture.pitch").get();
		mYaw = pt.get_optional<double>("sensor_posture.yaw").get();
		mRoll = pt.get_optional<double>("sensor_posture.roll").get();

		return true;
	}

	bool loadSensorRangeData()
	{
		ptree pt;
		read_xml("Range_Setting.xml", pt);
		mMinX = pt.get_optional<double>("sensing_range.range_width.min").get();
		mMaxX = pt.get_optional<double>("sensing_range.range_width.max").get();
		mMinY = pt.get_optional<double>("sensing_range.range_depth.min").get();
		mMaxY = pt.get_optional<double>("sensing_range.range_depth.max").get();
		mMinZ = pt.get_optional<double>("sensing_range.range_height.min").get();
		mMaxZ = pt.get_optional<double>("sensing_range.range_height.max").get();

		return true;
	}


	bool saveSensorPostureData()
	{
		ptree pt;
		pt.put("sensor_posture.x", mX);
		pt.put("sensor_posture.y", mY);
		pt.put("sensor_posture.z", mZ);
		pt.put("sensor_posture.pitch", mPitch);
		pt.put("sensor_posture.yaw", mYaw);
		pt.put("sensor_posture.roll", mRoll);

		write_xml("Sensor_Setting.xml", pt, locale(), xml_writer_make_settings<string>(' ', 2));
		return false;
	}

private:
	friend class Singleton<Sensor>;
	Sensor() {}
	~Sensor() {}


	//Posture
	double mX, mY, mZ, mPitch, mYaw, mRoll;
	//Range
	double mMinX, mMaxX, mMinY, mMaxY, mMinZ, mMaxZ;
};