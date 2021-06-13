#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;


class CameraPoseVisualization {
public:
	string m_marker_ns;

	CameraPoseVisualization(float r, float g, float b, float a);
	
	void setImageBoundaryColor(float r, float g, float b, float a=1.0);
	void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
	void setScale(double s);
	void setLineWidth(double width);

	void add_pose(const Vector3d& p, const Quaterniond& q);
	void reset();

	void publish_by(ros::Publisher& pub, const std_msgs::Header& header);
	void add_edge(const Vector3d& p0, const Vector3d& p1, float line_width=0.005);
	void add_loopedge(const Vector3d& p0, const Vector3d& p1);

private:
	vector<visualization_msgs::Marker> m_markers;
	std_msgs::ColorRGBA m_image_boundary_color;
	std_msgs::ColorRGBA m_optical_center_connector_color;
	double m_scale;
	double m_line_width;

	static const Vector3d imlt;
	static const Vector3d imlb;
	static const Vector3d imrt;
	static const Vector3d imrb;
	static const Vector3d oc  ;
	static const Vector3d lt0 ;
	static const Vector3d lt1 ;
	static const Vector3d lt2 ;
};
