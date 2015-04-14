#pragma once

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "OriCPD.h"
#include "Fade_2D.h"

namespace FM {

	typedef std::vector<cv::Point> PointSet;

	struct Triangle {
	public:
		int va;
		int vb;
		int vc;
		Triangle(){}
		Triangle(int a, int b, int c){
			va = a; vb = b; vc = c;
		}
	};	// C++中不需要显式typedef一个结构体。 4.14
	typedef std::vector<Triangle> TriMesh;


	static cv::Point SGCPDPointToCVPoint(SGCPD::CPointDouble& cpdp){
		return cv::Point(cpdp.x, cpdp.y);
	}

	static SGCPD::CPointDouble CVPointToSGCPDPoint(cv::Point& cvp){
		return SGCPD::CPointDouble(cvp.x, cvp.y);
	}

	static cv::Point Fade2DPointToCVPoint(GEOM_FADE2D::Point2& f2dp){
		return cv::Point(f2dp.x(), f2dp.y());
	}

	static GEOM_FADE2D::Point2 CVPointToFade2DPoint(cv::Point& cvp){
		return GEOM_FADE2D::Point2(cvp.x, cvp.y);
	}
}