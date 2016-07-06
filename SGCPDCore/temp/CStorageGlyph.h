#pragma once
#include "CCurvePoint.h"
#include <vector>
#include <string>

namespace GlyphVectorization {
	class CStorageGlyph {
		public:
			short m_sContourNumber;				//轮廓段的个数
			short m_sPointNumber;				//轮廓点的个数
			std::vector<CCurvePoint> m_vPoints;	//关键点的个数
			std::vector<short> m_vContours;		//每个轮廓段在m_vPoints中的下标
			int m_nUnicode;						//字的unicode编码
			std::string m_strName;				//字库的名字
	};
}
