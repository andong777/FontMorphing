#pragma once
#include "CCurvePoint.h"
#include <vector>
#include <string>

namespace GlyphVectorization {
	class CStorageGlyph {
		public:
			short m_sContourNumber;				//�����εĸ���
			short m_sPointNumber;				//������ĸ���
			std::vector<CCurvePoint> m_vPoints;	//�ؼ���ĸ���
			std::vector<short> m_vContours;		//ÿ����������m_vPoints�е��±�
			int m_nUnicode;						//�ֵ�unicode����
			std::string m_strName;				//�ֿ������
	};
}
