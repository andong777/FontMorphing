#pragma once

namespace GlyphVectorization {
	class CCurvePoint {
		public:
			short x;		//以图像的左下角为坐标原点，x坐标
			short y;		//y坐标
			char m_cTag;	//1：on curve， 0：off,  https://developer.apple.com/fonts/TTRefMan/RM06/Chap6glyf.html
			CCurvePoint(short x, short y, bool online) {
				this->x = x;
				this->y = y;
				this->m_cTag = online ? 1 : 0;
			}
	};
}