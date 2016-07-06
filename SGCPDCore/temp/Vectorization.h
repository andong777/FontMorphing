#pragma once
#include "CStorageGlyph.h"
#include "CMyGlyph.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace GlyphVectorization {
	class Vectorization {
	    private:
            /*图像数据*/
			cv::Mat m_OriImg;		//输入原图
			cv::Mat m_BinImg;		//二值图
			cv::Mat m_RecImg;		//矢量化结果

			/*轮廓点数据*/
			CStorageGlyph m_TTGlyph;//TrueType轮廓
			CMyGlyph m_VecGlyph;	//矢量化轮廓

			/*算法参数*/
			static const int MaximaErrT = 2;//控制删除代价算法得到的关键点个数，成反比
			static const int AverErrT = 1;//控制曲线拟合的参数

		public:
            /*载入图像数据*/
			void LoadImg(std::string imgAdd);

            /*设置数据为二值图像*/
			void SetData(std::vector<std::vector<bool> > pixels);

            /*从二值图生成矢量结果*/
			void GenerateContour();		

			/*返回矢量化结果*/
			CStorageGlyph GetGlyphContour();		

	    private:
			/*二值化图像*/
			void BinarizeImage(cv::Mat &src, cv::Mat &dst, int flag);

			/*从二值图得到轮廓*/
			void TraceContour();

			/*删除代价算法找出关键点*/
			void DeletionCost();
			double calculateDeleteCost(int st, int mi, int ed, int cn);
			double PointsDis(cv::Point p0, cv::Point p1, cv::Point p2);

            /*曲线拟合得到TrueType轮廓*/
			void CurveFitting();
			void FitCurveI(int cno, int pno);

			/*将矢量化结果写入ttGlyph中*/
			void SetGlyphContour();
	};
}