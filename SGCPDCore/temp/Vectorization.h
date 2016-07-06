#pragma once
#include "CStorageGlyph.h"
#include "CMyGlyph.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace GlyphVectorization {
	class Vectorization {
	    private:
            /*ͼ������*/
			cv::Mat m_OriImg;		//����ԭͼ
			cv::Mat m_BinImg;		//��ֵͼ
			cv::Mat m_RecImg;		//ʸ�������

			/*����������*/
			CStorageGlyph m_TTGlyph;//TrueType����
			CMyGlyph m_VecGlyph;	//ʸ��������

			/*�㷨����*/
			static const int MaximaErrT = 2;//����ɾ�������㷨�õ��Ĺؼ���������ɷ���
			static const int AverErrT = 1;//����������ϵĲ���

		public:
            /*����ͼ������*/
			void LoadImg(std::string imgAdd);

            /*��������Ϊ��ֵͼ��*/
			void SetData(std::vector<std::vector<bool> > pixels);

            /*�Ӷ�ֵͼ����ʸ�����*/
			void GenerateContour();		

			/*����ʸ�������*/
			CStorageGlyph GetGlyphContour();		

	    private:
			/*��ֵ��ͼ��*/
			void BinarizeImage(cv::Mat &src, cv::Mat &dst, int flag);

			/*�Ӷ�ֵͼ�õ�����*/
			void TraceContour();

			/*ɾ�������㷨�ҳ��ؼ���*/
			void DeletionCost();
			double calculateDeleteCost(int st, int mi, int ed, int cn);
			double PointsDis(cv::Point p0, cv::Point p1, cv::Point p2);

            /*������ϵõ�TrueType����*/
			void CurveFitting();
			void FitCurveI(int cno, int pno);

			/*��ʸ�������д��ttGlyph��*/
			void SetGlyphContour();
	};
}