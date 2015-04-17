#include "c2curveevaluator.h"
#include <cassert>

void C2InterCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
					 std::vector<Point>& ptvEvaluatedCurvePts, 
					 const float& fAniLength, 
					 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();
	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());
	float wrapLength = ptvCtrlPts[0].x + (fAniLength - ptvCtrlPts.back().x);
	ptvEvaluatedCurvePts.clear();

	if (bWrap)
	{
		Point temp = Point(ptvCtrlPts[0].x - wrapLength, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp = Point(fAniLength + ptvCtrlPts[0].x, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
		temp = Point(fAniLength + ptvCtrlPts[0].x + 1, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
		temp = Point(fAniLength + ptvCtrlPts[0].x + 2, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
	}
	else
	{
		Point temp = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp = Point(fAniLength + 4.0, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
		temp = Point(fAniLength + 5.0, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
		temp = Point(fAniLength + 6.0, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
	}

	Mat4f bezierBasis(-1, 3, -3, 1,
						3, -6, 3, 0,
						-3, 3, 0, 0,
						1, 0, 0, 0);
	Mat4f c2Inter(26, -7, 2, -1,
					-7, 14, -4, 2,
					2, -4, 14, -7,
					-1, 2, -7, 26);

	Vec4f xCoords, yCoords;
	for (int i = 0; i < newCtrlPoints.size() - 3; i++)
	{
		xCoords = Vec4f(newCtrlPoints[i+1].x - newCtrlPoints[i].x, newCtrlPoints[i+2].x - newCtrlPoints[i].x, newCtrlPoints[i+3].x - newCtrlPoints[i+1].x, newCtrlPoints[i+3].x - newCtrlPoints[i+2].x);
		yCoords = Vec4f(newCtrlPoints[i+1].y - newCtrlPoints[i].y, newCtrlPoints[i+2].y - newCtrlPoints[i].y, newCtrlPoints[i+3].y - newCtrlPoints[i+1].y, newCtrlPoints[i+3].y - newCtrlPoints[i+2].y);
		xCoords = xCoords*3.0;
		yCoords = yCoords*3.0;

		Vec4f xDs, yDs;
		xDs = c2Inter * xCoords / 45.0;
		yDs = c2Inter * yCoords / 45.0;

		Vec4f xInters, yInters;

		xInters = Vec4f(newCtrlPoints[i].x, newCtrlPoints[i].x + 1/3.0 * xDs[0], newCtrlPoints[i+1].x - 1/3.0 * xDs[1], newCtrlPoints[i+1].x);
		yInters = Vec4f(newCtrlPoints[i].y, newCtrlPoints[i].y + 1/3.0 * yDs[0], newCtrlPoints[i+1].y - 1/3.0 * yDs[1], newCtrlPoints[i+1].y);

		Vec4f xBez, yBez;

		xBez = bezierBasis * xInters;
		yBez = bezierBasis * yInters;

		for (int j = 0; j <= CurveEvaluator::STEPS ; j++)
		{
			float u;
			u = ((float)j)/(CurveEvaluator::STEPS);
			Point p = Point(xBez[3] + u * xBez[2] + u*u * xBez[1] + u*u*u * xBez[0], 
							yBez[3] + u * yBez[2] + u*u * yBez[1] + u*u*u * yBez[0]);
			ptvEvaluatedCurvePts.push_back(p);
		}
	}
}