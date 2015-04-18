#include "cinvcurveevaluator.h"
#include <cassert>

void CInvBezCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
					 std::vector<Point>& ptvEvaluatedCurvePts, 
					 const float& fAniLength, 
					 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();
	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());
	float wrapLength = ptvCtrlPts[0].x + (fAniLength - ptvCtrlPts.back().x);
	ptvEvaluatedCurvePts.clear();
	int pointsNeeded = 4 - ((numCtrlPoints) % 4);
	pointsNeeded = pointsNeeded == 4? 0 : pointsNeeded;

	if (bWrap)
	{
		float y1;
		if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[numCtrlPoints - 1].x > 0.0f) {
			y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[numCtrlPoints - 1].x) + 
				  ptvCtrlPts[numCtrlPoints - 1].y * ptvCtrlPts[0].x) /
				 (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[numCtrlPoints - 1].x);
		}
		else 
			y1 = ptvCtrlPts[0].y;
		Point temp = Point(-0.1, y1);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp = Point(fAniLength + 0.1, y1);
		newCtrlPoints.push_back(temp);
	}
	else
	{
		Point temp = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp = Point(fAniLength + 4.0, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);

		// for (int i = 0; i < pointsNeeded; i++)
		// {
		// 	temp = Point(fAniLength + 5.0 + i, ptvCtrlPts[numCtrlPoints-1].y);
		// 	newCtrlPoints.push_back(temp);
		// }
	}
	if (newCtrlPoints.size() < 6)
	{
		ptvEvaluatedCurvePts.assign(newCtrlPoints.begin(), newCtrlPoints.end());
		return;
	}
	Mat4f bezierBasis(-1, 3, -3, 1,
						3, -6, 3, 0,
						-3, 3, 0, 0,
						1, 0, 0, 0);

	Vec4f xCoords, yCoords;
	ptvEvaluatedCurvePts.push_back(newCtrlPoints[0]);
	int i = 0;
	for (i = 1; (i+3) <= newCtrlPoints.size() - 2; i = i+4)
	{
		xCoords = Vec4f(newCtrlPoints[i].x, newCtrlPoints[i+1].x, newCtrlPoints[i+2].x, newCtrlPoints[i+3].x);
		yCoords = Vec4f(newCtrlPoints[i].y, newCtrlPoints[i+1].y, newCtrlPoints[i+2].y, newCtrlPoints[i+3].y);

		xCoords = bezierBasis * xCoords;
		yCoords = bezierBasis * yCoords;

		for (int j = 0; j <= CurveEvaluator::STEPS ; j++)
		{
			float u;
			u = ((float)j)/(CurveEvaluator::STEPS);
			Point p = Point(xCoords[3] + u * xCoords[2] + u*u * xCoords[1] + u*u*u * xCoords[0], 
							yCoords[3] + u * yCoords[2] + u*u * yCoords[1] + u*u*u * yCoords[0]);
			ptvEvaluatedCurvePts.push_back(p);
		}
	}
	for (int j = i; j < newCtrlPoints.size(); j++)
	{
		ptvEvaluatedCurvePts.push_back(newCtrlPoints[j]);
	}
	// ptvEvaluatedCurvePts.push_back(newCtrlPoints[newCtrlPoints.size() - 1]);
}