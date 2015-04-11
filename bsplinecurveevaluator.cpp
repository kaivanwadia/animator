#include "bsplinecurveevaluator.h"
#include <cassert>

void BSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();

	// At least 2 control points needed
	ptvEvaluatedCurvePts.clear();

	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());
	// Wrapped or not wrapped check
	if (bWrap)
	{
		// Insert points at the beginning of the curve
		Point temp  = Point(ptvCtrlPts[numCtrlPoints-1].x - fAniLength, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(ptvCtrlPts[numCtrlPoints-2].x - fAniLength, ptvCtrlPts[numCtrlPoints-2].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);

		// Insert points at the end of the curve
		temp  = Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
		temp  = Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y);
		newCtrlPoints.push_back(temp);
	}
	else
	{
		// Insert points at the beginning of the curve
		Point temp  = Point(ptvCtrlPts[0].x, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(ptvCtrlPts[0].x, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);

		temp  = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);

		// Insert points at the end of the curve
		temp  = Point(ptvCtrlPts[numCtrlPoints-1].x, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
		temp  = Point(ptvCtrlPts[numCtrlPoints-1].x, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);

		temp  = Point(fAniLength + 4, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
		temp  = Point(fAniLength + 4, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.push_back(temp);
	}

	int numNewCtrlPoints = newCtrlPoints.size();

	Mat4f bSplineBasis(-1, 3, -3, 1,
						3, -6, 3, 0,
						-3, 0, 3, 0,
						1, 4, 1, 0);

	Vec4f xCoords, yCoords;
	for (int i = 0; i < (numNewCtrlPoints - 3); i++)
	{
		xCoords = Vec4f(newCtrlPoints[i].x, newCtrlPoints[i+1].x, newCtrlPoints[i+2].x, newCtrlPoints[i+3].x);
		yCoords = Vec4f(newCtrlPoints[i].y, newCtrlPoints[i+1].y, newCtrlPoints[i+2].y, newCtrlPoints[i+3].y);

		xCoords = bSplineBasis * xCoords / 6.0;
		yCoords = bSplineBasis * yCoords / 6.0;

		for (int j = 0; j < CurveEvaluator::STEPS ; j++)
		{
			float u;
			u = ((float)j)/(CurveEvaluator::STEPS);
			Point p = Point(xCoords[3] + u * xCoords[2] + u*u * xCoords[1] + u*u*u * xCoords[0], 
							yCoords[3] + u * yCoords[2] + u*u * yCoords[1] + u*u*u * yCoords[0]);
			ptvEvaluatedCurvePts.push_back(p);
		}
	}
}
