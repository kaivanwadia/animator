#include "qbsplinecurveevaluator.h"
#include <cassert>

void QBSplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();

	// At least 2 control points needed
	ptvEvaluatedCurvePts.clear();

	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());

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

	Mat3f qBSplineBasis(1, -2, 1,
                        -2, 2, 0,
                        1, 1, 0);

	Vec3f xCoords, yCoords;

	for (int i = 0; i < (numNewCtrlPoints - 2); i++)
	{
		xCoords = Vec3f(newCtrlPoints[i].x, newCtrlPoints[i+1].x, newCtrlPoints[i+2].x);
		yCoords = Vec3f(newCtrlPoints[i].y, newCtrlPoints[i+1].y, newCtrlPoints[i+2].y);

		xCoords = qBSplineBasis * xCoords / 2.0;
		yCoords = qBSplineBasis * yCoords / 2.0;

		for (int j = 0; j < CurveEvaluator::STEPS ; j++)
		{
			float u;
			u = ((float)j)/(CurveEvaluator::STEPS);

			Point p = Point(xCoords[2] + u * xCoords[1] + u*u * xCoords[0],
							yCoords[2] + u * yCoords[1] + u*u * yCoords[0]);
			ptvEvaluatedCurvePts.push_back(p);
		}
	}
}
