#include "beziercurveevaluator.h"
#include <cassert>

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
					 std::vector<Point>& ptvEvaluatedCurvePts, 
					 const float& fAniLength, 
					 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();
	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());
	float wrapLength = ptvCtrlPts[0].x + (fAniLength - ptvCtrlPts.back().x);
	ptvEvaluatedCurvePts.clear();

	if (bWrap && (numCtrlPoints%3 != 0)) // Wrapping and not multiple of 3
	{
		Point temp  = Point(ptvCtrlPts[0].x, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(ptvCtrlPts[0].x - wrapLength, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(ptvCtrlPts[0].x - wrapLength, ptvCtrlPts[numCtrlPoints-1].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
	}

	if (!(bWrap && (numCtrlPoints%3 == 0))) // Not wrapping and not multiple of 3
	{
		Point temp  = Point(ptvCtrlPts[0].x, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
		temp  = Point(-4.0, ptvCtrlPts[0].y);
		newCtrlPoints.insert(newCtrlPoints.begin(), temp);
	}
	
	if (bWrap && (numCtrlPoints%3 == 0)) //Wrap and multiple of 3
	{
		Point temp = Point(ptvCtrlPts[numCtrlPoints-1].x + wrapLength, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
		temp = Point(ptvCtrlPts[numCtrlPoints-1].x + wrapLength, ptvCtrlPts[0].y);
		newCtrlPoints.push_back(temp);
	}
	Point temp = Point(ptvCtrlPts[numCtrlPoints-1].x, ptvCtrlPts[numCtrlPoints-1].y);
	newCtrlPoints.push_back(temp);
	temp = Point(ptvCtrlPts[numCtrlPoints-1].x, ptvCtrlPts[numCtrlPoints-1].y);
	newCtrlPoints.push_back(temp);

	temp = Point(fAniLength + 4, ptvCtrlPts[numCtrlPoints-1].y);
	newCtrlPoints.push_back(temp);
	temp = Point(fAniLength + 4, ptvCtrlPts[numCtrlPoints-1].y);
	newCtrlPoints.push_back(temp);

	Mat4f bezierBasis(-1, 3, -3, 1,
						3, -6, 3, 0,
						-3, 3, 0, 0,
						1, 0, 0, 0);

	Vec4f xCoords, yCoords;

	if (bWrap && (numCtrlPoints%3 == 0))
	{
		xCoords = Vec4f(ptvCtrlPts[numCtrlPoints-3].x - fAniLength, ptvCtrlPts[numCtrlPoints-2].x - fAniLength, ptvCtrlPts[numCtrlPoints-1].x - fAniLength, ptvCtrlPts[0].x);
		yCoords = Vec4f(ptvCtrlPts[numCtrlPoints-3].y, ptvCtrlPts[numCtrlPoints-2].y, ptvCtrlPts[numCtrlPoints-1].y, ptvCtrlPts[0].y);

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

	for (int i = 0; i < newCtrlPoints.size() - 7; i=i+3)
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

	if (!bWrap && (numCtrlPoints%3 == 0))
	{
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[numCtrlPoints - 2]);
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[numCtrlPoints - 1]);
		ptvEvaluatedCurvePts.push_back(Point(fAniLength + 4, ptvEvaluatedCurvePts.back().y));
	}

	if (numCtrlPoints%3 == 2)
	{
		ptvEvaluatedCurvePts.push_back(ptvCtrlPts[numCtrlPoints-1]);
		if (!bWrap)
		{
			ptvEvaluatedCurvePts.push_back(Point(fAniLength+4.0, ptvEvaluatedCurvePts.back().y));
		}
		else
		{
			ptvEvaluatedCurvePts.push_back(Point(ptvCtrlPts[numCtrlPoints-1].x + wrapLength, ptvCtrlPts[0].y));
		}
	}
	else if (numCtrlPoints%3 == 1)
	{
		if (!bWrap)
		{
			ptvEvaluatedCurvePts.push_back(Point(fAniLength+4.0, ptvEvaluatedCurvePts.back().y));
		}
		else
		{
			ptvEvaluatedCurvePts.push_back(Point(ptvCtrlPts[numCtrlPoints-1].x + wrapLength, ptvCtrlPts[0].y));
		}
	}
}