#include "catromcurveevaluator.h"
#include <iostream>

static float dist(Point p1, Point p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

void CatRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
	int numCtrlPoints = ptvCtrlPts.size();
	std::vector<Point> newCtrlPoints(ptvCtrlPts.begin(), ptvCtrlPts.end());
	ptvEvaluatedCurvePts.clear();

	// Insert extra control
    if(bWrap) 
    {
    	Point temp = Point(ptvCtrlPts[numCtrlPoints-1].x - fAniLength, ptvCtrlPts[numCtrlPoints-1].y);
    	newCtrlPoints.insert(newCtrlPoints.begin(), temp);
    	temp = Point(ptvCtrlPts[numCtrlPoints-2].x - fAniLength, ptvCtrlPts[numCtrlPoints-2].y);
    	newCtrlPoints.insert(newCtrlPoints.begin(), temp);

    	temp = Point(ptvCtrlPts[0].x + fAniLength, ptvCtrlPts[0].y);
        newCtrlPoints.push_back(temp);
        temp = Point(ptvCtrlPts[1].x + fAniLength, ptvCtrlPts[1].y);
		newCtrlPoints.push_back(temp);
    } 
    else 
    {
    	Point temp = Point(-2.0, ptvCtrlPts[0].y);
    	newCtrlPoints.insert(newCtrlPoints.begin(), temp);
    	temp = Point(-2.0, ptvCtrlPts[0].y);
    	newCtrlPoints.insert(newCtrlPoints.begin(), temp);

    	temp = Point(fAniLength+2, ptvCtrlPts[numCtrlPoints-1].y);
        newCtrlPoints.push_back(temp);
        temp = Point(fAniLength+2, ptvCtrlPts[numCtrlPoints-1].y);
        newCtrlPoints.push_back(temp);
    }
    int numNewCtrlPoints = newCtrlPoints.size();

    Vec4f xCoords, yCoords;
    
    Mat4f catRomBasis(-1, 3, -3, 1,
                       2, -5, 4, -1,
                      -1, 0, 1, 0,
                       0, 2, 0, 0);

    // for(int i = 2; i<3; i++)
    for(int i = 1; i < (numNewCtrlPoints-2); i++)
    {
        // float dist1 = dist(newCtrlPoints[i-1], newCtrlPoints[i]);
        // float dist2 = dist(newCtrlPoints[i], newCtrlPoints[i+1]);
        // float dist3 = dist(newCtrlPoints[i+1], newCtrlPoints[i+2]);
        // float totalDist = dist(newCtrlPoints[i-1], newCtrlPoints[i+2]);
        // if ((dist1 + dist2 + dist3)/totalDist < 1 + s_fFlatnessEpsilon)
        // {
        //     std::cout << "dsadsasa\n";
        //     ptvEvaluatedCurvePts.push_back(newCtrlPoints[i]);
        //     ptvEvaluatedCurvePts.push_back(newCtrlPoints[i+1]);
        //     continue;
        // }

        // if (abs(newCtrlPoints[i].x - newCtrlPoints[i+1].x) < 1)
        // {
        //     std::cout << "dsadsasa\n";
        //     ptvEvaluatedCurvePts.push_back(newCtrlPoints[i]);
        //     ptvEvaluatedCurvePts.push_back(newCtrlPoints[i+1]);
        //     continue;    
        // }
        xCoords = Vec4f(newCtrlPoints[i-1].x, newCtrlPoints[i].x, newCtrlPoints[i+1].x, newCtrlPoints[i+2].x);
        yCoords = Vec4f(newCtrlPoints[i-1].y, newCtrlPoints[i].y, newCtrlPoints[i+1].y, newCtrlPoints[i+2].y);
        
        xCoords = catRomBasis * xCoords * 0.5;
        yCoords = catRomBasis * yCoords * 0.5;
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
