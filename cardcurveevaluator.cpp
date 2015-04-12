#include "cardcurveevaluator.h"
#include "animatoruiwindows.h"
#include <cassert>

void CardCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
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
    double m_ntau = ModelerUIWindows::m_nTAU;
    Mat4f cardBasis(-1, 2/m_ntau - 1, -2/m_ntau + 1, 1,
                    2, -3/m_ntau + 1, 3/m_ntau - 2, -1,
                    -1, 0, 1, 0,
                    0, 1/m_ntau, 0, 0);
    
    Vec4f xCoords, yCoords;

    for(int i = 1; i < (numNewCtrlPoints-2); i++) 
    {         
        xCoords = Vec4f(newCtrlPoints[i-1].x, newCtrlPoints[i].x, newCtrlPoints[i+1].x, newCtrlPoints[i+2].x);
        yCoords = Vec4f(newCtrlPoints[i-1].y, newCtrlPoints[i].y, newCtrlPoints[i+1].y, newCtrlPoints[i+2].y);
        
        xCoords = cardBasis * xCoords * m_ntau;
        yCoords = cardBasis * yCoords * m_ntau;
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
