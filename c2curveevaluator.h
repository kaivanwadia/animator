#ifndef INCLUDED_C2INTERPOLATING_CURVE_EVALUATOR_H
#define INCLUDED_C2INTERPOLATING_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  

#include "curveevaluator.h"

class C2InterCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
};

#endif
