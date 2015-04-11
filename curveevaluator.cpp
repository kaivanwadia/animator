#include "curveevaluator.h"

float CurveEvaluator::s_fFlatnessEpsilon = 0.00001f;
int CurveEvaluator::s_iSegCount = 16;
float CurveEvaluator::STEPS = 25.0;

CurveEvaluator::~CurveEvaluator(void)
{
}
