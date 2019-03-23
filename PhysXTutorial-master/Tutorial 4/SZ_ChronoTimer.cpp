#include "SZ_ChronoTimer.h"



SZ_ChronoTimer::SZ_ChronoTimer()
{
}

void SZ_ChronoTimer::resetChronoTimer()
{
	startChrono = Clock::now();
}

float SZ_ChronoTimer::getChronoTime()
{
	std::chrono::steady_clock::time_point now = Clock::now();
	auto timeDiff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - startChrono).count();
	return (float)timeDiff;
}


SZ_ChronoTimer::~SZ_ChronoTimer()
{
}
