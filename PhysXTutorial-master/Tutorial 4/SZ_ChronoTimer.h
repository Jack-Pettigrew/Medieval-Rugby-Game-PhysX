#pragma once
#ifndef aHighResTimerFILE
#define aHighResTimerFILE
#include <iostream>
#include <chrono>

// Written by Olivier Szymanezyk oszymanezyk@lincoln.ac.uk
class SZ_ChronoTimer
{
private:
	typedef std::chrono::high_resolution_clock Clock;
	std::chrono::steady_clock::time_point startChrono;

public:
	SZ_ChronoTimer();
	void resetChronoTimer();
	float getChronoTime();
	~SZ_ChronoTimer();

};
#endif