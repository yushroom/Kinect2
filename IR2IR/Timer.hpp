#pragma once
#include "RenderFish.hpp"
#include <chrono>

class Timer
{
private:
	typedef std::chrono::high_resolution_clock::time_point time_point;
	//typedef std::chrono::time_point<std::chrono::steady_clock> time_point;
	time_point _start;
	time_point _end;
	string _label;
	bool _has_started;

public:
	Timer(string label) : _label(label), _has_started(false) {}

	void set_label(string label) {
		_label = label;
	}

	void reset() {
		_has_started = false;
	}

	void begin() {
		_start = std::chrono::high_resolution_clock::now();
		_has_started = true;
	}
	void end() {
		if (!_has_started) {
			error("timer %s not started.", _label);
		}
		_end = std::chrono::high_resolution_clock::now();
		//_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _start).count();
	}

	void print() {
		//info("Time info for [%s]: ", _label.c_str());
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _start).count();
		if (ms >= 1000)
			info("Time info for [%s]: %f s\n", _label.c_str(), float(ms) / 1000);
		else
			info("Time info for [%s]: %lld ms\n", _label.c_str(), ms);
	}
	//friend std::ostream& operator<<(std::ostream& os, const Timer& timer) {
	//	log("Time info for [%s]:\n", timer.label);
	//	os << "Time info for [" << label << "]:"
	//}
};

