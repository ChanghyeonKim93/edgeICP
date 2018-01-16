#include "timer.h"

auto ti_ = std::chrono::high_resolution_clock::now(); // initial
auto t1_ = ti_;

unsigned long tic(){
	auto tf_ = std::chrono::high_resolution_clock::now();
	auto ticks_ = std::chrono::duration_cast<std::chrono::microseconds>(tf_ - ti_);

	return (unsigned long)ticks_.count();
}

unsigned long toc(){
	auto tf_ = std::chrono::high_resolution_clock::now();
	auto ticks_ = std::chrono::duration_cast<std::chrono::microseconds>(tf_ - t1_);
	t1_ = tf_;

	return (unsigned long)ticks_.count();
}
