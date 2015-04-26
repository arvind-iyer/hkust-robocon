#include "omp.h"
#include <stdio.h>
#include <iostream>
#include <iostream>
#include <chrono>

class Timer
{
public:
	Timer() : beg_(clock_::now()) {}
	void reset() { beg_ = clock_::now(); }
	double elapsed() const {
		return std::chrono::duration_cast<second_>
			(clock_::now() - beg_).count();
	}

private:
	typedef std::chrono::high_resolution_clock clock_;
	typedef std::chrono::duration<double, std::ratio<1> > second_;
	std::chrono::time_point<clock_> beg_;
};

using namespace std;
int mainojfod()
{
	omp_set_num_threads(4);
	Timer t;
	
	#pragma omp parallel 
	{
		
		int ID = omp_get_thread_num();
		
		for (int i = 0; i < 50000; i++)
		cout << "\b";
		printf("Hello %d\n\n", ID);
	}
	cout << t.elapsed() << endl;
	t.reset();
	
	for (int i = 0; i < 4 * 50000; i++)
		cout << "\b";
	printf("Hello \n\n");
	cout << t.elapsed() << endl;
	
	while (1);
}