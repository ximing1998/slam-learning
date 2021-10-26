#include "hello.h"
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <string.h>
#include <iostream>
using namespace std;

DEFINE_int64(print_times,1,"print times");

void GlogInit(char* name);
void GlogUnInit();

int main( int argc, char** argv ) {
    //GFLAGS
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	//GTEST
	EXPECT_GE(FLAGS_print_times, 0);
	//GLOG
	//将所有日志输出到文件和stderr(终端)
	//FLAGS_alsologtostderr = 1;
    GlogInit(argv[0]);
	for(int i=0; i<FLAGS_print_times; i++)
		sayHello();
    GlogUnInit();

    return 0;
}
