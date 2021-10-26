#include "hello.h"
#include <iostream>
#include "glog/logging.h"

void sayHello() 
{
	LOG(INFO)<<"Hello SLAM";
}

void GlogInit(char* name){
	//初始化GLOG
	google::InitGoogleLogging(name);
	//设置日志输出文件名
	google::SetLogDestination(google::GLOG_INFO, "log");
	//设置日志输出到命令行
	google::SetStderrLogging(google::GLOG_INFO);
}
void GlogUnInit()
{
	google::ShutdownGoogleLogging();
}
