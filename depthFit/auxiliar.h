#pragma once


#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>


#define UINT_TYPE 0
#define FOLAT_TYPE 1


#define RAW_DEPTH_TYPE unsigned short

bool FileExists(const char* strFilename);









template <class T>
bool getbin(FILE *fp,std::vector<T>&vec,int width,int height) {
	RAW_DEPTH_TYPE *buff = new RAW_DEPTH_TYPE[width * height];
	int ret = fread(buff, sizeof(RAW_DEPTH_TYPE), width * height, fp);
	if (ret != width * height) {
		return false;
	}
	vec.resize(width * height);
	for (int i = 0; i < width * height; i++) {
		vec[i] = static_cast<T>(buff[i]);
	}
	return true;
}


template <class T>
bool getBinFromefile(const std::string &path, std::vector<std::vector<T>>&depthVec,int width,int height,int frames) {
	FILE *fp = NULL;
	errno_t err_fp;
	err_fp = fopen_s(&fp,path.c_str(),"rb");
	if (err_fp != 0) {
		printf("ERROR:cannot open file :%s\n",path.c_str());
		return false;
	}
	depthVec.resize(frames);
	for (int i = 0; i < frames; i++) {
		int get_ret = getbin(fp,depthVec[i],width,height);
		if (get_ret == false) {
			printf("ERROR:read bin error in %d frame!\n",frames);
			return false;
		}
	}
	return true;
}


