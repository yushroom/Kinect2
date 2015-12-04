#pragma once


#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <windows.h>

#define RAW_DEPTH_TYPE unsigned short

#define __DEBUG__
bool FileExists(const char* strFilename);

template <class T>
bool getbin(FILE *fp,std::vector<T>&vec,int width,int height) {
	RAW_DEPTH_TYPE *buff = new RAW_DEPTH_TYPE[width * height];
	int ret = (int)fread(buff, sizeof(RAW_DEPTH_TYPE), width * height, fp);
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
		bool get_ret = getbin(fp,depthVec[i],width,height);
		if (!get_ret) {
			printf("ERROR:read bin error in %d frame!\n",frames);
			return false;
		}
	}
	return true;
}



template <class T> 
bool MergeBinFromFile(const std::string &path,std::vector<std::vector<T>>&depthVec) {
	if (depthVec.empty()) return false;
	FILE *fp = NULL;
	errno_t err_fp;
	err_fp = fopen_s(&fp,path.c_str(),"wb");
	if (err_fp != 0) {
		printf("ERROR:cannot open file :%s\n", path.c_str());
		return false;
	}
	int ele_cnt = (int)depthVec[0].size();
	RAW_DEPTH_TYPE *depth = new RAW_DEPTH_TYPE[ele_cnt];
	for (int i = 0; i < depthVec.size(); i++) {
		for (int j = 0; j < depthVec[i].size(); j++) 
			depth[j] = static_cast<RAW_DEPTH_TYPE>(depthVec[i][j]);
		fwrite(depth,sizeof(RAW_DEPTH_TYPE),ele_cnt,fp);
	}
	delete[]depth;
	fclose(fp);
	return true;
}





bool FileExists(const char* strFilename) {
	struct stat stFileInfo;
	bool blnReturn;
	int intStat;

	// Attempt to get the file attributes 
	intStat = stat(strFilename, &stFileInfo);
	if (intStat == 0) {
		// We were able to get the file attributes 
		// so the file obviously exists. 
		blnReturn = true;
	}
	else {
		// We were not able to get the file attributes. 
		// This may mean that we don't have permission to 
		// access the folder which contains this file. If you 
		// need to do that level of checking, lookup the 
		// return values of stat which will give you 
		// more details on why stat failed. 
		blnReturn = false;
	}

	return(blnReturn);
}



namespace iniConfig{
	std::string iniFilePath;
	void setiniFile(const std::string &);
	int readIntValue(const char *section,const char *key,int defalutValue);
	float readFloatValue(const char *section,const char *key,float defalutValue);
	std::string readStringValue(const char *section,const char *key,const char *defalutValue);
};


int iniConfig::readIntValue(const char *section, const char *key, int defalutValue) {
	int ret = GetPrivateProfileInt(section,key,defalutValue,iniFilePath.c_str());
	return ret;
}



struct Parameters_T{
	int width, height, frames;
	std::string v1_source_path;
	std::string v2_source_path;
	std::string output_folder_path;

};

enum INI_TYPE  { type_int, type_str    };
enum SEC_TYPE  { sec_argu, sec_param   };

Parameters_T *readIniFile(const std::string &ini_path) {
	if (!FileExists(ini_path.c_str())) {
		printf("cannot find ini file :%s\n",ini_path.c_str());
		return NULL;
	}
	char ini_buff[128];

	Parameters_T * params = new Parameters_T();

	
	params->width  = GetPrivateProfileInt("argument","width", -1,ini_path.c_str());
	params->height = GetPrivateProfileInt("argument","height",-1,ini_path.c_str());
	if (params->width == -1 || params->height == -1) {
		printf("cannot read image size from %s ini file\n", ini_path.c_str());
		goto ERR;
	}
	params->frames = GetPrivateProfileInt("argument","frames",-1, ini_path.c_str());
	if (params->frames == -1) {
		printf("cannot read frames from %s ini file\n", ini_path.c_str());
		goto ERR;
	}
	GetPrivateProfileString("argument","v1_source_path","invaild",ini_buff,128,ini_path.c_str());
	if (strcmp(ini_buff, "invaild") == 0) {
		printf("cannot read v1_source_path from %s ini file\n", ini_path.c_str());
		goto ERR;
	} else {
		params->v1_source_path = std::string(ini_buff);
	}
	GetPrivateProfileString("argument","v2_source_path","invaild",ini_buff,128,ini_path.c_str());
	if (strcmp(ini_buff, "invaild") == 0) {
		printf("cannot read v2_source_path from %s ini file\n", ini_path.c_str());
		goto ERR;
	}
	else {
		params->v2_source_path = std::string(ini_buff);
	}
	GetPrivateProfileString("argument", "output_folder_path", "invaild", ini_buff, 128, ini_path.c_str());
	if (strcmp(ini_buff, "invaild") == 0) {
		printf("cannot read output_folder_path from %s ini file\n", ini_path.c_str());
		goto ERR;
	}
	else {
		params->output_folder_path = std::string(ini_buff);
	}

#ifdef __DEBUG__
	printf("read arugments from %s ini file\n",ini_path.c_str());
	printf("%20s: %20d\n", "width",  params->width);
	printf("%20s: %20d\n", "height", params->height);
	printf("%20s: %20d\n", "frames", params->frames);

	printf("%20s: %20s\n", "v1_source_path", params->v1_source_path.c_str());
	printf("%20s: %20s\n", "v2_source_path", params->v2_source_path.c_str());
	printf("%20s: %20s\n", "output_folder_path", params->output_folder_path.c_str());

#endif
	return params;
ERR:
	delete params;
	return NULL;
}

Parameters_T *get_arguments(int argc, char **argv) {
	if (argc != 7) {
		printf("usage: depthFit.exe $v1_sources_path &v2_sources_path $output_folder_path $width $height frames\n");
		exit(-1);
	}
	Parameters_T *params = new Parameters_T();
	params->v1_source_path = std::string(argv[1]);
	params->v2_source_path = std::string(argv[2]);
	params->output_folder_path = std::string(argv[3]);
	params->width = atoi(argv[4]);
	params->height = atoi(argv[5]);
	params->frames = atoi(argv[6]);
	return params;
}