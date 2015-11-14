#include "KinectNoiseMode.h"

enum KinectType {
	KinectV1,
	KinectV2
};

void process(string depth_bin_path, string IR_path, KinectType type)
{
	KinectNoiseMode* noise_mode;
	if (type == KinectV1)
		noise_mode = new KinectNoiseMode1(depth_bin_path, IR_path);
	else
		noise_mode = new KinectNoiseMode2(depth_bin_path, IR_path);

	float lateral_noise = 0, axial_noise = 0;
	noise_mode->process();
	delete noise_mode;
}

int main()
{
	string root_dir = "D:\\yyk\\image\\NoiseModel_1114\\";
	//string number_ID = "1447405332-2";
	//string prefix = root_dir + number_ID;
	//process(prefix + "-a.bin", prefix + "-a-IR.bmp", KinectV1);
	//waitKey();
	//process(prefix + "-b.bin", prefix + "-b-IR.bmp", KinectV2);
	//waitKey();
	//return 0;

	//string path_file(PATH_FILE);
	ifstream fin(root_dir+"path.txt");
	string str;
	vector<string> image_path_list;
	while (fin >> str) {
		image_path_list.push_back(str);
	}
	while (image_path_list.size() % 4 != 0) {
		image_path_list.pop_back();
	}
	cout << "image path list: " << image_path_list.size() << endl;

	bool save_temp_file = true;

	//int count = 100;

	for (int i = 0; i < image_path_list.size() / 4; ++i) {
		const string& IR1_path		= image_path_list[i * 4];
		const string& depth1_path	= image_path_list[i * 4 + 1];
		const string& IR2_path		= image_path_list[i * 4 + 2];
		const string& depth2_path	= image_path_list[i * 4 + 3];
		process(depth1_path, IR1_path, KinectV1);
		waitKey();
		process(depth2_path, IR2_path, KinectV2);
		waitKey();
	}
	return 0;
}