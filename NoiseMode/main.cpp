#include "KinectNoiseModel.h"

enum KinectType {
	KinectV1,
	KinectV2
};

struct NoiseModel {
	float distance;
	float angle;
	float lateral_noise;
	float axial_noise;

	NoiseModel() : distance(0), angle(0), lateral_noise(0), axial_noise(0) {

	}

	NoiseModel(float d, float a, float ln, float an) 
		: distance(d), angle(a), lateral_noise(ln), axial_noise(an) {
	}

	NoiseModel& operator+=(const NoiseModel& nm) {
		distance += nm.distance; angle += nm.angle;
		lateral_noise += nm.lateral_noise; axial_noise += nm.axial_noise;
		return *this;
	}

	NoiseModel& operator/=(const float f) {
		assert(f > 0);
		distance /= f; angle /= f; lateral_noise /= f; axial_noise /= f;
		return *this;
	}

	friend ostream& operator<<(ostream& os, const NoiseModel& nm) {
		os << nm.distance << ", " << nm.angle << ", " << nm.lateral_noise << ", " << nm.axial_noise;
		return os;
	}
};

vector<NoiseModel> noise_model_data_v1;
vector<NoiseModel> noise_model_data_v2;

void process(string depth_bin_path, string IR_path, KinectType type)
{
	KinectNoiseModel* noise_model;
	if (type == KinectV1) {
		noise_model = new KinectNoiseModel1(depth_bin_path, IR_path);
		if (noise_model->process()) {
			noise_model_data_v1.push_back(NoiseModel(noise_model->m_distance, noise_model->m_angle, 
				noise_model->m_lateral_noise, noise_model->m_axial_noise));
		}
	}
	else {
		noise_model = new KinectNoiseModel2(depth_bin_path, IR_path);
		if (noise_model->process()) {
			noise_model_data_v2.push_back(NoiseModel(noise_model->m_distance, noise_model->m_angle, 
				noise_model->m_lateral_noise, noise_model->m_axial_noise));
		}
	}

	delete noise_model;
	cout << endl;
}

void process_and_write_data()
{

#if 1
	{
		ofstream fout("D://noise_model_V1_1119_a_all.csv");
		fout << "distance, angle, lateral noise, axial noise\n";
		for (auto& n : noise_model_data_v1) {
			//fout << n.distance << ", " << n.angle << ", " 
			//		<< n.lateral_noise << ", " << n.axial_noise << '\n';
			fout << n << '\n';
		}
		fout.close();
	}
	{
		ofstream fout("D://noise_model_V2_1119_b_all.csv");
		fout << "distance, angle, lateral noise, axial noise\n";
		for (auto& n : noise_model_data_v2) {
			//fout << n.distance << ", " << n.angle << ", " 
			//		<< n.lateral_noise << ", " << n.axial_noise << '\n';
			fout << n << '\n';
		}
		fout.close();
	}
#endif
#if 0
	ofstream fout("D://noise_model_V1_1116.csv");
	fout << "distance, angle, lateral noise, axial noise\n";
	//vector<NoiseModel> average_noise_model;
	NoiseModel average;
	int count = 0;
	for (auto& n : noise_model_data) {
		if (count > 0 && fabsf(n.angle - average.angle / count) > 1) { // new class			
			cout << "new group: ";
			average /= count;
			//average_noise_model.push_back(average);
			fout << average << '\n';
			average = n;
			count = 1;
		}
		else {
			average += n;
			count ++;
		}
	}
	average /= count;
	fout << average << '\n';

	fout.close();
#endif
}

int main()
{
	log_system_init();

	string root_dir = "D:\\yyk\\image\\NoiseModel_1119\\";
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

	for (int i = 0; i < image_path_list.size() / 4; ++i) {
		const string& IR1_path		= image_path_list[i * 4];
		const string& depth1_path	= image_path_list[i * 4 + 1];
		const string& IR2_path		= image_path_list[i * 4 + 2];
		const string& depth2_path	= image_path_list[i * 4 + 3];
		process(depth1_path, IR1_path, KinectV1);
		//waitKey();
		process(depth2_path, IR2_path, KinectV2);
		//waitKey();
	}

	process_and_write_data();

	return 0;
}