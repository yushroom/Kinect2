#include <gaussian_utils.h>
#include <dump_utils.h>
#include <string>

const std::string path_pre = R"(D:\cvpr\synthetic_depth_image\)";

int main()
{
	std::vector<unsigned short> depth_v1, depth_v2;
    std::vector<unsigned short> depth_v1_true, depth_v2_true, depth_v2_shift_true;
	const int width = 640, height = 480;
	const float bias_v1 = 0.f, bias_v2 = 50.f / 640;
	const unsigned short base = 500;
	const float sigma_v1 = 10, sigma_v2 = 5;
	const int gaussian_width1 = 7, gaussian_width2 = 3;
	const float gaussian_sigma1 = 3.f, gaussian_sigma2 = 1.2f;
	RNG rng;
	//auto func = [](int x, int y) -> unsigned short { return (unsigned short)2; };
	const int border = 100;
	std::function<unsigned short(int, int)> func1 = [width, height, base, border](int x, int y) -> unsigned short { 
		if (x < border || width - x < border || y < border || height - y < border) {
			return base + 100;
		}
		return base;
	};

    std::function<unsigned short(int, int)> func2 = [width, height, base, border, bias_v2](int x, int y) -> unsigned short {
		//return 500;
		if (x < border || width - x < border || y < border || height - y < border) {
			return base + 100;
		}
		return base + (unsigned short)(x * 50.f / width);
	};

	const int shift_x = 2;
	std::function<unsigned short(int, int)> func2_shift = [width, height, base, border, shift_x, bias_v2](int x, int y) -> unsigned short {
		//return 500;
		x += shift_x;
		y += shift_x;
		if (x < border || width - x < border || y < border || height - y < border) {
			return base + 100;
		}
		return base + (unsigned short)(x * 50.f / width);
	};

	gen_gaussian_image(
		depth_v1,
		rng,
		sigma_v1,
		gaussian_width1,
		gaussian_sigma1,
		width,
		height,
		func1
	);
	dump_normalized_image(depth_v1, (path_pre + "synthetic_depth_v1.bmp").c_str(), width, height);
	dump_raw_vector(depth_v1, path_pre+"synthetic_depth_v1.bin");

    depth_v1_true.resize(width*height);
    depth_v2_true.resize(width*height);
    depth_v2_shift_true.resize(width*height);
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            int idx = j * width + i;
            depth_v1_true[idx] = func1(i, j);
            depth_v2_true[idx] = func2(i, j);
            depth_v2_shift_true[idx] = func2_shift(i, j);
        }
    }
    dump_normalized_image(depth_v1_true, (path_pre + "synthetic_depth_v1_true.bmp").c_str(), width, height);
	dump_raw_vector(depth_v1_true, path_pre+"synthetic_depth_v1_true.bin");
    dump_normalized_image(depth_v2_true, (path_pre + "synthetic_depth_v2_true.bmp").c_str(), width, height);
	dump_raw_vector(depth_v2_true, path_pre+"synthetic_depth_v2_true.bin");
    dump_normalized_image(depth_v2_shift_true, (path_pre + "synthetic_depth_v2_shift_true.bmp").c_str(), width, height);
	dump_raw_vector(depth_v2_shift_true, path_pre+"synthetic_depth_v2_shift_true.bin");

	gen_gaussian_image(
		depth_v2,
		rng,
		sigma_v2,
		gaussian_width2,
		gaussian_sigma2,
		width,
		height,
		func2
	);
	dump_raw_vector(depth_v2, path_pre+"synthetic_depth_v2.bin");
	dump_normalized_image(depth_v2, (path_pre+"synthetic_depth_v2.bmp").c_str(), width, height);

    gen_gaussian_image(
		depth_v2,
		rng,
		sigma_v2,
		gaussian_width2,
		gaussian_sigma2,
		width,
		height,
		func2_shift
	);
	dump_raw_vector(depth_v2, path_pre+"synthetic_depth_v2_shift.bin");
	dump_normalized_image(depth_v2, (path_pre+"synthetic_depth_v2_shift.bmp").c_str(), width, height);

	system("Pause");
	return 0;
}