#include "algorithm_usage.h"


void build_traj(){
	std::cout << "Hello World!\n";
}

void write_traj() {
	std::vector<std::pair<double, double>> traj;
	traj.push_back({ 0.1, 0.1 });
	for (int i = 1; i < 10; ++i) {
		std::pair<double, double> par = { traj[i - 1].first + 30, traj[i - 1].second + 30 };
		traj.push_back(par);
	}
	//std::string filename = "./cpp_version_dlls/cpptraj.txt";
	std::string filename = "./cpptraj.txt";

	std::ofstream out;          // поток для записи
	out.open(filename);      // открываем файл для записи
	if (out.is_open())
	{
		for (auto el : traj) {
			out << el.first << " " << el.second << std::endl;
		}
	}

}