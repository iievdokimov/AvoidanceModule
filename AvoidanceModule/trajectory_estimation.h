#ifndef TRAJ_EST
#define TRAJ_EST

#include <vector>

#include "logger.h"
#include "models.h"


enum EstimationResult {
	must_switch = 0,
	unneeded_switch,
	consider_switch,
	empty,
};


class TrajectoryEstimator {
public:
	//TrajectoryEstimator() : _result{ EstimationResult::empty } {}
	TrajectoryEstimator(std::vector<ModelState> new_traj, FinishLog new_log,
		std::vector<ModelState> cur_traj, FinishLog cur_log);


	EstimationResult result() const { return _result;  }
	const std::vector<ModelState>& proposed_trajectory() const;
	const std::vector<ModelState>& new_trajectory() const { return _new_trajectory; }

	const auto& cur_traj_log() const { return _log_cur_trajectory; }
	const auto& new_traj_log() const { return _log_new_trajectory; }
	
	//TrajectoryEstimator(TrajectoryEstimator&& other) noexcept
	//TrajectoryEstimator& operator=(const TrajectoryEstimator& other)
private:
	void set_result();

	FinishLog _log_new_trajectory;
	FinishLog _log_cur_trajectory;

	std::vector<ModelState> _new_trajectory;
	std::vector<ModelState> _cur_trajectory;

	EstimationResult _result;
};




#endif TRAJ_EST
