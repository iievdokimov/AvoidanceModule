#include "trajectory_estimation.h"

TrajectoryEstimator::TrajectoryEstimator(std::vector<ModelState> new_traj, FinishLog new_log, std::vector<ModelState> cur_traj, FinishLog cur_log):
	_new_trajectory{ new_traj }, _log_new_trajectory{ new_log }, _cur_trajectory{ cur_traj }, _log_cur_trajectory{cur_log}
{
	set_result();
}

const std::vector<ModelState>& TrajectoryEstimator::proposed_trajectory() const
{
	// if est_rating_new > est_rating_cur
	if (_result == EstimationResult::must_switch) {
		return _new_trajectory;
	}
	else if (_result == EstimationResult::unneeded_switch) {
		return _cur_trajectory;
	}
}

void TrajectoryEstimator::set_result()
{
	// заглушка
	// TODO: качественный критерий
	if (not _log_new_trajectory.collision_happened() and _log_cur_trajectory.collision_happened()) {
		_result = EstimationResult::must_switch;
	}
	else {
		_result = EstimationResult::unneeded_switch;
	}
}

/*
	TrajectoryEstimator(TrajectoryEstimator&& other) noexcept
		: _log_new_trajectory{ std::move(other._log_new_trajectory) },
		_log_cur_trajectory{ std::move(other._log_cur_trajectory) }

	{
		_new_trajectory = std::move(other._new_trajectory);
		_cur_trajectory = std::move(other._cur_trajectory);
		_result = std::move(other._result);
	}

	TrajectoryEstimator& operator=(const TrajectoryEstimator& other) {
		if (this != &other) {
			_log_new_trajectory = other._log_new_trajectory;
			_log_cur_trajectory = other._log_cur_trajectory;
			_new_trajectory = other._new_trajectory;
			_cur_trajectory = other._cur_trajectory;
			_result = other._result;
		}
		return *this;
	}
	*/