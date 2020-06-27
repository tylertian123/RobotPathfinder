#include "trajectories.h"
#include <list>
#include <memory>
#include <mutex>

extern std::list<std::shared_ptr<rpf::Path>> pinstances;
extern std::list<std::shared_ptr<rpf::BasicTrajectory>> btinstances;
extern std::list<std::shared_ptr<rpf::TankDriveTrajectory>> ttinstances;

extern std::mutex pinstances_mutex;
extern std::mutex btinstances_mutex;
extern std::mutex ttinstances_mutex;
