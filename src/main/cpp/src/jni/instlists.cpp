#include "jni/instlists.h"

// This is a list of all the existing instances of objects (Java side)
// With each instance created an entry will be added
// With each instance destroyed an entry will be deleted
// This ensures that when there are no more Java instances of an object, the C++ object is also deleted
std::list<std::shared_ptr<rpf::Path>> pinstances;
std::list<std::shared_ptr<rpf::BasicTrajectory>> btinstances;
std::list<std::shared_ptr<rpf::TankDriveTrajectory>> ttinstances;

std::mutex pinstances_mutex;
std::mutex btinstances_mutex;
std::mutex ttinstances_mutex;
