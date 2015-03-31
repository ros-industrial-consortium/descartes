#include "descartes_core/trajectory_id.h"

using namespace descartes_core;
boost::atomic<uint64_t> IdGenerator<uint64_t>::counter_ (1);

