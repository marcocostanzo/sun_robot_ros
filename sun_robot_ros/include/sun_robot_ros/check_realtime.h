#ifndef SUN_ROBOT_ROS_CHECK_REALTIME
#define SUN_ROBOT_ROS_CHECK_REALTIME

#include <fstream>  // std::ifstream
#include <iostream> // std::cout
#include <pthread.h>
#include <sched.h>

namespace sun {

bool check_realtime(){

  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }

  return has_realtime;

}

bool set_realtime_SCHED_FIFO(int priority_to_max = 0) {

  bool ok = false;

  bool has_realtime = check_realtime();

  if (has_realtime) {
    const int thread_priority = sched_get_priority_max(SCHED_FIFO) - priority_to_max;

    if (thread_priority != -1) {
      // We'll operate on the currently running thread.
      pthread_t this_thread = pthread_self();

      // struct sched_param is used to store the scheduling priority
      struct sched_param params;

      // We'll set the priority to the maximum.
      params.sched_priority = thread_priority;

      int ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);
      if (ret != 0) {
        std::cerr << "Unsuccessful in setting main thread realtime "
                     "priority. Error code: "
                  << ret << std::endl;
      }
      // Now verify the change in thread priority
      int policy = 0;
      ret = pthread_getschedparam(this_thread, &policy, &params);
      if (ret != 0) {
        std::cerr << "Couldn't retrieve real-time scheduling paramers"
                  << std::endl;
      }

      // Check the correct policy was applied
      if (policy != SCHED_FIFO) {
        std::cerr << "Main thread: Scheduling is NOT SCHED_FIFO!" << std::endl;
      } else {
        std::cout << "Main thread: SCHED_FIFO OK" << std::endl;
        ok = true;
      }

      // Print thread scheduling priority
      std::cout << "Main thread priority is " << params.sched_priority
                << std::endl;
    } else {
      std::cerr << "Could not get maximum thread priority for main thread"
                << std::endl;
    }
  } else {
    std::cout << "NO REALTIME CAPABILITIES" << std::endl;
  }
  return ok;
}

} // namespace sun

#endif
