// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

//** Customized header for handling MuJoCo data */
//! Custom 하면 파일 추가
#include "Useful.hpp"
#include "Mclquad.hpp"
#include "MotionTrajectory.hpp"
#include "MuJoCoInterface.hpp"
#include "MotorController.hpp"
#include "data_logging.hpp"
#include "estimate.hpp"


#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

//* ******************************************************************************************** *//
//* ******************************** MY CONSTANTS & OBJECT DECLARATION ************************* *//
//* ******************************************************************************************** *//

const double sim_end_time = 45.0;       // simulation end time (seconds)
unsigned int loop_iter = 0;             // loop iteration counter
RobotLeg<float> robot = buildMclQuad<float>();  // robot model
MotorController<float> motor_ctrl(robot);     // tracking controller
MotionTrajectory<float> traj_generator;            // motion trajectory
DataLogging<float> data_logger(robot);         // data logger
Estimate<float> estimator(robot);              // state estimator

std::shared_ptr<MuJoCoInterface<float>::MuJoCoActuatorCommand> actuator_cmd_ptr;
std::shared_ptr<MotionTrajectory<float>::DesiredFootTrajectory> foot_traj_ptr;
std::shared_ptr<MotionTrajectory<float>::DesiredJointTrajectory> joint_traj_ptr;
std::shared_ptr<Estimate<float>::EstimateParam> estimate_param_ptr;


//* ******************************************************************************************** *//



// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;


//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

//* ******************************************************************************************** *//
//* ******************************** Custom Function  ****************************************** *//
void Motor_ID_Setting(mjData * d)
{
  /**
   * !주의사항
   * * 1. Trunk Mass =>100000 or 터무니 없는 숫자로 키우기
   * * 2. Control Frequency => 1000Hz 변경
   * * 3. 중력값 변경 => 0
   * * 4.
   */
  d->ctrl[2] = 0.1*traj_generator.system_ID(d->time);
  robot.joint_torque_des_[0][1] = d->ctrl[2];

  d->qpos[0] = 0.0;
  d->qpos[1] = 0.0;
  d->qpos[2] = 1;
  d->qpos[3] = 0.0;
  d->qpos[4] = 0.0;
  d->qpos[5] = 0.0;
  d->qpos[6] = 0.0;
  d->qvel[0] = 0.0;
  d->qvel[1] = 0.0;
  d->qvel[2] = 0.0;
  d->qvel[3] = 0.0;
  d->qvel[4] = 0.0;
  d->qvel[5] = 0.0;
  /*********************** */
  d->qpos[7] = 0.0;
  d->qvel[6] = 0.0;

  d->qpos[8] =  0.0;
  d->qpos[7] =  0.0;

  // d->qpos[9] = 0.0;
  // d->qvel[8] = 0.0;

  d->qpos[10] = 0.0;
  d->qvel[9] = 0.0;

  d->qpos[11] = 0;
  d->qvel[10] = 0;

  d->qpos[12] = 0;
  d->qvel[11] = 0;

  d->qpos[13] = 0;
  d->qvel[12] = 0;

  d->qpos[14] = 0;
  d->qvel[13] = 0;

  d->qpos[15] = 0;
  d->qvel[14] = 0;

  d->qpos[16] = 0;
  d->qvel[15] = 0;

  d->qpos[17] = 0;
  d->qvel[16] = 0;

  d->qpos[18] = 0;
  d->qvel[17] = 0;

  d->qpos[19] = 0;
  d->qvel[18] = 0;

  d->qpos[20] = 0;
  d->qvel[19] = 0;

  d->qpos[21] = 0;
  d->qvel[20] = 0;

  d->qpos[22] = 0;
  d->qvel[21] = 0;
}

void apply_joint_control(mjData * d)
{
  /**
   * @param : qpos[0~6]: Trunk Pos, Quaternions
   * @param : qpos[7+4*i] qvel[6+4*i] suspension
   * @param : qpos[8+4*i] qvel[7+4*i] Sus_Cover !-> No Use Fix Angle
   * @param : qpos[9+4*i] qvel[8+4*i] Steer
   * @param : qpos[10+4*i] qvel[9+4*i] Drive
   * @param : ctrl[3*i] suspension , ctrl[1+3*i] steer, ctrl[2+3*i] drive
   */

  //** 4절 링크 모션을 위한 Joint 구속에 해당하는 부분 */
  d->qpos[8] =  -d->qpos[7]/498;
  d->qpos[12] = -d->qpos[11]/498;
  d->qpos[16] = -d->qpos[15]/498;
  d->qpos[20] = -d->qpos[19]/498;

  //! 지금은 구동모터만 제어해주기 위함으로 setting
  for (size_t i = 0; i < 4; i++)
  {
    //* Suspension 제어
    d->ctrl[0 + 4*i] = robot.joint_torque_des_[i][0];

    //* Suspension Cover 제어
    //! 4절 링크 모션에 문제가 생기면 사용할려고 했던 부분 일단 사용 X
    // d->ctrl[1 + 4*i] = 50*(0-d->qpos[8]) + 5*(0-d->qvel[7]);

    //* Steer 제어
    d->ctrl[2 + 4*i] = robot.joint_torque_des_[i][1];

    //* Drive 제어
    d->ctrl[3 + 4*i] = robot.joint_torque_des_[i][2]; // Torque 양수값 넣어서 도는 방향 확인
  }

}

void YCM_controller()
{
  //! Custom Controller
  // bool bIsPerturbOn = false;

  //* trajectory generation
  traj_generator.driving_test(d->time);
  // traj_generator.suspension_test(d->time);

  //* Controller
  motor_ctrl.suspension_motor_control();
  motor_ctrl.drive_motor_control();
  motor_ctrl.steer_motor_control();

  // Motor_ID_Setting(d); // 사용할 때 이 함수 안에 꼭 읽어보기
  apply_joint_control(d);
}

void Mu_Lambda_setting()
{
  //** Mu_Lambda Curve Setting */
  estimator.cal_mu();
  if (d->time > 2)
  {
    for (size_t i = 0; i < 4; i++)
    {
      m->geom_friction[36+7*3*i] = estimate_param_ptr->mu_[i];
      // m->geom_friction[36+7*3*i] = 0.9;

    }

  }

}

//* ******************************************************************************************** *//

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load())
  {
    // ! Exit simulation if time exceeded the specified end time //
    if (sim.d_->time > sim_end_time)
    {
      sim.exitrequest = 1;
    }

    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          if (sim.ctrl_noise_std) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

            for (int i=0; i<m->nu; i++) {
              // update noise
              ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // * **************************************************************************************** *//

            //* ***** GENERATE DESIRED JOINT COMMAND AND APPLY CONTROL INPUT FOR SIMULATION***** *//
            //! Custom Controller
            YCM_controller();
            Mu_Lambda_setting();

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            stepped = true;
            //* ******************************************************************************** *//
            //* ******** READ SENSOR DATA AND CONDUCT CALCULATOIN FOR STATE ESTIMATION ********* *//
            robot.get_sensor_data(d);

            if (loop_iter % data_logger.get_logging_freq() == 0)
            {
              data_logger.save_data(m, d);
            }
            loop_iter++;


          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

            // * **************************************************************************************** *//
            //* ***** GENERATE DESIRED JOINT COMMAND AND APPLY CONTROL INPUT FOR SIMULATION***** *//
            //! Custom Controller
            YCM_controller();
            Mu_Lambda_setting();

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            stepped = true;
            //* ******************************************************************************** *//
            // TODO: Make this sequence as a function
            //* ******** READ SENSOR DATA AND CONDUCT CALCULATOIN FOR STATE ESTIMATION ********* *//
            robot.get_sensor_data(d);

            if (loop_iter % data_logger.get_logging_freq() == 0)
            {
              data_logger.save_data(m, d);
            }
            loop_iter++;

              //* **************************************************************************************** *//
              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = "../model/mcl_rover/scene_terrain.xml"; // Settting My model
  if (argc >  1) {
    filename = argv[1];
  }

  //* ****************************************************************************************** *//
  //* ********** CREATE MUJOCO INTERFACE OBJECT AND SENSOR & ACTUATOR POINTER ****************** *//
  //* ****************************************************************************************** *//
  //! ptr 만들면 무조건 여기에 set 함수 만들어줘야함
  auto mujoco_interface = std::make_shared<MuJoCoInterface<float>>(sim.get(), robot);
  foot_traj_ptr = traj_generator.set_foot_traj_ptr();
  joint_traj_ptr = traj_generator.set_joint_traj_ptr();
  estimate_param_ptr = estimator.set_estimate_param_ptr();


  //* ****************************************************************************************** *//
  //* *************** GET POINTER s.t. EACH OBJECT CAN SHARE THE SAME DATA ********************* *//
  //* ****************************************************************************************** *//
  //* **************** Changmin StanceForceControl Test Version ******************************** *//
  //! 다른 folder로 넘겨줄거면 여기서 get 함수 만들어 줘야함

  motor_ctrl.get_traj_pointer(foot_traj_ptr, joint_traj_ptr);
  data_logger.get_traj_ptr(foot_traj_ptr, joint_traj_ptr);
  data_logger.get_estimate_ptr(estimate_param_ptr);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
