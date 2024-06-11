// /**
//  * @file simviz.cpp
//  * @brief Simulation and visualization of panda robot with 1 DOF gripper
//  *
//  */
// #include <math.h>
// #include <signal.h>
// #include <iostream>
// #include <mutex>
// #include <string>
// #include <thread>
// #include <fstream>
// #include <filesystem>
// #include <iostream>
// #include <vector>
// #include <typeinfo>
// #include <random>
// #include "Sai2Graphics.h"
// #include "Sai2Model.h"
// #include "Sai2Simulation.h"
// #include "Sai2Primitives.h"
// #include "redis/RedisClient.h"
// #include "timer/LoopTimer.h"
// #include "logger/Logger.h"
// #include "redis_keys.h"

// bool fSimulationRunning = true;
// void sighandler(int) { fSimulationRunning = false; }

// using namespace Eigen;
// using namespace std;

// // mutex and globals
// VectorXd ui_torques;
// mutex mutex_torques, mutex_update;

// // specify urdf and robots
// const string world_file = "./resources/world_spot.urdf";
// const string robot_file = "./resources/spot.urdf";
// const string robot_name = "spot";
// const string camera_name = "camera_fixed";

// // dynamic objects information
// const vector<std::string> object_names = {"jumprope"};
// vector<Affine3d> object_poses;
// vector<VectorXd> object_velocities;
// const int n_objects = object_names.size(); // simulation thread

// void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);

// int main()
// {
//     std::cout << "Loading URDF world model file: " << world_file << endl; // start redis client
//     auto redis_client = Sai2Common::RedisClient();
//     redis_client.connect(); // set up signal handler
//     signal(SIGABRT, &sighandler);
//     signal(SIGTERM, &sighandler);
//     signal(SIGINT, &sighandler); // load graphics scene
//     auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
//     graphics->setBackgroundColor(66.0 / 255, 135.0 / 255, 245.0 / 255); // set blue background
//     // graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
//     // graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes
//     graphics->addUIForceInteraction(robot_name); // load robots
//     auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
//     VectorXd q_init = VectorXd::Zero(robot->dof());
//     q_init << 0, 0, 0.52, 0, 0, 0,
//         0, 0.78539816339, -1.57079632679,
//         0, 0.78539816339, -1.57079632679,
//         0, 0.78539816339, -1.57079632679,
//         0, 0.78539816339, -1.57079632679;
//     robot->setQ(q_init);
//     robot->setDq(0 * robot->q());
//     robot->updateModel();
//     ui_torques = VectorXd::Zero(robot->dof()); // load simulation world
//     auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
//     sim->setJointPositions(robot_name, robot->q());
//     sim->setJointVelocities(robot_name, robot->dq());
//     const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"};
//     const Vector3d sensor_pos_in_link = Vector3d(0.0, 0.0, 0.0);
//     const string robot_name = "spot";
//     Affine3d T_sensor_FL = Affine3d::Identity();
//     Affine3d T_sensor_FR = Affine3d::Identity();
//     Affine3d T_sensor_RL = Affine3d::Identity();
//     Affine3d T_sensor_RR = Affine3d::Identity();
//     T_sensor_FL.translation() = sensor_pos_in_link;
//     T_sensor_FR.translation() = sensor_pos_in_link;
//     T_sensor_RL.translation() = sensor_pos_in_link;
//     T_sensor_RR.translation() = sensor_pos_in_link;
//     sim->addSimulatedForceSensor(robot_name, primary_control_links[0], T_sensor_FL, 15.0);
//     sim->addSimulatedForceSensor(robot_name, primary_control_links[1], T_sensor_FR, 15.0);
//     sim->addSimulatedForceSensor(robot_name, primary_control_links[2], T_sensor_RL, 15.0);
//     sim->addSimulatedForceSensor(robot_name, primary_control_links[3], T_sensor_RR, 15.0);
//     // graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);
//     // graphics->addForceSensorDisplay(sim->getAllForceSensorData()[1]);
//     // graphics->addForceSensorDisplay(sim->getAllForceSensorData()[2]);
//     // graphics->addForceSensorDisplay(sim->getAllForceSensorData()[3]);    // fill in object information
//     for (int i = 0; i < n_objects; ++i)
//     {
//         object_poses.push_back(sim->getObjectPose(object_names[i]));
//         object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
//     } // set co-efficient of restition to zero for force control
//     sim->setCollisionRestitution(0.0); // set co-efficient of friction
//     sim->setCoeffFrictionStatic(1.0);
//     sim->setCoeffFrictionDynamic(1.0); /*------- Set up visualization -------*/
//     // init redis client values
//     redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
//     redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq());
//     redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q());

//     // start simulation thread
//     thread sim_thread(simulation, sim);

//     Eigen::Affine3d prev_jumprope_pos = Eigen::Affine3d::Identity();
//     prev_jumprope_pos.translate(Eigen::Vector3d(-1.0, 0.0, 1.2));

//     // create a loop timer
//     const double control_freq = 1000;
//     Sai2Common::LoopTimer timer(control_freq);

//     int num_radians = 1;
//     const int INCREASE_SPEED = 9;
//     const int DECREASE_SPEED = 8;
//     int pre_keyboardkey = 0;
//     int state = 0;
//     // while window is open:
//     while (graphics->isWindowOpen() && fSimulationRunning)
//     {
//         // redis_client.setInt(KEYBOARD_KEY, state);
//         state = redis_client.getInt(KEYBOARD_KEY);

//         // std::cout << object_names << std::endl;
//         Eigen::Affine3d jumprope_pos = prev_jumprope_pos;
//         // wait for next scheduled loop
//         timer.waitForNextLoop();
//         double time = timer.elapsedTime();
//         // 1 radian per second to start, and allow user to increase/decrease for in the range of 1 to 10 radians per iteration
//         if (pre_keyboardkey != state)
//         {
//             if (state == INCREASE_SPEED && num_radians < 10)
//             {
//                 num_radians += 1;
//                 cout << "num radians increased to: " << endl;
//                 cout << num_radians << endl;
//             }
//             else if (state == DECREASE_SPEED && num_radians > 1)
//             {
//                 num_radians -= 1;
//                 cout << "num radians decreased to: " << endl;
//                 cout << num_radians << endl;
//             }
//         }

//         pre_keyboardkey = state;
//         double angle = num_radians * M_PI / 180;
//         jumprope_pos.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));

//         graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
//         {
//             lock_guard<mutex> lock(mutex_update);
//             for (int i = 0; i < n_objects; ++i)
//             {
//                 graphics->updateObjectGraphics(object_names[i], jumprope_pos);
//             }
//         }
//         graphics->renderGraphicsWorld();
//         {
//             lock_guard<mutex> lock(mutex_torques);
//             ui_torques = graphics->getUITorques(robot_name);
//         }
//         prev_jumprope_pos = jumprope_pos;
//         // graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
//         // graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[1]);
//         // graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[2]);
//         // graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[3]);    }
//     } // stop simulation
//     fSimulationRunning = false;
//     sim_thread.join();
//     return 0;
// } //------------------------------------------------------------------------------
// void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim)
// {
//     // fSimulationRunning = true;
//     const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"}; // create redis client
//     auto redis_client = Sai2Common::RedisClient();
//     redis_client.connect(); // create a timer
//     double sim_freq = 1000;
//     Sai2Common::LoopTimer timer(sim_freq);
//     sim->setTimestep(1.0 / sim_freq);
//     sim->enableGravityCompensation(true);
//     sim->enableJointLimits(robot_name);
//     while (fSimulationRunning)
//     {
//         timer.waitForNextLoop();
//         VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
//         if (control_torques.norm() != 0)
//         {
//             sim->enableGravityCompensation(false);
//         }
//         {
//             lock_guard<mutex> lock(mutex_torques);
//             sim->setJointTorques(robot_name, control_torques + ui_torques);
//         }
//         sim->integrate();
//         redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
//         redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));
//         redis_client.setEigen(FORCE_SENSOR_KEY_FL, sim->getSensedForce(robot_name, primary_control_links[0]));
//         redis_client.setEigen(FORCE_SENSOR_KEY_FR, sim->getSensedForce(robot_name, primary_control_links[1]));
//         redis_client.setEigen(FORCE_SENSOR_KEY_RL, sim->getSensedForce(robot_name, primary_control_links[2]));
//         redis_client.setEigen(FORCE_SENSOR_KEY_RR, sim->getSensedForce(robot_name, primary_control_links[3]));
//         // update object information
//         {
//             lock_guard<mutex> lock(mutex_update);
//             for (int i = 0; i < n_objects; ++i)
//             {
//                 object_poses[i] = sim->getObjectPose(object_names[i]);
//                 object_velocities[i] = sim->getObjectVelocity(object_names[i]);
//             }
//         }
//     }
//     timer.stop();
//     cout << "\nSimulation loop timer stats:\n";
//     timer.printInfoPostRun();
// }

// JOSH's code

/**
 * @file simviz.cpp
 * @brief Simulation and visualization of panda robot with 1 DOF gripper
 *
 */
#include <math.h>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <random>
#include "Sai2Graphics.h"
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"
#include "redis_keys.h"

bool fSimulationRunning = true;
void sighandler(int) { fSimulationRunning = false; }

using namespace Eigen;
using namespace std;

// mutex and globals
VectorXd ui_torques;
mutex mutex_torques, mutex_update;

// specify urdf and robots
const string world_file = "./resources/world_spot.urdf";
const string robot_file = "./resources/spot.urdf";
const string robot_name = "spot";
const string camera_name = "camera_fixed";

// dynamic objects information
const vector<std::string> object_names = {"jumprope"};
vector<Affine3d> object_poses;
vector<VectorXd> object_velocities;
const int n_objects = object_names.size(); // simulation thread
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim);
int main()
{
    std::cout << "Loading URDF world model file: " << world_file << endl; // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect(); // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler); // load graphics scene
    auto graphics = std::make_shared<Sai2Graphics::Sai2Graphics>(world_file, camera_name, false);
    graphics->setBackgroundColor(66.0 / 255, 135.0 / 255, 245.0 / 255); // set blue background
    // graphics->showLinkFrame(true, robot_name, "link7", 0.15);  // can add frames for different links
    // graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);  // set the near and far clipping planes
    graphics->addUIForceInteraction(robot_name); // load robots
    auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
    VectorXd q_init = VectorXd::Zero(robot->dof());
    q_init << 0, 0, 0.52, 0, 0, 0,
        0, 0.78539816339, -1.57079632679,
        0, 0.78539816339, -1.57079632679,
        0, 0.78539816339, -1.57079632679,
        0, 0.78539816339, -1.57079632679;
    robot->setQ(q_init);
    robot->setDq(0 * robot->q());
    robot->updateModel();
    ui_torques = VectorXd::Zero(robot->dof()); // load simulation world
    auto sim = std::make_shared<Sai2Simulation::Sai2Simulation>(world_file, false);
    sim->setJointPositions(robot_name, robot->q());
    sim->setJointVelocities(robot_name, robot->dq());
    const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"};
    const Vector3d sensor_pos_in_link = Vector3d(0.0, 0.0, 0.0);
    const string robot_name = "spot";
    Affine3d T_sensor_FL = Affine3d::Identity();
    Affine3d T_sensor_FR = Affine3d::Identity();
    Affine3d T_sensor_RL = Affine3d::Identity();
    Affine3d T_sensor_RR = Affine3d::Identity();
    T_sensor_FL.translation() = sensor_pos_in_link;
    T_sensor_FR.translation() = sensor_pos_in_link;
    T_sensor_RL.translation() = sensor_pos_in_link;
    T_sensor_RR.translation() = sensor_pos_in_link;
    sim->addSimulatedForceSensor(robot_name, primary_control_links[0], T_sensor_FL, 15.0);
    sim->addSimulatedForceSensor(robot_name, primary_control_links[1], T_sensor_FR, 15.0);
    sim->addSimulatedForceSensor(robot_name, primary_control_links[2], T_sensor_RL, 15.0);
    sim->addSimulatedForceSensor(robot_name, primary_control_links[3], T_sensor_RR, 15.0);
    graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);
    graphics->addForceSensorDisplay(sim->getAllForceSensorData()[1]);
    graphics->addForceSensorDisplay(sim->getAllForceSensorData()[2]);
    graphics->addForceSensorDisplay(sim->getAllForceSensorData()[3]); // fill in object information
    for (int i = 0; i < n_objects; ++i)
    {
        object_poses.push_back(sim->getObjectPose(object_names[i]));
        object_velocities.push_back(sim->getObjectVelocity(object_names[i]));
    } // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0); // set co-efficient of friction
    sim->setCoeffFrictionStatic(1.0);
    sim->setCoeffFrictionDynamic(1.0); /*------- Set up visualization -------*/
    // init redis client values
    redis_client.setEigen(JOINT_ANGLES_KEY, robot->q());
    redis_client.setEigen(JOINT_VELOCITIES_KEY, robot->dq());
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * robot->q()); // start simulation thread
    thread sim_thread(simulation, sim);

    // My jumprope additions
    Eigen::Affine3d prev_jumprope_pos = Eigen::Affine3d::Identity();
    prev_jumprope_pos.translate(Eigen::Vector3d(-1.0, 0.0, 1.2));

    // create a loop timer
    const double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq);

    int num_radians = 1;
    const int INCREASE_SPEED = 9;
    const int DECREASE_SPEED = 8;
    int pre_keyboardkey = 0;
    int state = 0;

    // while window is open:
    while (graphics->isWindowOpen() && fSimulationRunning)
    {
        redis_client.setInt(KEYBOARD_KEY, state);
        state = redis_client.getInt(KEYBOARD_KEY);

        // std::cout << object_names << std::endl;
        Eigen::Affine3d jumprope_pos = prev_jumprope_pos;
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();
        // 1 radian per second to start, and allow user to increase/decrease for in the range of 1 to 10 radians per iteration
        if (pre_keyboardkey != state)
        {
            if (state == INCREASE_SPEED && num_radians < 10)
            {
                num_radians += 1;
                cout << "num radians increased to: " << endl;
                cout << num_radians << endl;
            }
            else if (state == DECREASE_SPEED && num_radians > 1)
            {
                num_radians -= 1;
                cout << "num radians decreased to: " << endl;
                cout << num_radians << endl;
            }
        }

        pre_keyboardkey = state;
        double angle = num_radians * M_PI / 180;
        jumprope_pos.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
        graphics->updateRobotGraphics(robot_name, redis_client.getEigen(JOINT_ANGLES_KEY));
        {
            lock_guard<mutex> lock(mutex_update);
            for (int i = 0; i < n_objects; ++i)
            {
                graphics->updateObjectGraphics(object_names[i], jumprope_pos);
            }
        }
        graphics->renderGraphicsWorld();
        {
            lock_guard<mutex> lock(mutex_torques);
            ui_torques = graphics->getUITorques(robot_name);
        }
        graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
        graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[1]);
        graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[2]);
        graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[3]);
        prev_jumprope_pos = jumprope_pos;
    } // stop simulation
    fSimulationRunning = false;
    sim_thread.join();
    return 0;
} //------------------------------------------------------------------------------
void simulation(std::shared_ptr<Sai2Simulation::Sai2Simulation> sim)
{
    // fSimulationRunning = true;
    const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"}; // create redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect(); // create a timer
    double sim_freq = 1000;
    Sai2Common::LoopTimer timer(sim_freq);
    sim->setTimestep(1.0 / sim_freq);
    sim->enableGravityCompensation(true);
    sim->enableJointLimits(robot_name);
    while (fSimulationRunning)
    {
        timer.waitForNextLoop();
        VectorXd control_torques = redis_client.getEigen(JOINT_TORQUES_COMMANDED_KEY);
        if (control_torques.norm() != 0)
        {
            sim->enableGravityCompensation(false);
        }
        {
            lock_guard<mutex> lock(mutex_torques);
            sim->setJointTorques(robot_name, control_torques + ui_torques);
        }
        sim->integrate();
        redis_client.setEigen(JOINT_ANGLES_KEY, sim->getJointPositions(robot_name));
        redis_client.setEigen(JOINT_VELOCITIES_KEY, sim->getJointVelocities(robot_name));
        redis_client.setEigen(FORCE_SENSOR_KEY_FL, sim->getSensedForce(robot_name, primary_control_links[0]));
        redis_client.setEigen(FORCE_SENSOR_KEY_FR, sim->getSensedForce(robot_name, primary_control_links[1]));
        redis_client.setEigen(FORCE_SENSOR_KEY_RL, sim->getSensedForce(robot_name, primary_control_links[2]));
        redis_client.setEigen(FORCE_SENSOR_KEY_RR, sim->getSensedForce(robot_name, primary_control_links[3]));
        // update object information
        {
            lock_guard<mutex> lock(mutex_update);
            for (int i = 0; i < n_objects; ++i)
            {
                object_poses[i] = sim->getObjectPose(object_names[i]);
                object_velocities[i] = sim->getObjectVelocity(object_names[i]);
            }
        }
    }
    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
}
