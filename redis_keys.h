/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief
 * @version 0.1
 * @date 2022-04-30
 *
 * @copyright Copyright (c) 2022
 *
 */
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::toro::controller";
const std::string KEYBOARD_KEY = "keyboard";
const std::string FORCE_SENSOR_KEY_FL = "force_sensor_FL";
const std::string FORCE_SENSOR_KEY_FR = "force_sensor_FR";
const std::string FORCE_SENSOR_KEY_RL = "force_sensor_RL";
const std::string FORCE_SENSOR_KEY_RR = "force_sensor_RR";