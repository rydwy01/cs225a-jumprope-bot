/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <Sai2Model.h>
#include "Sai2Primitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <random>

using namespace std;
using namespace Eigen;
using namespace Sai2Primitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/spot.urdf";
const string body_name = "body";
const Vector3d pos_in_body = Vector3d(0.0,0.0,0.0);
// States 
enum State {
	POSTURE = 0, 
	FEET_FIXED_MOTION,
    LAUNCH,
    FLIGHT
};

int main() {
    //REDIS SETUP
    

	// initial state 
	// int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = Sai2Common::RedisClient();
	redis_client.connect();

    int state=0;
    redis_client.setInt(KEYBOARD_KEY,state);
    state = redis_client.getInt(KEYBOARD_KEY); //STATE_KEY IN REDIS KEYS--string stateKey = whatever

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

    

    //insie while loop
	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // joint torques
	MatrixXd N_prec = MatrixXd::Identity(dof, dof); //null space of previous task

    // create tasks
    auto body_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "body", Affine3d::Identity());
    body_task->disableInternalOtg(); //disabling internal trajectory generation? what for?
    body_task->setPosControlGains(400, 40, 0);
    body_task->setOriControlGains(400, 40, 0);
    Vector3d body_pos;

    // auto bodyVel_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "bodyVel", Affine3d::Identity());
    // bodyVel_task->disableInternalOtg(); //disabling internal trajectory generation? what for?
    // bodyVel_task->setPosControlGains(400, 40, 0);
    // bodyVel_task->setOriControlGains(400, 40, 0);
    // Vector3d body_pos;
    Vector3d body_vel;
    //ADDED
    Matrix3d body_rot = Matrix3d::Identity();
    Matrix3d initial_rot = Matrix3d::Identity();
    initial_rot = robot->rotation(body_name);

	// create map for feet task 
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> primary_tasks;
    const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"};
    const std::vector<Vector3d> primary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    // control points are the origins of the primary control links?
    std::vector<Vector3d> controlled_directions_translation = {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};

    for (int i = 0; i < primary_control_links.size(); ++i) {        
        Affine3d compliant_frame = Affine3d::Identity(); // defines reference frame for task relative to a specific link
        compliant_frame.translation() = primary_control_points[i];
        primary_tasks[primary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, //tasks relative to frame of individual feet
                                                                                                    primary_control_links[i], 
                                                                                                    controlled_directions_translation, 
                                                                                                    std::vector<Vector3d>{}, 
                                                                                                    compliant_frame);
        primary_tasks[primary_control_links[i]]->disableInternalOtg();
        primary_tasks[primary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
        primary_tasks[primary_control_links[i]]->setPosControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->setOriControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->handleAllSingularitiesAsType1(true);
    }

    MatrixXd U = MatrixXd::Zero(robot->dof() - 6, robot->dof()); // underactuated robot matrix dof-6 rows, dof columns
    //rows control actuation directions, columns dictate # of joints: in this case, 6 DoF are underactuated compared to joints
    //total robot dof = 18 ----- subtract 6 DoF for indirectly actuated robot body
    U.block(0, 6, robot->dof() - 6, robot->dof() - 6).setIdentity();
    // cout << robot->dof(); //ADDED**************************************************************
	auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
	joint_task->disableInternalOtg();
	VectorXd q_desired = robot->q();
	joint_task->setGains(400, 40, 0); //18x1 vector for each joint DoF in RADIANS
    // first 6 elements are underactuated: 3 prismatic, 3 rotation. other 12 are joints rotation angles.
	joint_task->setGoalPosition(q_desired);
    // cout << q_desired;
	// get starting poses
    std::vector<Affine3d> primary_starting_pose;
    for (int i = 0; i < primary_control_links.size(); ++i) {
        Affine3d current_pose; //WHAT IS AFFINE3D
        current_pose.translation() = robot->position(primary_control_links[i], primary_control_points[i]);
        //translation spits out 12x1--- xyz position of each link control point?
        current_pose.linear() = robot->rotation(primary_control_links[i]);
        //translation spits out 12x3: orientation of each link? but there's only 4 links?
        // cout << primary_control_links[i];
        // cout << current_pose.linear();
        primary_starting_pose.push_back(current_pose); //appends current pose to primary starting pose
    }
    // cout << primary_starting_pose;

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	Sai2Common::LoopTimer timer(control_freq, 1e6);

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 1
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();
        state = redis_client.getInt(KEYBOARD_KEY);
        // cout << state;
		if (state == POSTURE) { // initial neutral position FULLY ACTUATED
			// update task model 
			N_prec.setIdentity(); //previous nullspace = all ones == takes priority 1 since in all spaces
			joint_task->updateTaskModel(N_prec); // sets joint task as highest priority

			command_torques = joint_task->computeTorques();
            body_rot = robot->rotation(body_name);
            // cout << body_rot.rows();

			if ((robot->q() - q_desired).norm() < 1e-2) { //if error between joint angles and desired angles is <error
				// cout << "Posture To Motion" << endl;
                
				for (auto name : primary_control_links) {
					primary_tasks[name]->reInitializeTask(); // sets desired/goal states to current position
				}
                body_pos = robot->position("body", Vector3d(0, 0, 0)); //position of body origin
				joint_task->reInitializeTask(); //set joint tasks to current task
                // cout << joint_task;
				// state = FEET_FIXED_MOTION;
			}
		} else if (state == FEET_FIXED_MOTION) { //crouching
            // get underactuation matrix
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof()); //jacobian of feet contacts. 3 translation directions * 4 feet
            for (int i = 0; i < 4; ++i) {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr);//nullspace of legs, all other motions outside of feet 
            MatrixXd UNr = U * Nr; //all robot joints projected across feet null space--now UNr motions won't effect feet location
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() * \
                                    (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();

            // update body task
            N_prec.setIdentity();
            body_task->updateTaskModel(N_prec);
            N_prec = body_task->getTaskAndPreviousNullspace();
                
            // redundancy completion 

            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            // body_task->setGoalPosition(body_pos + Vector3d(0, 0, 0.1 * sin(2 * M_PI * time)));//GOAL POSITION SETTING
            
            Vector3d goalPos = Vector3d(0,0,0.2);
            if (abs(body_pos(2) - goalPos(2)) > 1e-1)  {
                body_task->setGoalPosition(body_pos-Vector3d(0,0,0.05));//GOAL POSITION SETTING
                Matrix3d neutralOrientation = Matrix3d::Identity();
                Vector3d zeroAng = Vector3d::Zero();
                body_task->setGoalOrientation(initial_rot);//GOAL ORIENTATION SETTING
            } else  {
                body_task->setGoalPosition(body_pos);
            }
            
            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            command_torques = U.transpose() * UNr_bar.transpose() * command_torques;  // project underactuation 
            body_pos = robot->position("body", Vector3d(0, 0, 0)); //position of body origin

            // cout << body_pos;
            if (abs(body_pos(2) - goalPos(2)) < 1e-1)  {
                cout << "crouched";
                // state = LAUNCH;
            }


        } else if (state == LAUNCH) {
            // get underactuation matrix
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof()); //jacobian of feet contacts. 3 translation directions * 4 feet
            for (int i = 0; i < 4; ++i) {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr);//nullspace of legs, all other motions outside of feet 
            MatrixXd UNr = U * Nr; //all robot joints projected across feet null space--now UNr motions won't effect feet location
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() * \
                                    (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();

            // update body task
            N_prec.setIdentity();
            body_task->updateTaskModel(N_prec);
            N_prec = body_task->getTaskAndPreviousNullspace();
                
            // redundancy completion 
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            
            // body_task->setPosControlGains(200, 40, 0);
            // body_task->setOriControlGains(200, 40, 0);
            // body_task->setGoalPosition(body_pos + Vector3d(0, 0, 0.1 * sin(2 * M_PI * time)));//GOAL POSITION SETTING
            body_task->setGoalPosition(Vector3d(0,0,0.8));//GOAL POSITION SETTING
            body_task->setGoalOrientation(initial_rot);//GOAL ORIENTATION SETTING

            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            command_torques = U.transpose() * UNr_bar.transpose() * command_torques;  // project underactuation 

            body_vel = robot->linearVelocity(body_name,pos_in_body);
            double desired_launch_vel = 0.5;
            if (abs(body_vel[2]-desired_launch_vel) < 0.01)  {
                cout << "launched";
                // state = FLIGHT;
            }

        } else if (state == FLIGHT) {
            // update primary task model
            N_prec.setIdentity();
            for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
                it->second->updateTaskModel(N_prec);
                // N_prec = it->second->getTaskAndPreviousNullspace();
            }
                
            // redundancy completion 
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            int i = 0;
            for (auto name : primary_control_links) {
                primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation());
                // primary_tasks[name]->setGoalOrientation();
                command_torques += primary_tasks[name]->computeTorques();
                ++i;
            }
            
            command_torques += joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
        }

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
