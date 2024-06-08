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
void sighandler(int) { runloop = false; }

#include "redis_keys.h"

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/spot.urdf";
const string body_name = "body";
const Vector3d pos_in_body = Vector3d(0.0, 0.0, 0.0);


double initial_height = 0.0;
double initial_velocity = 0.0;
double launch_start_time = 0.0;
bool launch_initialized = false;

// States
enum State
{
    POSTURE = 0,
    FEET_FIXED_MOTION,
    LAUNCH,
    FLIGHT,
    FALL
};


Vector3d getTraj(string trajType, double tStart, double tCurrent, double tLength, Vector3d initialPos, Vector3d initialVel, Vector3d finalPos, Vector3d finalVel){

    Vector3d traj;
    double slope;
    if(trajType == "linearPos"){
        if(tCurrent<=tLength+tStart){
            for(int i=0; i<=2; i++){
                slope = (initialPos(i)-finalPos(i))/(-tLength);
                traj(i) = slope*(tCurrent-tStart)+initialPos(i);
            }
        }
        else{
            traj = finalPos;
        }

    }

    if(trajType == "linearVel"){
        if(tCurrent<=tLength+tStart){
            for(int i=0; i<=2; i++){
                slope = (initialVel(i)-finalVel(i))/(-tLength);
                traj(i) = slope*(tCurrent-tStart)+initialVel(i);
            }
        }
        else{
            traj = finalVel;
        }

    }

    // if(trajType == "linearOri"){
    //     if(tCurrent<=tLength){
    //         for(i=0; i<=2; i++){
    //             slope = (initialOri(i)-finalOri(i))/(tLength);
    //             traj(i) = slope*(tCurrent-tStart)+initialOri(i);
    //         }
    //     }
    //     else{
    //         traj = finalOri;
    //     }
    // }
    return traj;
}

int main()
{
    // REDIS SETUP

    // initial state
    // int state = POSTURE;
    string controller_status = "1";




    Vector3d traj;

    bool startToggleFeetFixedMotion = true;
    bool startToggleLaunch = true;
    double startTime = 0;
    Vector3d initialPos;
    Vector3d initialVel;
    Vector3d initialOri;

    Vector3d goalPos;
    Vector3d goalVel;





    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    int state = 0;
    redis_client.setInt(KEYBOARD_KEY, state);
    state = redis_client.getInt(KEYBOARD_KEY); // STATE_KEY IN REDIS KEYS--string stateKey = whatever

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots, read current state and update the model
    auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
    robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
    robot->updateModel();

    // insie while loop
    //  prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof); // joint torques
    MatrixXd N_prec = MatrixXd::Identity(dof, dof); // null space of previous task

    // torque limits
    //  Define torque limits (in Newton-meters, for example)
    VectorXd torque_limits_max = VectorXd::Constant(dof, 200.0);  // Set appropriate values for your robot
    VectorXd torque_limits_min = VectorXd::Constant(dof, -200.0); // Set appropriate values for your robot

    // create tasks
    auto body_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "body", Affine3d::Identity());
    body_task->disableInternalOtg(); // disabling internal trajectory generation? what for?
    body_task->setPosControlGains(400, 40, 0);
    body_task->setOriControlGains(400, 40, 0);
    Vector3d body_pos;

    // auto bodyVel_task = std::make_shared<Sai2Primitives::MotionForceTask>(robot, "bodyVel", Affine3d::Identity());
    // bodyVel_task->disableInternalOtg(); //disabling internal trajectory generation? what for?
    // bodyVel_task->setPosControlGains(400, 40, 0);
    // bodyVel_task->setOriControlGains(400, 40, 0);
    // Vector3d body_pos;
    Vector3d body_vel;
    // ADDED
    Matrix3d body_rot = Matrix3d::Identity();
    Matrix3d initial_rot = Matrix3d::Identity();
    initial_rot = robot->rotation(body_name);
    Matrix3d neutralOrientation = Matrix3d::Identity();

    // cout << randomTest << "\n";
    // Matrix3d randomMatDiag = Matrix3d::Zero();
    // randomMatDiag << cos(0.2), 0.0, sin(0.2), 0, 1, 0, -sin(0.2), 0, cos(0.2);
    // randomMatDiag << 0.883, -0.211, 0.419, 0.321, 0.923, -0.211, -0.342, 0.321, 0.883;
    // randomMatDiag = randomTest.asDiagonal();

    // cout << randomTest << "\n";
    // cout << randomMatDiag << "\n";

    // create map for feet task
    std::map<std::string, std::shared_ptr<Sai2Primitives::MotionForceTask>> primary_tasks;
    const std::vector<std::string> primary_control_links = {"front_left_foot", "front_right_foot", "rear_left_foot", "rear_right_foot"};
    const std::vector<Vector3d> primary_control_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

    // control points are the origins of the primary control links?
    std::vector<Vector3d> controlled_directions_translation = {Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};

    for (int i = 0; i < primary_control_links.size(); ++i)
    {
        Affine3d compliant_frame = Affine3d::Identity(); // defines reference frame for task relative to a specific link
        compliant_frame.translation() = primary_control_points[i];
        primary_tasks[primary_control_links[i]] = std::make_shared<Sai2Primitives::MotionForceTask>(robot, // tasks relative to frame of individual feet
                                                                                                    primary_control_links[i],
                                                                                                    controlled_directions_translation,
                                                                                                    std::vector<Vector3d>{},
                                                                                                    compliant_frame);
        primary_tasks[primary_control_links[i]]->disableInternalOtg();
        primary_tasks[primary_control_links[i]]->setDynamicDecouplingType(Sai2Primitives::DynamicDecouplingType::FULL_DYNAMIC_DECOUPLING);
        primary_tasks[primary_control_links[i]]->setPosControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->setOriControlGains(400, 40, 0);
        primary_tasks[primary_control_links[i]]->handleAllSingularitiesAsType1(true);
        
        primary_tasks[primary_control_links[i]]->setForceSensorFrame(primary_control_links[i],Affine3d::Identity());
    }

    MatrixXd U = MatrixXd::Zero(robot->dof() - 6, robot->dof()); // underactuated robot matrix dof-6 rows, dof columns
    // rows control actuation directions, columns dictate # of joints: in this case, 6 DoF are underactuated compared to joints
    // total robot dof = 18 ----- subtract 6 DoF for indirectly actuated robot body
    U.block(0, 6, robot->dof() - 6, robot->dof() - 6).setIdentity();
    // cout << robot->dof(); //ADDED**************************************************************
    auto joint_task = std::make_shared<Sai2Primitives::JointTask>(robot);
    joint_task->disableInternalOtg();
    VectorXd q_desired = robot->q();
    VectorXd q_initial = robot->q();
    VectorXd q_flight = robot->q();
    VectorXd q_land = robot->q();

    VectorXd q_joints = q_initial.tail(12);
    q_joints << 0, M_PI/4, -M_PI/2, 0, M_PI/4, -M_PI/2, 0, M_PI/4, -M_PI/2, 0, M_PI/4, -M_PI/2;



    Vector3d forceFL;
    Vector3d forceFR;
    Vector3d forceRL;
    Vector3d forceRR;
    // cout << "this is q joints " << q_joints;
    // q_desired.tail(12) = q_joints;

    joint_task->setGains(400, 40, 0); // 18x1 vector for each joint DoF in RADIANS
    // first 6 elements are underactuated: 3 prismatic, 3 rotation. other 12 are joints rotation angles.
    // q_desired.tail(12).setZero();
    // q_desired(4) = M_PI/4.0;
    // q_desired.head(6).setZero();
    q_desired(2) = q_desired(2)+0.05;
    // q_desired(17) = -M_PI/2;
    // cout << "This is q_desired: " << q_desired; 
    // cout << "This is robot body position:" << robot->positionInWorld("body", Vector3d(0, 0, 0));
    // Vector3f eulerAng = robot->rotation("body",Vector3d(0,0,0,));
    // cout << "This is robot body orientation" << robot->rotationInWorld("body", Vector3d(0,0,0));

    joint_task->setGoalPosition(q_desired);
    // cout << q_desired;
    // get starting poses
    std::vector<Affine3d> primary_starting_pose;
    for (int i = 0; i < primary_control_links.size(); ++i)
    {
        Affine3d current_pose; // WHAT IS AFFINE3D
        current_pose.translation() = robot->position(primary_control_links[i], primary_control_points[i]);
        // translation spits out 12x1--- xyz position of each link control point?
        current_pose.linear() = robot->rotation(primary_control_links[i]);
        // translation spits out 12x3: orientation of each link? but there's only 4 links?
        //  cout << primary_control_links[i];
        //  cout << current_pose.linear();
        primary_starting_pose.push_back(current_pose); // appends current pose to primary starting pose
    }
    // cout << primary_starting_pose[0].translation()[0];
    // cout << primary_starting_pose[0].translation()(1);
    // cout << primary_starting_pose[0].translation()[2];
    float heightThresh = 0.08;

    bool safeToExtend = false;

    auto COM_task = std::make_shared<Sai2Primitives::ComMotionTask>(robot, "body", Affine3d::Identity()); // CREATING COM MOTION FORCE TASK
    COM_task->disableInternalOtg();                                                                       // disabling internal trajectory generation? what for?
    COM_task->setPosControlGains(400, 40, 0);
    COM_task->setOriControlGains(400, 40, 0);
    Vector3d robotCOM = robot->comPosition(); // position of CoM in robot frame
    // cout << robotCOM << "\n";
    Vector3d comPos;
    // cout << COM_task->getCurrentPosition();
    // com_motion_task

    std::vector<Vector3d> leg_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};
    // std::vector<Vector3d> test_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

    // create a loop timer
    runloop = true;
    double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq, 1e6);
    // double startLaunchT;


    int counter = 0;

    while (runloop)
    {
        timer.waitForNextLoop();
        const double time = timer.elapsedSimTime();
        // cout << time << "\n";
        //  state = redis_client.getInt(KEYBOARD_KEY); //STATE_KEY IN REDIS KEYS--string stateKey = whatever

        // update robot 1
        robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
        robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
        forceFL = redis_client.getEigen(FORCE_SENSOR_KEY_FL);
        forceFR = redis_client.getEigen(FORCE_SENSOR_KEY_FR);
        forceRL = redis_client.getEigen(FORCE_SENSOR_KEY_RL);
        forceRR = redis_client.getEigen(FORCE_SENSOR_KEY_RR);
        
        // cout<<"This is x force " << forceFL(0) << "\n";
        // cout<<"This is y force " << forceFL(1) << "\n";
        // cout<<"This is z force " << forceFL(2) << "\n";

        robot->updateModel();

        body_rot = robot->rotationInWorld(body_name);
        body_pos = robot->positionInWorld("body", Vector3d(0, 0, 0)); // position of body origin
        body_vel = robot->linearVelocityInWorld("body", Vector3d(0, 0, 0)); // velocity of body origin
        comPos = COM_task->getCurrentPosition();                      // IN WORLD FRAME
        

        


        bool safeToExtend = false;
        // cout << state;
        if (state == POSTURE)
        { // initial neutral position FULLY ACTUATED
            // update task model
            // joint_task->reInitializeTask(); // set joint tasks to current task

            N_prec.setIdentity();                // previous nullspace = all ones == takes priority 1 since in all spaces
            joint_task->updateTaskModel(N_prec); // sets joint task as highest priority

            command_torques = joint_task->computeTorques();
            // cout << q_desired;

            if ((robot->q() - q_desired).norm() < 5e-1)
            { // if error between joint angles and desired angles is <error
                // cout << "returned to posture" << endl;

                for (auto name : primary_control_links)
                {
                    primary_tasks[name]->reInitializeTask(); // sets desired/goal states to current position
                }
                joint_task->reInitializeTask(); // set joint tasks to current task
                // cout << joint_task; //*************************************TRY PRINTING FEET LOCATION TO SEE INITIAL VAUE

                state = FEET_FIXED_MOTION;
                // q_desired = robot->q();
                // cout << q_desired << "\n";
            }
        }
        else if (state == FEET_FIXED_MOTION)
        { // crouching
            // get underactuation matrix


            if(startToggleFeetFixedMotion){
                startTime = time;
                initialPos = body_pos;
                initialVel = body_vel;
                startToggleFeetFixedMotion = false;
            }

            // creates nullspace of feet matrix so that feet don't move
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof()); // jacobian of feet contacts. 3 translation directions * 4 feet
            for (int i = 0; i < 4; ++i)
            {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr); // nullspace of legs, all other motions outside of feet
            MatrixXd UNr = U * Nr;                    // all robot joints projected across feet null space--now UNr motions won't effect feet location
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() *
                               (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();

            // update body task
            N_prec.setIdentity();
            body_task->updateTaskModel(N_prec);
            // COM_task->updateTaskModel(N_prec);

            N_prec = body_task->getTaskAndPreviousNullspace();
            // N_prec = COM_task->getTaskAndPreviousNullspace();

            // redundancy completion
            joint_task->updateTaskModel(N_prec);

            // joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();


            goalPos = Vector3d(0,0,0.25);
            goalVel = Vector3d(0, 0, 0);

            // traj = Vector3d(0,0,0.2);



            traj = getTraj("linearPos", startTime, time, 0.5, initialPos, initialVel, goalPos, goalVel);

            body_task->setGoalPosition(traj);//GOAL POSITION SETTING---- in robot frame?
            body_task->setGoalOrientation(neutralOrientation);







            // Vector3d goalPos = Vector3d(0, 0, 0.25);
            // cout << "This is body pos Z: " << body_pos(2) << "\n";
            // cout << "This is COM pos Z: " << comPos(2) << "\n";
            // cout << "This is comPosition(): " << robot->comPosition()(2) << "\n";
            
            // cout << "This is world goal pos Z: " << goalPos(2) << "\n";
            // cout << "This is body pos Z in robot frame " << robot->position("body", Vector3d(0, 0, 0))[2] << "\n";
            if (abs(body_pos(2) - goalPos(2)) > 1e-2)
            {
                // if (abs(body_pos(2) - goalPos(2)) > 1e-2 )  { //body position
                body_task->setGoalPosition(body_pos + Vector3d(0, 0, -0.07));// 0.05- Vector3d(0,0,(body_pos(2)-goalPos(2))));//GOAL POSITION SETTING
                // COM_task->setGoalPosition(comPos + Vector3d(0, 0, -0.05)); // GOAL POSITION SETTING---- in robot frame?

                // Vector3d zeroAng = Vector3d::Zero();

                body_task->setGoalOrientation(neutralOrientation);//GOAL ORIENTATION SETTING
                // COM_task->setGoalOrientation(neutralOrientation);
            }
            // else
            // {
            //     // body_task->setGoalPosition(Vector3d(0,0,body_pos(2)));
            //     // body_task->setGoalOrientation(neutralOrientation);
            // }

            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            // command_torques += COM_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector(); // compute joint task torques if DOF isn't filled

            command_torques = U.transpose() * UNr_bar.transpose() * command_torques; // project underactuation

            // resrict the command torques within the specified limits

            command_torques = command_torques.cwiseMin(torque_limits_max).cwiseMax(torque_limits_min);

            //cout << q_desired;
            // cout << "This is COM z: " << comPos(2) << "\n";
            // cout << "This is goal z: " << goalPos(2) << "\n";
            // cout << body_pos(2)-goalPos(2);
            if (abs(body_pos(2) - goalPos(2)) < 2e-2)
            {
                // cout << "crouched" << "\n";
                safeToExtend = false;
                for (int i = 0; i < 4; ++i)
                {
                    leg_points[i] = robot->position(primary_control_links[i], primary_control_points[i]);
                    // cout << "This is leg velocity: "  << leg_points[0](2) << "\n";
                }
                // cout << "switching state to launch \n";
                body_task->setOriControlGains(800,40,0); //800 40
                body_task->setPosControlGains(660,60,0);//tweak 600 60
                state = LAUNCH;
            }
        }
        else if (state == LAUNCH)
        {
            // check if legs have moved
            // std::vector<Vector3d> test_points = {Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)};

            // // for (int i = 0; i < 4; ++i)
            // // {
            // //     test_points[i] = robot->positionInWorld(primary_control_links[i], primary_control_points[i]); // position check
            // //     // double testZ = test_points[i]

            // //     // test_points[i] = robot->linearVelocityInWorld(primary_control_links[i],primary_control_points[i]); // velocity check
            // //     // cout << "This is test leg position in world: "  << test_points[i](2) << "\n";
            // //     // cout << "This is height thresh: " << heightThresh << "\n";
            // //     // cout << "This is prev leg: "  << leg_points[0](2) << "\n";
            // // }
            // position check on leg positions
            // if (test_points[0](2) > heightThresh && test_points[1](2) > heightThresh && test_points[2](2) > heightThresh && test_points[3](2) > heightThresh)
            // {
            //     // if (test_points[i](2)>0.1)  {
            //     state = FLIGHT;
            //     cout << "switched to flight" << "\n";
            //     // joint_task->setGains(0, 40, 0);
            //     joint_task->setGains(200, 40, 0); // 18x1 vector for each joint DoF in RADIANS
            //     // COM_task->setOriControlGains(400, 40, 0);
            //     body_task->setOriControlGains(400,40,0);
            //     // cout << "updated gains" << "/n";
            //     // cout << robot->q();
            //     continue;
            // }
            if (forceFL(2) >=-0.05 && forceFR(2) >=-0.05 && forceRL(2) >=-0.05 && forceRR(2) >=-0.05)  {


            // if (body_vel(2) >= 3.5)  { //3.7
                state = FLIGHT;
                // cout << "switched to flight" << "\n";
                // joint_task->setGains(0, 40, 0);
                joint_task->setGains(200, 40, 0); // 18x1 vector for each joint DoF in RADIANS
                COM_task->setOriControlGains(400, 40, 0);
                // body_task->setOriControlGains(800,40,0);
                // cout << "updated gains" << "/n";
                // cout << robot->q();
                q_flight = robot->q();


                continue;
            }

            // if (state == FLIGHT)  {
            //     q_flight = robot->q();
            //     // cout <<"switched to flight" << "\t" << state << "\n";
            // }

            // get underactuation matrix
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof()); // jacobian of feet contacts. 3 translation directions * 4 feet
            for (int i = 0; i < 4; ++i)
            {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr); // nullspace of legs, all other motions outside of feet
            MatrixXd UNr = U * Nr;                    // all robot joints projected across feet null space--now UNr motions won't effect feet location
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() *
                               (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();


            Matrix3d randomMatDiag = Matrix3d::Zero();
            randomMatDiag << cos(M_PI/6.4), 0.0, sin(M_PI/6.4), 0, 1, 0, -sin(M_PI/6.4), 0, cos(M_PI/6.4);
            // update body task
            N_prec.setIdentity();
            // cout << N_prec.rows();
            // body_task->updateTaskModel(N_prec);
            COM_task->updateTaskModel(N_prec);
            // N_prec = body_task->getTaskAndPreviousNullspace();
            N_prec = COM_task->getTaskAndPreviousNullspace();

            // redundancy completion
            joint_task->updateTaskModel(N_prec);

            // -------- set task goals and compute control torques
            command_torques.setZero();

            // body_task->setGoalPosition(body_pos + Vector3d(0, 0, 0.1 * sin(2 * M_PI * time)));//original sinusoid vibration.
            // body_task->setGoalPosition(body_pos + Vector3d(0,0,0.4));//GOAL POSITION SETTING needs tweaking
            // body_task->setGoalLinearVelocity(Vector3d(0, 0, 0.5));
            // body_task->setGoalOrientation(neutralOrientation);
            COM_task->setGoalPosition(comPos + Vector3d(0, 0, 0.2)); //0.2 GOAL POSITION SETTING needs tweaking
            COM_task->setGoalLinearVelocity(Vector3d(0, 0, 0.7));//0.7
            // cout << "jumping" << "\n"; 
            COM_task->setGoalOrientation(randomMatDiag);//GOAL ORIENTATION SETTING

            // command_torques +=  body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            command_torques += COM_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector(); // compute joint task torques if DOF isn't filled

    
            command_torques = U.transpose() * UNr_bar.transpose() * command_torques; // project underactuation
            command_torques = command_torques.cwiseMin(torque_limits_max).cwiseMax(torque_limits_min);
            
            // else if (state == FLIGHT)
            // {
            //     command_torques = U.transpose() * command_torques; // project underactuation
            //     command_torques = command_torques.cwiseMin(torque_limits_max).cwiseMax(torque_limits_min);
            // }

            // double desired_launch_vel = 0.5;
            // if (abs(body_vel[2]-desired_launch_vel) < 0.01)  {
            //     cout << "launched";
            //     // state = FLIGHT;
            //
            // }
        }
        else if (state == FLIGHT)
        {

            if (forceFL(2) < -0.1 && forceFR(2) < -0.1 && forceRL(2) < -0.1 && forceRR(2) < -0.1)  {
                state = FALL;
                // cout << "landing" << "\n"; 
                body_task->setPosControlGains(600,185,0);
                body_task->setOriControlGains(600,185,0);
                joint_task->setGains(350,100,0);
                COM_task->setPosControlGains(400,40,0);
                COM_task->setOriControlGains(400,40,0);  
                q_land = robot->q();
                // counter++;              
                continue;
            }
            // body_task->setOriControlGains(400,40,0);
            // joint_task->setGains(800, 100, 0); // 18x1 vector for each joint DoF in RADIANS

            // update primary task model
            // N_prec.setIdentity();
            // cout << "in flight" << "\n";

            //***********body nullspace computation
            // MatrixXd bodyJac = robot->Jv(body_name,body_pos);
            // MatrixXd bodyNull =robot->nullspaceMatrix(bodyJac);
            // MatrixXd U_bodyNull = U*bodyNull;
            // MatrixXd U_bodyNull_preInv = U_bodyNull*robot->MInv()*bodyNull.transpose();
            // MatrixXd U_bodyNull_bar = robot->MInv()*U_bodyNull.transpose() * \
            //                         (U_bodyNull_preInv).completeOrthogonalDecomposition().pseudoInverse();

            // body_pos = robot->position("body", Vector3d(0, 0, 0));
            // body_vel = robot->linearVelocity(body_name,pos_in_body);
            // MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            // MatrixXd UNr_bar = robot->MInv() * UNr.transpose() * \
            //                         (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();

            // cout << q_desired-robot->q() << "\n";

            //**********************************
            // for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
            //     it->second->updateTaskModel(N_prec); // set priority on primary tasks--it = iteration, second = task object
            //     // N_prec = it->second->getTaskAndPreviousNullspace();
            // }

            // for (auto it = primary_tasks.begin(); it != primary_tasks.end(); ++it) {
            //     // it->second->updateTaskModel(N_prec); // set priority on primary tasks--it = iteration, second = task object
            //     N_prec = it->second->getTaskAndPreviousNullspace(); //update all N_prec with primary priorities
            //     body_task->updateTaskModel(N_prec); // update body task to be in null space of ith primary task
            // }

            // N_prec = body_task->getTaskAndPreviousNullspace();

            // // redundancy completion
            // joint_task->updateTaskModel(N_prec);

            // // for (auto name : primary_control_links) {
            // //     primary_tasks[name]->updateTaskModel(N_prec);

            // // }

            // // for (auto name : primary_control_links) {
            // //     N_prec = N_prec * primary_tasks[name]->getTaskAndPreviousNullspace();

            // // }
            // // N_prec = joint_task->getTaskAndPreviousNullspace(); //HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
            // // body_task->updateTaskModel(N_prec);
            // // redundancy completion

            // //we want joint task now to be highest priority
            // // -------- set task goals and compute control torques

            //******************************************************************************
            // int i = 0;
            // for (auto name : primary_control_links) {
            //     // primary_tasks[name]->setGoalPosition(primary_starting_pose[i].translation());
            //     Vector3d leg_position = Vector3d(primary_starting_pose[i].translation());
            //     // leg_position.z() =
            //     // Vector3d leg_position = Vector3d(primary_starting_pose[i].translation()(0), primary_starting_pose[i].translation()(1),body_pos(2)-0.3 );
            //     primary_tasks[name]->setGoalPosition(leg_position);
            //     // cout << body_pos(2);

            //     // primary_tasks[name]->setGoalOrientation();
            //     command_torques += primary_tasks[name]->computeTorques();
            //     ++i;
            // }

            // body_task->setGoalOrientation(initial_rot);
            // command_torques += joint_task->computeTorques();
            // q_flight(7) = 0;
            // q_flight(10) = 0;
            // q_flight(13) = 0;
            // q_flight(16) = 0;
            q_flight << robot->q().head(6), q_joints;
            // cout << q_joints << "\n";
            command_torques.setZero();
            N_prec.setIdentity();

            // cout << "Current Body Z in World " << body_pos(2) << "\n";
            // if (comPos(2) > 0.5)
            // { // if safe to extend, body priority to original position
                // safeToExtend = true;
                // cout << "safe to extend reached";
                // body_task->updateTaskModel(N_prec);
                joint_task->updateTaskModel(N_prec);

                // N_prec = body_task->getTaskAndPreviousNullspace();
                N_prec = joint_task->getTaskAndPreviousNullspace();

                // joint_task->updateTaskModel(N_prec);
                body_task->updateTaskModel(N_prec);
                // COM_task->updateTaskModel(N_prec);

                /****************************************************
                set joint task to partial such that first 6 aren't being used anymore TO DOOOOOOOOOOO
                decide on a new q desired--multiply initial q_)desired by underactuation to get underactuated q's*/
            Matrix3d randomMatDiag = Matrix3d::Zero();
            randomMatDiag << cos(-M_PI/6), 0.0, sin(-M_PI/6), 0, 1, 0, -sin(-M_PI/6), 0, cos(-M_PI/6);
                joint_task->setGoalPosition(q_flight); // set joint task to initial joint position
                body_task->setGoalOrientation(neutralOrientation);
                // COM_task->setGoalOrientation(neutralOrientation);
                // cout << "joint priority" << "\n";
            // }
            // else
            // { /// prioritize COM orientation
            //     safeToExtend = false;

            //     body_task->updateTaskModel(N_prec);
            //     // COM_task->updateTaskModel(N_prec);

            //     N_prec = body_task->getTaskAndPreviousNullspace();
            //     // N_prec = COM_task->getTaskAndPreviousNullspace();

            //     joint_task->updateTaskModel(N_prec);

            //     joint_task->setGoalPosition(q_flight); // set joint task to initial joint position
            //     body_task->setGoalOrientation(neutralOrientation);
            //     // COM_task->setGoalOrientation(neutralOrientation);

            //     // cout << "body priority" << "\n";
            // }

            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            // command_torques += COM_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector(); // compute joint task torques if DOF isn't filled
            // command_torques =  command_torques;                                                                                   // project robot underactuation matrix
            command_torques.head(6).setZero();
            command_torques = command_torques.cwiseMin(torque_limits_max).cwiseMax(torque_limits_min);
            // command_torques.setZero();
        } else if (state == FALL) {
            MatrixXd Jr = MatrixXd::Zero(3 * 4, robot->dof()); // jacobian of feet contacts. 3 translation directions * 4 feet
            for (int i = 0; i < 4; ++i)
            {
                Jr.block(3 * i, 0, 3, robot->dof()) = robot->Jv(primary_control_links[i], primary_control_points[i]);
            }
            MatrixXd Nr = robot->nullspaceMatrix(Jr); // nullspace of legs, all other motions outside of feet
            MatrixXd UNr = U * Nr;                    // all robot joints projected across feet null space--now UNr motions won't effect feet location
            MatrixXd UNr_pre_inverse = UNr * robot->MInv() * UNr.transpose();
            MatrixXd UNr_bar = robot->MInv() * UNr.transpose() *
                               (UNr_pre_inverse).completeOrthogonalDecomposition().pseudoInverse();


            command_torques.setZero();
            // cout<< "in fall" << "\n";
            N_prec.setIdentity();
            body_task->updateTaskModel(N_prec);

            N_prec = body_task->getTaskAndPreviousNullspace();
            joint_task->updateTaskModel(N_prec);

                // COM_task->updateTaskModel(N_prec);

                /****************************************************
                set joint task to partial such that first 6 aren't being used anymore TO DOOOOOOOOOOO
                decide on a new q desired--multiply initial q_)desired by underactuation to get underactuated q's*/
            Matrix3d randomMatDiag = Matrix3d::Zero();
            randomMatDiag << cos(-M_PI/6), 0.0, sin(-M_PI/6), 0, 1, 0, -sin(-M_PI/6), 0, cos(-M_PI/6);



            joint_task->setGoalPosition(q_initial); // set joint task to initial joint position
            body_task->setGoalOrientation(neutralOrientation);
            body_task->setGoalPosition(q_initial.head(3));
            // joint_task->reInitializeTask();
            command_torques += body_task->computeTorques() + joint_task->computeTorques() + robot->coriolisForce() + robot->jointGravityVector();  // compute joint task torques if DOF isn't filled
            command_torques = U.transpose()*UNr_bar.transpose() * command_torques; // project underactuation

            command_torques.head(6).setZero();
            command_torques = command_torques.cwiseMin(torque_limits_max).cwiseMax(torque_limits_min);


            if (body_vel.norm() < 0.02)  {
                state = POSTURE;
                body_task->setPosControlGains(400,40,0);
                body_task->setOriControlGains(40,40,0);
                joint_task->setGains(400,40,0);
                COM_task->setPosControlGains(400,40,0);
                COM_task->setOriControlGains(400,40,0);  
            }

        }

        // execute redis write callback
        redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    }

    timer.stop();
    cout << "\nSimulation loop timer stats:\n";
    timer.printInfoPostRun();
    redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques); // back to floating
}