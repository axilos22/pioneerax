#include <iostream>

#include "../include/robothandler.h"

using namespace std;
/**
 * @brief Robothandler::Robothandler constructor, need the information from command line to execute properly.
 * @param argc main input
 * @param argv main input
 */
Robothandler::Robothandler(int argc, char **argv)
{
    Aria::init();
    m_parser = new ArArgumentParser(&argc,argv);
    m_parser->loadDefaultArguments();
    m_robot = new ArRobot;
    m_gyro = new ArAnalogGyro(m_robot);
    m_sonar = new ArSonarDevice();
    m_robotConnector= new ArRobotConnector(m_parser,m_robot);
    m_laserConnector=new ArLaserConnector(m_parser,m_robot,m_robotConnector);
    m_time = new ArTime();
    m_keyHandler = new ArKeyHandler();
}
/**
 * @brief Robothandler::connection try to connect with the robt.
 * @return 1 if connection failed / 0 if successfull
 */
int Robothandler::connection()
{		
    if(!m_robotConnector->connectRobot()) {
        ArLog::log(ArLog::Terse, "Ax-Example@connection: Could not connect to the robot.");
        if(m_parser->checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
            return 1;
        }
    }
    if (!Aria::parseArgs() || !m_parser->checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }
    ArLog::log(ArLog::Normal, "Ax-Example@connection: Connected to robot.");
    return 0;
}
/**
 * @brief Robothandler::disconnection end the robot thread and terminate connection with the robot.
 * @return 1 failed / 0 successfull
 */
int Robothandler::disconnection() {
    ArLog::log(ArLog::Normal,"Ax-Example@disconnection Ending robot thread");
    m_robot->stopRunning();
    //wait for the thread to stop
    m_robot->waitForRunExit();
    return 0;
}
/**
 * @brief Robothandler::getInitialData print some intial data of the robot.
 */
void Robothandler::getInitialData(){
    m_robot->enableMotors();
    //background robot processing cycle
    m_robot->runAsync(true);
    //print some data to SIP
    m_robot->lock();
    ArLog::log(ArLog::Normal,"Ax-example: pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
               m_robot->getX(),m_robot->getY(),m_robot->getTh(),m_robot->getVel(),m_robot->getBatteryVoltage());
    m_robot->unlock();

    //sleep for 3s
    ArLog::log(ArLog::Normal,"Ax-example: sleeping 3s");
    ArUtil::sleep(1000);

    //ending robot thread
    //m_robot->stopRunning();
    //wait for the thread to stop
    //m_robot->waitForRunExit();
}
/**
 * @brief Robothandler::getPose print and give back the pose of the robot.
 * @return the pose of the robot.
 */
std::vector<double> Robothandler::getPose() {	
    m_robot->lock();
    std::vector<double> outPose;
    outPose.push_back(m_robot->getX());
    outPose.push_back(m_robot->getY());
    outPose.push_back(m_robot->getTh()); //getting heading in degree
    //~ outPose.push_back(m_robot->getThRad());
    m_robot->unlock();
    ArLog::log(ArLog::Verbose,"Ax-example: pose=(%.2f,%.2f,%.2f)",m_robot->getX(),m_robot->getY(),m_robot->getTh());
    return outPose;
}
/**
 * @brief Robothandler::getPositionEigen give the position of the robot with Eigen::Vector2d format
 * @return position of the robot.
 */
Eigen::Vector2d Robothandler::getPositionEigen() {	
    m_robot->lock();
    Eigen::Vector2d position(m_robot->getX(),m_robot->getY());
    m_robot->unlock();
    return position;
}
/**
 * @brief Robothandler::getPoseEigen return the pose of the robot with an Eigen::Vector3d format
 * @return pose of the vector
 */
Eigen::Vector3d Robothandler::getPoseEigen() {
    m_robot->lock();
    //heading is given between [-180,180]
    Eigen::Vector3d pose(m_robot->getX(),m_robot->getY(),degree2rad(m_robot->getTh()));
    m_robot->unlock();
    return pose;
}
/**
 * @brief Robothandler::activateSonar try to activate sonar (if any)
 */
void Robothandler::activateSonar()
{
    m_robot->addRangeDevice(m_sonar);
}
/**
 * @brief Robothandler::activateLaser try to activate laser (if any)
 */
void Robothandler::activateLaser()
{
    //try to connect to laser. if fail, warn & continue
    if(!m_laserConnector->connectLasers()) {
        ArLog::log(ArLog::Normal, "Warning: unable to connect lasers. Using only sonar");
    }
}
/**
 * @brief Robothandler::makeKeyHandler makes a key handler (ESC by default) to cleanly exit the program
 */
void Robothandler::makeKeyHandler()
{
    //Make a key handler, so that escape will shut down the program
    Aria::setKeyHandler(m_keyHandler);
    m_robot->attachKeyHandler(m_keyHandler);
    printf("You may press escape to exit\n");
}
/**
 * @brief Robothandler::wander stack some action to make the robot wander around while avoiding obstacles
 * @return
 */
int Robothandler::wander()
{
    ArLog::log(ArLog::Normal,"Ax-example: Starting wondering sequence...");
    //turn on the motors
    //m_robot->enableMotors();
    //turn off amigobot sounds
    //m_robot->comInt(ArCommands::SOUNDTOG,0);
    //Wander core: set of actions to make wander bahavior
    ArActionStallRecover recover;
    ArActionBumpers bumpers;
    ArActionAvoidFront avoidFrontNear("Avoid front near",255,0);
    ArActionAvoidFront avoidFrontFar;
    ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
    m_robot->addAction(&recover,100);
    m_robot->addAction(&bumpers,75);
    m_robot->addAction(&avoidFrontNear,50);
    m_robot->addAction(&avoidFrontFar,49);
    m_robot->addAction(&constantVelocity,25);

    ArLog::log(ArLog::Normal,"Ax-example: Actions are stored, begin wondering in 5s");
    ArUtil::sleep(1000);
    ArLog::log(ArLog::Normal,"Ax-example: 4s");
    ArUtil::sleep(1000);
    ArLog::log(ArLog::Normal,"Ax-example: 3s");
    ArUtil::sleep(1000);
    ArLog::log(ArLog::Normal,"Ax-example: 2s");
    ArUtil::sleep(1000);
    ArLog::log(ArLog::Normal,"Ax-example: 1s");
    // wait for robot task loop to end before exiting the program
    m_robot->waitForRunExit();
    return 0;
}
/**
 * @brief Robothandler::~Robothandler destructor
 */
Robothandler::~Robothandler()
{	
    //remove allocation of all elements
    delete m_parser;
    delete m_robot;
    delete m_gyro;
    delete m_sonar;
    delete m_robotConnector;
    delete m_laserConnector;
}
/**
 * @brief Robothandler::followSquare make the robot follow a square trajectory
 */
void Robothandler::followSquare() {
    getPose();
    m_robot->enableMotors();
    //sleep 3s to be rdy for commands
    ArUtil::sleep(3000);
    for(int i=0;i<4;i++) {
        //go forward 500mm/s for 2s
        m_robot->lock();
        m_robot->setRotVel(0);
        m_robot->setVel(500);
        m_robot->unlock();
        ArUtil::sleep(2000);

        //stop
        m_robot->lock();
        m_robot->stop();
        m_robot->unlock();
        ArUtil::sleep(1000);

        //turn 90degrees
        m_robot->lock();
        m_robot->setRotVel(-30);
        m_robot->setVel(0);
        m_robot->unlock();
        ArUtil::sleep(3000);
        getPose();
    }

    m_robot->stopRunning();
    m_robot->waitForRunExit();
}
/**
 * @brief Robothandler::getTime return the current time (s) of the robot
 * @return time (in s)
 */
const ArTime* Robothandler::getTime() {
    return m_time;
}
/**
 * @brief Robothandler::resetTime reset the internal robot timer for convinience
 */
void Robothandler::resetTime() {
    //~ m_time->setSec(0);
    //~ m_time->setMSec(0);
    m_time->setToNow();
}
/**
 * @brief Robothandler::setCommand send command to the robot
 * @param v linear speed
 * @param w angular speed
 */
void Robothandler::setCommand(double v, double w) {
    m_robot->lock();
    m_robot->setVel(v);
    m_robot->setRotVel(rad2degree(w));//set in degree per second
    m_robot->unlock();
    ArUtil::sleep(30);//200ms sampling period
}
/**
 * @brief Robothandler::prepareToMove switch on the robot engines and launch the robot thread.
 */
void Robothandler::prepareToMove() {
    m_robot->enableMotors();
    //background robot processing cycle
    m_robot->runAsync(true);
}
/**
 * @brief Robothandler::rad2degree convert a values from rad to degree
 * @param radValue
 * @return
 */
double Robothandler::rad2degree(double radValue)
{
    return (radValue*180.0)/M_PI;
}
/**
 * @brief Robothandler::degree2rad convert a values from degree to rad
 * @param degValue
 * @return
 */
double Robothandler::degree2rad(double degValue)
{
    return (degValue*M_PI)/180.0;
}

	
	// limiter for close obstacles
	ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250);
	// limiter for far away obstacles
	ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 400);
	// limiter that checks IR sensors (like Peoplebot has)
	ArActionLimiterTableSensor tableLimiter;
	// limiter so we don't bump things backwards
	ArActionLimiterBackwards backwardsLimiter;	
	// the joydrive action
	ArActionJoydrive joydriveAct;
	// the keydrive action
	ArActionKeydrive keydriveAct;

void Robothandler::teleop() {

	ArLog::log(ArLog::Normal,"Start teleoperating sequence...");
	// if we don't have a joystick, let 'em know
	if (!joydriveAct.joystickInited())
	printf("Do not have a joystick, only the arrow keys on the keyboard will work.\n");
	
	// add the sonar to the robot
	m_robot->addRangeDevice(m_sonar);
	// set the robots maximum velocity (sonar don't work at all well if you're going faster)
	m_robot->setAbsoluteMaxTransVel(400);
	// enable the motor
	m_robot->enableMotors();
	
	// Add the actions, with the limiters as highest priority, then the teleop.
	// actions.  This will keep the teleop. actions from being able to drive too 
	// fast and hit something
	m_robot->addAction(&tableLimiter, 100);
	m_robot->addAction(&limiter, 95);
	m_robot->addAction(&limiterFar, 90);
	m_robot->addAction(&backwardsLimiter, 85);
	m_robot->addAction(&joydriveAct, 50);
	m_robot->addAction(&keydriveAct, 45);
	
	// Configure the joydrive action so it will let the lower priority actions
	// (i.e. keydriveAct) request motion if the joystick button is
	// not pressed.
	joydriveAct.setStopIfNoButtonPressed(false);
	
	// run the robot, true means that the run will exit if connection lost
	//~ m_robot->run(true);
	prepareToMove();
}
