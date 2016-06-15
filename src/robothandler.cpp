#include "robothandler.h"

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
}

int Robothandler::connection()
{		
    if(!m_robotConnector->connectRobot()) {
        ArLog::log(ArLog::Terse, "Ax-Example: Could not connect to the robot.");
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
    ArLog::log(ArLog::Normal, "Ax-Example: Connected to robot.");
    return 0;
}

int Robothandler::disconnection() {
	ArLog::log(ArLog::Normal,"Ending robot thread");
	m_robot->stopRunning();
	//wait for the thread to stop
	m_robot->waitForRunExit();
	//exiting
	Aria::exit(0);
	return 0;
}

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
	ArUtil::sleep(3000);
	
	//ending robot thread
	//m_robot->stopRunning();
	//wait for the thread to stop
	//m_robot->waitForRunExit();		
}

std::vector<double> Robothandler::getPose() {	
	m_robot->lock();
	std::vector<double> outPose;
	outPose.push_back(m_robot->getX());	
	outPose.push_back(m_robot->getY());
	outPose.push_back(m_robot->getTh());
	m_robot->unlock();
	ArLog::log(ArLog::Normal,"Ax-example: pose=(%.2f,%.2f,%.2f)",m_robot->getX(),m_robot->getY(),m_robot->getTh());
	return outPose;
}

void Robothandler::activateSonar()
{
    //declare sonar
    m_robot->addRangeDevice(m_sonar);
    m_robot->runAsync(true);
}

void Robothandler::activateLaser()
{
    //try to connect to laser. if fail, warn & continue
    if(!m_laserConnector->connectLasers()) {
        ArLog::log(ArLog::Normal, "Warning: unable to connect lasers. Using only sonar");
    }
}

void Robothandler::makeKeyHandler()
{
    //Make a key handler, so that escape will shut down the program
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    m_robot->attachKeyHandler(&keyHandler);
    printf("You may press escape to exit\n");
}

int Robothandler::wander()
{
	ArLog::log(ArLog::Normal,"Ax-example: Starting wondering sequence...");
    //turn on the motors
    m_robot->enableMotors();
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

void Robothandler::followSquare() {
	getPose();
	m_robot->enableMotors();
	for(int i=0;i<4;i++) {
		//go forward 100mm/s for 3s
		m_robot->lock();		
		m_robot->setRotVel(0);
		m_robot->setVel(300);
		m_robot->unlock();
		ArUtil::sleep(3000);
		
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
