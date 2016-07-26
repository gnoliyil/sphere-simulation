// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h> // arena
#include <ode_robots/passivesphere.h>  // passive balls

// controller
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>

// dep rules
#include "dep.h"

using namespace lpzrobots;

/// Class to wrap a sensor and feed its delayed values.
class DelaySensor : public Sensor, public Configurable {
public:
  /// the sensor is supposed to be initialized (or gets initialized elsewhere before sense etc. is called)
  DelaySensor (std::shared_ptr<Sensor> src, int delay=1)
    : src(src), buffer(100), delay(delay), t(0) {
    assert(src);
    addParameter("delay", &this->delay, 0, 100, "delay in steps of sensor value");
    SensorMotorInfo smi=src->getBaseInfo();
    setBaseInfo(smi.changename("Delayed " + smi.name));
    setNamingFunc(src->getNamingFunc());
  }

  void init(Primitive* own, Joint* joint = 0) {
    // src->init(own, joint); // we do not want to init the sensor again. // debateable
  }

  bool sense(const GlobalData& globaldata){
    bool res =  src->sense(globaldata);
    buffer[t]=src->getList();
    t++;
    return res;
  }

  int getSensorNumber() const {
    return src->getSensorNumber();
  }

  std::list<sensor> getList() const {
    return buffer[t-1-delay];
  }

protected:
  std::shared_ptr<Sensor> src;
  RingBuffer<std::list<sensor> > buffer;
  int delay;
  long t;
};



class ThisSim : public Simulation {
public:
  AbstractController* controller;
  Sphererobot3Masses* sphere1;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    // Spherical Robot with axis (gyro) sensors:
    // - get default configuration for robot
    // - create pointer to spherical robot (with odeHandle, osgHandle and configuration)
    // - place robot (unfortunatelly there is a type cast necessary, which is not quite understandable)
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    // regular behaviour   
    
    conf.motorsensor = true;
    conf.diameter = 1.0;
    conf.pendularrange = 0.2;
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
                                       conf, "Sphere1", 0.2);
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
    
    //  std::list<Sensor*> sas = sphere1->getSensorList();     
    //  printf("sas = %d\n", sas.size()); 
      
    //  int k=0;
    //  for(auto& sa: sas){
    //    if(k%2==0 || k%2){
    //      auto s = std::make_shared<DelaySensor>(std::make_shared<Sensor>(sa));
    //      sphere1->addSensor(s);
    //      global.configs.push_back(s.get());
    //    }
    //    k++;
    //    printf("k = %d\n", k); 
    //  } 
      
    global.configs.push_back ( sphere1 );

    // Selforg - Controller
    // create pointer to controller
    // set some parameters
    // push controller in global list of configurables
    //controller = new InvertMotorSpace(1);
    //controller->setParam("epsA",0); // model learning rate
    //controller->setParam("epsC",0); // controller learning rate
    //controller->setParam("rootE",3);    // model and contoller learn with square rooted error
    
    //--------------------------------
    // CREATE DEP CONTROLLER HERE
    //--------------------------------
    
    DEPConf pc = DEP::getDefaultConf();
    pc.initFeedbackStrength = 0.0;
    pc.calcEigenvalues = true;

    DEP* dep = new DEP(pc);
    dep->setParam("epsM",0);
    dep->setParam("epsh",0.0);
    dep->setParam("synboost",2.2);
    dep->setParam("urate",0.05);
    dep->setParam("indnorm",1); // 0 is global normalization
    dep->setParam("timedist",4);
    controller = dep;
    global.configs.push_back ( controller );
    
    // create pointer to one2onewiring which uses colored-noise
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , sphere1 , wiring );

    // the following line will enable a position tracking of the robot, which is written into a file
    //agent->setTrackOptions(TrackRobot(true, true, true, false, "Sphere_zaxis", 20));
    global.agents.push_back ( agent );

    // display all parameters of all configurable objects on the console

  }

  /** is called if a key was pressed.
      For keycodes see: osgGA::GUIEventAdapter
      @return true if the key was handled
  */
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData,
                       int key, bool down) {
    if (down) { // only when key is pressed, not when released
      switch ( (char) key ) {
      case 'X' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , 30 ,0 , 0 ); break;
      case 'x' : dBodyAddForce ( sphere1->getMainPrimitive()->getBody() , -30 , 0 , 0 ); break;
      case 'T' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , 3 ); break;
      case 't' : dBodyAddTorque ( sphere1->getMainPrimitive()->getBody() , 0 , 0 , -3 ); break;
      default:
        return false;
      }
      return true;
    } else return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: X","Push robot to right (positive x)");
    au.addKeyboardMouseBinding("Simulation: x","Push robot to left (negative x)");
    au.addKeyboardMouseBinding("Simulation: T","Spin robot counter-clockwise");
    au.addKeyboardMouseBinding("Simulation: t","Spin robot clockwise");
  }

};

int main (int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/yellow_velour.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}


