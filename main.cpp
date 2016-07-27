// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h> // arena
#include <ode_robots/passivesphere.h>  // passive balls
#include <ode_robots/playground.h>
#include <ode_robots/operators.h>
#include <ode_robots/boxpile.h>

// controller
#include <selforg/invertmotorspace.h>
#include <selforg/sinecontroller.h>

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/speedsensor.h>
#include "sphere.h" 
#include "axisorientationsensor.h"

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
  Sphererobot3MassesMod* sphere1;

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    // initialization
    // - set global noise to 0.1
    global.odeConfig.setParam("noise",0.1);
    //  global.odeConfig.setParam("gravity", 0); // no gravity
    
    if (false) {
      Boxpile* boxpile = new Boxpile(odeHandle, osgHandle,osg::Vec3(20.0, 20.0, 1.0), 100, 1,
                                     osg::Vec3(1, 1, 0.07), osg::Vec3(.4, .4, 0.02));
      boxpile->setColor("wall");
      boxpile->setPose(ROTM(M_PI/5.0,0,0,1)*TRANSM(20 ,0 , 0));
      global.obstacles.push_back(boxpile);
    }

    // stacked Playgrounds
    double scale = 10;
    double heightoffset = 2;
    double height = 1;
    for (int i=0; i< 1; i++){
      //auto playground = new Playground(odeHandle, osgHandle,
      //                            osg::Vec3((4+4*i)*scale, 1, heightoffset +i*height), 1, false);
      //      playground->setColor(Color((i & 1) == 0,(i & 2) == 0,(i & 3) == 0,0.3f));
      // playground->setTexture("Images/really_white.rgb");
      //playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
      //global.obstacles.push_back(playground);
    }

   TerrainGround* terrainground =
          new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
                            //                        "terrains/dip128_flat.ppm","terrains/dip128_flat_texture.ppm", 
                            "terrains/dip128.ppm","terrains/dip128_texture.ppm",
                            20, 20, 1.3 /* height */, OSGHeightField::Red);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground);
        //addRobot(odeHandle, osgHandle, global, 3); 
    

    // Spherical Robot with axis (gyro) sensors:
    // - get default configuration for robot
    // - create pointer to spherical robot (with odeHandle, osgHandle and configuration)
    // - place robot (unfortunatelly there is a type cast necessary, which is not quite understandable)
    Sphererobot3MassesModConf conf = Sphererobot3MassesMod::getDefaultConf();
    conf.addSensor(new AxisOrientationSensorMod(AxisOrientationSensorMod::ZProjection));
    conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel)); 

    // regular behaviour  
    conf.irAxis1 = false; 
    conf.irAxis2 = false; 
    conf.irAxis3 = false;     
    conf.irsensorscale = 2; 
    conf.irInverseValue = true; 
    conf.motorsensor = false;
    conf.motor_ir_before_sensors = true; 
    conf.diameter = 1.0;
    conf.pendularrange = 0.2;
    sphere1 = new Sphererobot3MassesMod ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)),
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
    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise );

    // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
    // initialize pointer with controller, robot and wiring
    // push agent in globel list of agents
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , sphere1 , wiring );

    // the following line will enable a position tracking of the robot, which is written into a file
    agent->setTrackOptions(TrackRobot(true, true, true, false, "Sphere_zaxis", 20));
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


