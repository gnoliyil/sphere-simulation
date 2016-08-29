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
#include "conceptordep.h"

#include <cmath>
#include <algorithm>
#include <random>

using namespace lpzrobots;

/// Class to wrap a sensor and feed its delayed values.
class DelaySensor : public Sensor, public Configurable {
public:
    /// the sensor is supposed to be initialized (or gets initialized elsewhere before sense etc. is called)
    DelaySensor(std::shared_ptr<Sensor> src, int delay = 1)
            : src(src), buffer(100), delay(delay), t(0) {
        assert(src);
        addParameter("delay", &this->delay, 0, 100, "delay in steps of sensor value");
        SensorMotorInfo smi = src->getBaseInfo();
        setBaseInfo(smi.changename("Delayed " + smi.name));
        setNamingFunc(src->getNamingFunc());
    }

    void init(Primitive *own, Joint *joint = 0) {
        // src->init(own, joint); // we do not want to init the sensor again. // debateable
    }

    bool sense(const GlobalData &globaldata) {
        bool res = src->sense(globaldata);
        buffer[t] = src->getList();
        t++;
        return res;
    }

    int getSensorNumber() const {
        return src->getSensorNumber();
    }

    std::list<sensor> getList() const {
        return buffer[t - 1 - delay];
    }

protected:
    std::shared_ptr<Sensor> src;
    RingBuffer<std::list<sensor> > buffer;
    int delay;
    long t;
};

static int speed_to_state(Sphererobot3MassesMod* sphere, ConceptorDEP *dep)
{
    double speed = sphere->getSpeed().x;
    int current_conceptor = dep->getTargetConceptorId();

    double max_sp = 0;
    double min_sp = 3;
    int segments = 30;
    int target = (int)((speed - min_sp) / (max_sp - min_sp) * segments);

    if (target < 0)
        target = 0;
    if (target >= segments)
        target = segments - 1;

    return target + current_conceptor * segments;
}

static double reward(Sphererobot3MassesMod* sphere)
{
    //double target_speed = -1;
    //sphere->getSpeed().x
    // -|speed|
    // return -fabs((sphere->getSpeed().x - 2) * (sphere->getSpeed().x - 2));
    return -fabs(sphere->getSpeed().x);
}

template <int N>
static double exp_lambda(double x)
{
    return exp(x / N);
}

static int explore(matrix::Matrix & Q, int state, double random_rate)
{
    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> uniform(0, 1);

    matrix::Matrix sub = Q.row(state);

    int choice;

    if (uniform(generator) < random_rate){
        sub = sub.map(exp_lambda<3>);
        sub *= 1.0 / sub.elementSum();
        std::list<double> sub_list = sub.convertToList();
        std::discrete_distribution<int> distribution(sub_list.begin(), sub_list.end());
        choice = distribution(generator);

//        std::list<double> sub_list = sub.convertToList();
//        auto x = std::max_element(sub_list.begin(), sub_list.end());
//        auto y = sub_list.begin();
//        for (int i = 0; i < sub_list.size(); i++) {
//            if (y == x) {
//                choice = i;
//                break;
//            }
//            y ++;
//        }
    } else {
        choice = (int) (uniform(generator) * sub.getN());
    }

    return choice;
}


class ThisSim : public Simulation {
public:
    AbstractController *controller;
    Sphererobot3MassesMod *sphere1;
    ConceptorDEP* dep;
    OdeAgent *agent;
    One2OneWiring *wiring;
    std::vector<matrix::Matrix> conceptors;
    matrix::Matrix W;
    matrix::Matrix W_out;
    matrix::Matrix W_bias;

    bool first;
    bool first_rl;

    double eta = 0.25;
    double learn_gamma = 0.99;

    // reinforcement learning part
    matrix::Matrix* Q;
    int tot_action = 6;
    // TODO: tot_action = 6 here, should be 12
    int tot_state = 30 * 6;
    double (*reward_f)(Sphererobot3MassesMod* sphere);
    int (*explore_f)(matrix::Matrix & Q, int state, double random_rate);
    int (*to_state)(Sphererobot3MassesMod* sphere, ConceptorDEP *dep);
    double total_score;
    int tot_iter;
    int cur_iter;
    int cur_state;
    int cur_action;
    double cur_reward;
    int bef_state;

    ThisSim(){
        first = true;

        // initialize Q
        RandGen randGen;
        randGen.init(123456);

        Q = new matrix::Matrix(tot_state, tot_action);
        double *temp = new double [tot_action * tot_state];
        for (int i = 0; i < tot_action * tot_state; i++)
            temp[i] = 0.05 * randGen.rand(); // - 0.5 + cconf.initialXMean;
        Q->set(temp);
        delete[] temp;

        tot_iter = 500;
        cur_iter = 0;

        reward_f = reward;
        explore_f = explore;
        to_state = speed_to_state;

        total_score = 0;

        first_rl = true;
    }

    ~ThisSim() {
        delete Q;
    }

    void loadConceptors() {
        //--------------------------------
        // LOAD CONCEPTORS FROM FILE
        //--------------------------------
        FILE *fp = fopen("conceptor.dat", "r");
        for (int i = 0; i < 6; i++) {
            matrix::Matrix c;
            c.read(fp, false);
            conceptors.push_back(c);
        }
        W.read(fp, false);
        W_out.read(fp, false);
        W_bias.read(fp, false);
    }

    // starting function (executed once at the beginning of the simulation loop)
    void start(const OdeHandle &odeHandle, const OsgHandle &osgHandle, GlobalData &global) {
        setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
        // initialization
        // - set global noise to 0.1
        global.odeConfig.setParam("noise", 0.1);
        //  global.odeConfig.setParam("gravity", 0); // no gravity

        if (first) {
            // stacked Playgrounds
            double scale = 200;
            double heightoffset = 2;
            double height = 1;
            for (int i = 0; i < 1; i++) {
                auto playground = new Playground(odeHandle, osgHandle,
                                                 osg::Vec3((4 + 4 * i) * scale, 1, heightoffset + i * height), 1,
                                                 false);
                playground->setColor(Color((i & 1) == 0, (i & 2) == 0, (i & 3) == 0, 0.3f));
                playground->setTexture("Images/really_white.rgb");
                playground->setPosition(osg::Vec3(0, 0, 0)); // playground positionieren und generieren
                global.obstacles.push_back(playground);
            }
        }


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
        sphere1 = new Sphererobot3MassesMod(odeHandle, osgHandle.changeColor(Color(1.0, 0.0, 0)),
                                            conf, "Sphere1", 0.2);
        ((OdeRobot *) sphere1)->place(Pos(0, 0, 0.1));

        global.configs.push_back(sphere1);

        if (first)
            loadConceptors();

        //--------------------------------
        // CREATE DEP CONTROLLER HERE
        //--------------------------------

        DEPConf pc = DEP::getDefaultConf();
        pc.initFeedbackStrength = 0.0;
        pc.calcEigenvalues = true;

        ConceptorDEPConf cpc = ConceptorDEP::getDefaultConf();
        cpc.initialXDeviation = 0.5;
        cpc.initialXMean = 0.5;
        cpc.lambdaC = 0.5;
        cpc.lambdaH = 1;
        cpc.transitionTime = 5;

        dep = new ConceptorDEP(conceptors, W, W_out, W_bias, cpc, pc);
        dep->setParam("lambdaC", 0.0);
        dep->setParam("lambdaH", 1.0);
        dep->setParam("epsM", 0);
        dep->setParam("epsh", 0.05);
        dep->setParam("synboost", 1.4);
        dep->setParam("urate", 0.05);
        dep->setParam("indnorm", 0); // 0 is global normalization
        dep->setParam("timedist", 4);
        controller = dep;
        global.configs.push_back(controller);

        // create pointer to one2onewiring which uses colored-noise
        wiring = new One2OneWiring(new ColorUniformNoise);

        // create pointer to agent (plotoptions is provided by Simulation (generated from cmdline options)
        // initialize pointer with controller, robot and wiring
        // push agent in globel list of agents
        agent = new OdeAgent(global);
        agent->init(controller, sphere1, wiring);

        // the following line will enable a position tracking of the robot, which is written into a file
        // agent->setTrackOptions(TrackRobot(true, true, true, false, "Sphere_zaxis", 20));
        global.agents.push_back(agent);

        // display all parameters of all configurable objects on the console
        dBodyAddForce(sphere1->getMainPrimitive()->getBody(), 50, 0, 0);

    }

    /** is called if a key was pressed.
        For keycodes see: osgGA::GUIEventAdapter
        @return true if the key was handled
    */
    virtual bool command(const OdeHandle &, const OsgHandle &, GlobalData &globalData,
                         int key, bool down) {
        static int down_n = 0, down_z = 0;
        if (down) { // only when key is pressed, not when released
            switch ((char) key) {
                case 'X' :
                    dBodyAddForce(sphere1->getMainPrimitive()->getBody(), 30, 0, 0);
                    break;
                case 'x' :
                    dBodyAddForce(sphere1->getMainPrimitive()->getBody(), -30, 0, 0);
                    break;
                case 'T' :
                    dBodyAddTorque(sphere1->getMainPrimitive()->getBody(), 0, 0, 3);
                    break;
                case 't' :
                    dBodyAddTorque(sphere1->getMainPrimitive()->getBody(), 0, 0, -3);
                    break;
                case 'n' :
                    down_n = 1;
                    break;
                case 'z':
                    down_z = 1;
                    break;
                default:
                    if (key >= '0' && key <= '5')
                    {
                        if (down_n)
                            ((ConceptorDEP*)controller)->switchConceptor(key - '0');
                        else if (down_z)
                            ((ConceptorDEP*)controller)->switchConceptorh(key - '0');
                        down_n = 0;
                        down_z = 0;
                        return true;
                    }
                    down_n = 0;
                    return false;
            }
            return true;
        } else return false;
    }

    bool restart(const OdeHandle &odeHandle, const OsgHandle &osgHandle, GlobalData &global)
    {
        first = false;
        global.obstacles.clear();
        global.agents.pop_back();
        global.configs.pop_back();
        global.configs.pop_back();
        printf("there left %d agents. \n", global.agents.size());
        delete agent;

        FILE *fp = fopen("reinforce.log", "a");
        fprintf(fp, "%d %.3f %.4f\n", cur_iter, global.time, total_score);
        fclose(fp);

        if (cur_iter >= tot_iter)
            return false;

        total_score = 0;
        cur_iter += 1;
        first_rl = true;

        if (cur_iter % 100 == 0)
        {
            char fn[100];
            sprintf(fn, "rl/Q%d.dat", cur_iter);
            FILE *fp = fopen(fn, "w");
            Q->store(fp);
            fclose(fp);
        }

        end(global);
        start(odeHandle, osgHandle, global);

        return true; 
    }

    virtual void addCallback(GlobalData &globalData, bool draw,
                             bool pause, bool control )
    {
        if (globalData.time > 30)
            simulation_time_reached = true;

        if (globalData.time < 10)
            return;

        if (fabs(globalData.time * 2 - round(globalData.time * 2)) < 1e-4)
        {
            double x_speed;
            if (!first_rl)
            {
                bef_state = cur_state;
                x_speed = sphere1->getSpeed().x;
                cur_state = to_state(sphere1, dep);

                std::list<double> sub = Q->row(cur_state).convertToList();
                double value = *(std::max_element(sub.begin(), sub.end()));
                Q->val(bef_state, cur_action) +=
                    eta * (cur_reward + learn_gamma * value - Q->val(bef_state, cur_action));
            } else
            {
                x_speed = sphere1->getSpeed().x;
                cur_state = to_state(sphere1, dep);
                first_rl = false;
            }

            cur_reward = reward(sphere1);
            if (globalData.time > 20)
                total_score += cur_reward;
            double random_rate = min(1, 1.5/(globalData.time - 10)) *
                                 min(1.0 / log2(cur_iter + 2), 1);
            cur_action = explore_f(*Q, cur_state, random_rate);

            if (cur_action < 6)
                dep->switchConceptor(cur_action);
            else
                dep->switchConceptorh(cur_action - 6);
            // switch between conceptors
            printf("it: %d  time: %.3f  score: %.3f  speed: %.3f  action: %d\n",
                   cur_iter, globalData.time, total_score, x_speed, cur_action);
        }
    }

    virtual void bindingDescription(osg::ApplicationUsage &au) const {
        au.addKeyboardMouseBinding("Simulation: X", "Push robot to right (positive x)");
        au.addKeyboardMouseBinding("Simulation: x", "Push robot to left (negative x)");
        au.addKeyboardMouseBinding("Simulation: T", "Spin robot counter-clockwise");
        au.addKeyboardMouseBinding("Simulation: t", "Spin robot clockwise");
    }

};

int main(int argc, char **argv) {
    ThisSim sim;
    sim.tot_iter = 50000;

    sim.setGroundTexture("Images/yellow_velour.rgb");
    return sim.run(argc, argv) ? 0 : 1;
}


