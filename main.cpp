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
using matrix::Matrix;


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

static double reward(Sphererobot3MassesMod* sphere)
{
    //double target_speed = -1;
    //sphere->getSpeed().x
    // -|speed|
    // return -fabs((sphere->getSpeed().x - 2) * (sphere->getSpeed().x - 2));
    return -fabs(sqr(sphere->getSpeed().x - 2));    git 
}

static matrix::Matrix get_state(Sphererobot3MassesMod* sphere, ConceptorDEP* dep)
{
    double sensors[6];
    sphere->getSensorsIntern(sensors, 6);
    Matrix _state = Matrix(1, 5, sensors);
    _state.val(0, 3) = sphere->getSpeed().x;
    _state.val(0, 4) = dep->getTargetConceptorId();
    return _state;
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


std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(0, 1);

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

    matrix::Matrix centroids; // (n_centroids, dim_states)
    matrix::Matrix state, activations, action_values, min_state, max_state;
    // state : 1, dim_states    activation: 1, n_centroids
    matrix::Matrix new_state, new_activations;
    matrix::Matrix theta;
    // theta : n_centroids, n_actions
    matrix::Matrix e; // ((n_centroids, n_actions)
    matrix::Matrix vals, new_vals;
    int action, new_action;
    double tot_reward;

    double width, rbf_sigma;
    double rbf_density;
    double epsilon, epsilon_final, epsilon_coefficient;
    double lambda, alpha, gamma;
    int num_rbf;
    int n_centroids, n_actions, dim_states;

    int episode, total_episode;

    matrix::Matrix phi(const matrix::Matrix & state){
        matrix::Matrix _phi = matrix::Matrix(1, centroids.getM());
        _phi.toZero();
        for (unsigned int i = 0; i < centroids.getM(); i++)
            _phi.val(0, i) = exp( (state - centroids.row(i)).norm_sqr() / rbf_density);
        return _phi;
    }

    double state_value(const matrix::Matrix & activation, int action){
        double val = (activation * theta.column(action)).val(0, 0);
        return val;
    }

    matrix::Matrix state_values(const matrix::Matrix & activation){
        matrix::Matrix val = (activation * theta);
        return val;
    }

    matrix::Matrix normalize_state(const matrix::Matrix & state){
        matrix::Matrix _state = (state - min_state);
        matrix::Matrix _size = max_state - min_state;
        // (1, dim)
        for (unsigned int  i = 0; i < _state.getN(); i++)
            _state.val(0, i) /= _size.val(0, i);
        return _state;
    }

    int epsilon_greedy(double _epsilon, const matrix::Matrix _vals){
        // vals : (1, actions)

        static std::default_random_engine generator;
        static std::uniform_real_distribution<double> uniform(0, 1);
        int _action;

        if (uniform(generator) < 1 - _epsilon)
            _action = argmax(_vals);
        else
            _action = int(uniform(generator) * tot_action);


        return _action;
    }

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

        first_rl = true;

        dim_states = 5;
        n_actions = 6;

        int num_per_dim = 6;
        num_rbf = num_per_dim * num_per_dim * num_per_dim * num_per_dim * num_per_dim;

        centroids = matrix::Matrix(0, dim_states);
        n_centroids = num_rbf;

        int id_rbf = 0;

        width = 1.0 / (6 - 1);
        for (int i0 = 0; i0 < num_per_dim; i0 ++)
            for (int i1 = 0; i1 < num_per_dim; i1 ++)
                for (int i2 = 0; i2 < num_per_dim; i2 ++)
                    for (int i3 = 0; i3 < num_per_dim; i3 ++)
                        for (int i4 = 0; i4 < num_per_dim; i4 ++)
                        {
                            double data[] = {i0 * width, i1 * width, i2 * width, i3 * width, i4 * width};
                            centroids.addRows(1, data);
                            id_rbf++;
                        }

        // initialize centroids
        total_episode = 1000;
        rbf_sigma = width * 1.5;
        rbf_density = 2 * rbf_sigma * rbf_sigma;
        epsilon = 0.6;
        epsilon_final = 0.1;
        epsilon_coefficient = pow(epsilon - epsilon_final, 1.0 / total_episode);

        lambda = 0.5;
        alpha = 0.01;
        gamma = 0.99;

        activations = Matrix(1, n_centroids);
        new_activations = Matrix(1, n_centroids);
        theta = Matrix(n_centroids, n_actions);
        theta.toMap([](double){
                    return uniform(generator) * 2 - 1;
                });

        double mins[] = {-1, -1, -1, -3, 0};
        double maxs[] = { 1,  1,  1,  3, 6};
        min_state = Matrix(1, 5, mins);
        max_state = Matrix(1, 5, maxs);

        e = Matrix(n_centroids, n_actions);
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
        first_rl = true;

        global.obstacles.clear();
        global.agents.pop_back();
        global.configs.pop_back();
        global.configs.pop_back();
        printf("there left %d agents. \n", global.agents.size());
        delete agent;

        FILE *fp = fopen("reinforce.log", "a");
        fprintf(fp, "%d %.3f %.4f\n", episode, global.time, episode);
        fclose(fp);

        if (episode % 100 == 0)
        {
            char fn[100];
            sprintf(fn, "rl/Q%d.dat", episode);
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
        if (globalData.time > 60)
            simulation_time_reached = true;

        if (globalData.time < 10)
            return;

        if (fabs(globalData.time / 2 - round(globalData.time / 2)) < 1e-4)
        {
            if (first_rl) {
                first_rl = false;
                tot_reward = 0;

                e.toZero();

                state = get_state(sphere1, dep);
                state = normalize_state(state);
                activations = phi(state);

                vals = state_values(activations);
                action = epsilon_greedy(epsilon, vals);
            } else {

                // action = 0..6
                //take action using --action--
                dep->switchConceptor(action);
                dep->switchConceptorh(action);

                new_state = get_state(sphere1, dep);
                double reward_value = reward(sphere1);
                tot_reward += reward_value;
                new_activations = phi(new_state);
                new_vals = state_values(activations);
                new_action = epsilon_greedy(epsilon, new_vals);

                double Q = state_value(activations, action);
                double Q_new = state_value(new_activations, new_action);
                double target = reward_value + gamma * Q_new - Q;

                for (int i = 0; i < activations.getN(); i++)
                    e.val(i, action) = activations.val(0, i);

                for (int k = 0; k < n_centroids; k++)
                    for (int a = 0; a < n_actions; a++)
                        theta.val(k, a) += alpha * target * e.val(k, a);
                e *= gamma * lambda;

                action = new_action;
                activations = new_activations;
                state = new_state;

                printf("time %.3f : score: %.3f speed: %.3f action: %d\n",
                       globalData.time, tot_reward, new_state.val(0, 3), new_action);
            }

            // printf("it: %d  time: %.3f  score: %.3f  speed: %.3f  action: %d\n",
            //       cur_iter, globalData.time, total_score, x_speed, cur_action);
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
    // sim.tot_iter = 50000;

    sim.setGroundTexture("Images/yellow_velour.rgb");
    return sim.run(argc, argv) ? 0 : 1;
}


