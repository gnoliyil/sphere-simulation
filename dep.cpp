/***************************************************************************
 *   Copyright (C) 2015 by Robot Group Leipzig                             *
 *    georg.martius@web.de                                                 *
 *    ralfder@mis.mpg.de                                                   *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
*                                                                         *
*                                                                         *
***************************************************************************/

#include "dep.h"
#include <selforg/matrixutils.h>

using namespace matrix;
using namespace std;

DEP::DEP(const DEPConf& conf)
  : AbstractController("DEP", "1.0"),
    conf(conf)
{
  t=0;

  addParameterDef("epsh", &epsh, 0.0,     0,5, "learning rate of the controller bias");
  addParameterDef("epsM", &epsM, 0.0,     0,5, "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,     1, buffersize-1, "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, 1, buffersize-1, "delay  (number of steps)");

  addParameterDef("learningrule",  (int*)(&this->conf.learningRule),
                  false,              string("which learning rule to use: ") +
                  std::accumulate(conf.LearningRuleNames.begin(),conf.LearningRuleNames.end(),
                                  std::string(),[](std::string a, std::pair<DEPConf::LearningRule,std::string> lr){return a + itos((int)lr.first) + ": " + lr.second + ", ";}));
  addParameterDef("timedist", &timedist, 1,     0,10, "time distance of product terms in learning rule");
  addParameterDef("synboost", &synboost, 5,     0,1,  "booster for synapses during motor signal creation");
  addParameterDef("urate", &urate, .1,          0,5,  "update rate ");

  //  addParameterDef("maxspeed", &maxSpeed, 0.5,   0,2, "maximal speed for motors");
  addParameterDef("indnorm", &indnorm,     1,   0,2, "individual normalization for each motor");
  addParameterDef("regularization", &regularization, 12, 0, 15, "exponent of regularization 10^{-regularization}");

  addInspectableMatrix("M", &M, false, "inverse-model matrix");

  addInspectableMatrix("h",  &h, false,   "acting controller bias");
  addInspectableMatrix("C", &C, false, "acting controller matrix");

  if(conf.calcEigenvalues){
    addInspectableMatrix("EvRe", &eigenvaluesLRe, false, "Eigenvalues of L (Re)");
    addInspectableMatrix("EvIm", &eigenvaluesLIm, false, "Eigenvalues of L (Im)");
    addInspectableMatrix("EVs",  &eigenvectors,   false, "Eigenvectors of L (Re)");
    addInspectableValue("proj1",  &proj_ev1,  "projection of x on first Eigenvector (Re)");
    addInspectableValue("proj2",  &proj_ev2,  "projection of x on second Eigenvector (Re)");
    addParameterDef("evinterval", &calcEVInterval, 1,          0,1000,  "interval to update eigenvalues etc (0: never) ");
  }

  addInspectableValue("norming", &norming, "Normalization");
  addInspectableMatrix("normmor", &normmot, false, "individual motor normalization factor");

  _internWithLearning=false; // used in step to enable learning in stepNoLearning and have only one function
};

DEP::~DEP(){
}


void DEP::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


  number_sensors= sensornumber;
  number_motors = motornumber;
  M.set(number_motors, number_sensors);
  C.set(number_motors, number_sensors);
  C_update.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors); // L = M * C
  eigenvectors.set(number_sensors,number_sensors);
  eigenvaluesLRe.set(number_sensors,1);
  eigenvaluesLIm.set(number_sensors,1);
  normmot.set(number_motors, 1);

  L.set(number_sensors, number_sensors);

  if(conf.initModel){
    // special model initialization for delay sensor
    //  (two ID matrices besides each other)
    if (number_sensors>=3*number_motors) {
      // Motor [x y z] + Ir[x, x'] + Ir[y, y'] + Ir[z, z'] + Sensor[x y z]
      Matrix M1(number_motors, number_motors);
      M1.toId();
      M = M1; 
      Matrix M2(number_motors, number_motors * 2); 
      M2.val(0, 0) = 1; 
      M2.val(0, 1) = -1; 
      M2.val(0, 2) = 0; 
      M2.val(0, 3) = 0; 
      M2.val(0, 4) = 0; 
      M2.val(0, 5) = 0; 
      
      M2.val(1, 2) = 1; 
      M2.val(1, 3) = -1;
      M2.val(2, 4) = 1; 
      M2.val(2, 5) = -1;
      M = M.beside(M2);
      M = M.beside(M1); 
    } else if (number_sensors>=2*number_motors){
      Matrix M1(number_motors, number_sensors/2);
      M1.toId();
      M=M1.beside(M1);
    }else
      M.toId(); // set a to identity matrix;
  }
  C_update.toId();
  C_update*=conf.initFeedbackStrength;

  x_smooth.set(number_sensors,1);

  x_buffer.init(buffersize, Matrix(number_sensors,1));
  y_buffer.init(buffersize, Matrix(number_motors,1));
 }


// performs one step (includes learning). Calculates motor commands from sensor inputs.
void DEP::step(const sensor* x_, int number_sensors,
               motor* y_, int number_motors){
  _internWithLearning=true;
  stepNoLearning(x_, number_sensors, y_, number_motors);
  _internWithLearning=false;
  // x -> y 
  // t`C(ij) = (`yi + d`yi)`x_j - C(ij)
};


void DEP::stepNoLearning(const sensor* x_, int number_sensors_robot,
                         motor* y_, int number_motors_){
  assert((unsigned)number_sensors_robot <= this->number_sensors
         && (unsigned)number_motors_ <= this->number_motors);

  Matrix xrobot(number_sensors_robot,1,x_); // store sensor values

  // averaging over the last s4avg values of x_buffer
  if(s4avg > 1)
    x_smooth += (xrobot - x_smooth)*(1.0/s4avg);
  else
    x_smooth = xrobot;

  x_buffer[t] = x_smooth;

  if(_internWithLearning)
    learnController();

  // Controller function
  Matrix y =   (C*x_smooth  + h  ).map(g);

  y_buffer[t] = y;

  if(_internWithLearning && epsM!=0)
    learnModel(epsM);

  y.convertToBuffer(y_, number_motors);
  // update step counter
  t++;
};


void DEP::learnController(){
  ///////////////// START of Controller Learning / Update  ////////////////
  int diff = 1;
  Matrix mu;
  Matrix v;
  switch(conf.learningRule){
  case DEPConf::DEP: { ////////////////////////////
    Matrix chi  = x_buffer[t] - x_buffer[t - diff];
    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    mu = (M * chi);
    break;
  }
  case DEPConf::DHL:  ////////////////////////////
    mu  = y_buffer[t -   diff] - y_buffer[t - 2*diff];
    v = x_buffer[t - timedist] - x_buffer[t - timedist - diff];
    break;
  case DEPConf::HL:  ////////////////////////////
    mu = y_buffer[t -   diff];
    v  = x_buffer[t - diff];
    break;
  default:
    cerr << "unkown learning rule!" << endl;
  }
  const Matrix& updateC =   ( mu ) * (v^T);


  if ( t > 10){
    C_update += ((updateC   - C_update)*urate);  // matrix for controller before normalization
  }

  double reg = pow(10,-regularization);
  switch(indnorm){
  case 1: {
    //***** individual normalization for each motor neuron***************
    const Matrix& CM=C_update*(M^T);
    for (int i=0; i<number_motors; i++) {
      double normi = sqrt(CM.row(i).norm_sqr()); // norm of one row
      // for some historic reasons there is a 0.3 here
      //  which we keep for consistency with the paper
      normmot.val(i,0) = .3*synboost/( normi + reg);
    }
    C = C_update.multrowwise(normmot);
    break;
  }
  case 0: { // global
    double norm1 = sqrt((C_update*(M^T)).norm_sqr());
    C = C_update * (synboost /(norm1 + reg)); // C stays relatively constant in size
    C.toMapP(5.0,clip); // nevertheless clip C to some reasonable range
    break;
  }
  default:  { // no normalization
    C = C_update * synboost;
    break;
  }}
  if(conf.calcEigenvalues){
    if(calcEVInterval!=0 && (t%calcEVInterval==0)){
      Matrix EVImag;
      const Matrix& L=(M^T)*C;
      eigenValuesVectors(L, eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
      toPositiveSignEigenVectors(eigenvectors, EVImag);
      scaleEigenVectorsWithValue(eigenvaluesLRe, eigenvaluesLIm, eigenvectors, EVImag);
    }
    // calc overlap of sensor state with first 2 eigenvectors (this we do every step)
    proj_ev1=((eigenvectors.column(0)^T) * x_buffer[t]).val(0,0);
    proj_ev2=((eigenvectors.column(1)^T) * x_buffer[t]).val(0,0);
  }

  const Matrix& y_last = y_buffer[t-1];
  if(epsh>=0)
    h -= ( y_last *  epsh).mapP(.05, clip) + h*.001;
  else
    h*=0;

  ///////////////// End of Controller Learning ////////
}

void DEP::learnModel(double eps){
  // learn inverse model y = F(x') = M x'
  // the effective x/y is (actual-steps4delay) element of buffer
  s4delay = ::clip(s4delay,1,buffersize-1);
  int  t_delay =  max(s4delay,1)-1;
  if(eps!=0){
    const Matrix& y = y_buffer[t - t_delay - timedist];
    const Matrix& x_fut   = x_buffer[t];
    const Matrix& xi = y - (M * x_fut);
    // M += eps xi x_fut^T
    M += (xi*(x_fut^T) * eps).mapP(0.05, clip) - M*eps*0.01; //update - damping
  }
};



void DEP::motorBabblingStep(const sensor* x_, int number_sensors_robot,
                            const motor* y_, int number_motors){
  assert((unsigned)number_motors <= this->number_motors);
  Matrix x(number_sensors_robot,1,x_); // convert to matrix
  Matrix y(number_motors,1,y_); // convert to matrix
  x_buffer[t] = x;
  y_buffer[t] = y;

  // model learning
  learnModel(1.0/sqrt(t+1));

  t++;
}


/* stores the controller values to a given file. */
bool DEP::store(FILE* f) const{
  // save matrix values
  C_update.store(f);
  h.store(f);
  M.store(f);
  b.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool DEP::restore(FILE* f){
  // save matrix values
  C_update.restore(f);
  h.restore(f);
  Matrix Mod;
  Mod.restore(f);
  // old versions stored the model matrix in transposed form
  if(Mod.getM()==M.getM() && Mod.getN()==M.getN())
    M=Mod;
  else if(Mod.getM()==M.getN() && Mod.getN()==M.getM())
    M=Mod^T;
  else
    return false;
  b.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}
