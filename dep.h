/***************************************************************************
 *   Copyright (C) 2015 by Robot Group Leipzig                             *
 *    georg.martius@web.de                                                 *
 *    ralfder@mis.mpg.de                                                   *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
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
 *                                            *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __DEP_H
#define __DEP_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/ringbuffer.h>

/// configuration object for DEP controller. Use DEP::getDefaultConf().
struct DEPConf {
#define LEARNINGRULES  \
    X(DEP,      "DEP") \
    X(DHL,      "DHL") \
    X(HL,       "HL")

  enum LearningRule {
#define X(Enum, String)       Enum,
    LEARNINGRULES
#undef X
  };
  std::map<LearningRule, std::string> LearningRuleNames = {
#define X(Enum, String) { Enum , String } ,
    LEARNINGRULES
#undef X
  };

  LearningRule learningRule;

  double initFeedbackStrength;  ///< initial strength of sensor to motor connection
  bool   initModel;             ///< initialize model or leave it 0 to be learned
  /// # of steps the sensors are averaged (1 means no averaging)
  int    steps4Averaging;
  /// # of steps the motor values are delayed (1 means no delay)
  int    steps4Delay;
  bool   calcEigenvalues;       ///< if true calculate the eigenvalues of L
};


/**
 * This controller implements a new very much simplified algorihm derived from TiPI maximization
 */
class DEP : public AbstractController {

public:
  DEP(const DEPConf& conf = getDefaultConf());
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~DEP();

  static DEPConf getDefaultConf(){
    DEPConf conf;
    conf.learningRule=DEPConf::DEP;
    conf.initFeedbackStrength = 0;
    conf.steps4Averaging      = 1;
    conf.steps4Delay          = 1;
    conf.calcEigenvalues      = false;
    conf.initModel            = true;
    return conf;
  }


  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_sensors; }
  /// returns the mumber of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const  { return number_motors; }

  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /// called during babbling phase
  virtual void motorBabblingStep(const sensor* , int number_sensors,
                                 const motor* , int number_motors);


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  // accessors to matrices
  virtual matrix::Matrix getM(){  return M; }
  virtual void setM(const matrix::Matrix& _M){
    assert(M.getM() == _M.getM() && M.getN() == _M.getN());
    M=_M;
  }
  // accessors to matrices
  virtual matrix::Matrix getC(){  return C_update; }
  virtual void setC(const matrix::Matrix& _C){
    assert(C_update.getM() == _C.getM() && C_update.getN() == _C.getN());
    C_update=_C;
  }

protected:
  unsigned short number_sensors;
  unsigned short number_motors;
  static const unsigned short buffersize = 150;

  DEPConf conf; // configuration object

  matrix::Matrix M; // Model Matrix
  matrix::Matrix C_update; // fast changing controller matrix (function of immediate history)
  matrix::Matrix C; // Acting Controller Matrix (normalized C_update)
  matrix::Matrix h; // Controller Bias
  matrix::Matrix b; // Model Bias
  matrix::Matrix L; // Jacobi Matrix

  RingBuffer<matrix::Matrix> x_buffer; // buffer needed for delay and derivatives
  RingBuffer<matrix::Matrix> y_buffer; // buffer needed for delay and derivatives

  matrix::Matrix x_smooth; // time average of x values
  matrix::Matrix normmot; // factors for individual normalization

  matrix::Matrix eigenvaluesLRe; //Eigenvalues of L matrix real part
  matrix::Matrix eigenvaluesLIm; //Eigenvalues of L matrix imaginary part
  matrix::Matrix eigenvectors; //Eigenvectors of L matrix (real part)
  double proj_ev1; // projection of x into first eigenvector
  double proj_ev2; // projection of x into second eigenvector
  int calcEVInterval;

  int t;

  paramval epsh;
  paramval epsM;
  paramval norming;
  paramint s4avg;          // # of steps the sensors are averaged (1 means no averaging)
  paramint s4delay;        // # of steps the motor values are delayed (1 means no delay)

  int      indnorm;        ///< individual normalization (1) and global normalization (0)
  int      regularization; ///< exponent of regularization 10^{-regularization}

  paramval urate;          ///<
  paramval synboost;       ///< kappa in the paper

  paramval timedist;
  bool _internWithLearning;

  /// learn  model (M = A^T )
  virtual void learnModel(double eps);

  /// learn controller (C,h, C_update)
  virtual void learnController();

  /// neuron transfer function
  static double g(double z)
  {
    return tanh(z);
  };

  /// function that clips the second argument to the interval [-r,r]
  static double clip(double r, double x){
    return min(max(x,-r),r);
  }


};

#endif
