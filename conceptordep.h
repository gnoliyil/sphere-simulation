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
#ifndef __CONCEPTORDEP_H
#define __CONCEPTORDEP_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/ringbuffer.h>

#include "dep.h"
#include <random> // used for normal distribution number generator

/// configuration object for DEP controller. Use DEP::getDefaultConf().
struct ConceptorDEPConf {
    double initialXMean;
    double initialXDeviation; // let X be initialized as a normal distributed vector
    double lambdaC; // C = lambdaC * (C_next) + (1 - lambdaC) * (C_conceptor)
    double lambdaH; // h = lambdaH * (h_next) + (1 - lambdaH) * (H_conceptor)
    int transitionTime; // time for transitions between two conceptors
};


/**
 * This controller implements a conceptor based DEP controller of lpzrobot simulator.
 */
class ConceptorDEP : public DEP {

public:
    ConceptorDEP(const std::vector<matrix::Matrix> &conceptors_,
                 const matrix::Matrix &initialW, const matrix::Matrix &initialWOut,
                 const matrix::Matrix &initialWbias,
                 const ConceptorDEPConf &conf_ = getDefaultConf(),
                 const DEPConf &depconf = DEP::getDefaultConf());

    virtual ~ConceptorDEP();

    virtual void init(int sensornumber, int motornumber, RandGen *randGen);

    static ConceptorDEPConf getDefaultConf() {
        ConceptorDEPConf conf;
        conf.initialXMean = 0.5;
        conf.initialXDeviation = 0.5;
        conf.lambdaC = 0.90;
        conf.lambdaH = 0.90;
        conf.transitionTime = 100;
        return conf;
    }

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE *f) const;

    /** loads the controller values from a given file. */
    virtual bool restore(FILE *f);

    void switchConceptor(int k);
    void switchConceptor(int i, int j);

    matrix::Matrix getCurrentConceptor();

    void switchConceptorh(int k);
    void switchConceptorh(int i, int j);

    matrix::Matrix getCurrentConceptorh();

    // accessors to matrices
    virtual matrix::Matrix getM() { return M; }

    virtual int getTargetConceptorId(){
        return transition_target;
    }

    virtual int getTargetConceptorIdh(){
        return transition_targeth;
    }

    virtual void setM(const matrix::Matrix &_M) {
        assert(M.getM() == _M.getM() && M.getN() == _M.getN());
        M = _M;
    }

    // accessors to matrices
    virtual matrix::Matrix getC() { return C_update; }

    virtual void setC(const matrix::Matrix &_C) {
        assert(C_update.getM() == _C.getM() && C_update.getN() == _C.getN());
        C_update = _C;
    } // C_update ?

protected:
    ConceptorDEPConf cconf; // configuration object

    // conceptor related variables
    matrix::Matrix neuron, neuronh; // neuron state
    matrix::Matrix W, Wh;      // x(i+1) = conceptor * tanh(W*(x(i) + conceptor_bias) + random_bias
    matrix::Matrix W_out, W_outh;  // W_out from conceptor matrix
    matrix::Matrix conceptor_bias, conceptor_biash;
    std::vector<matrix::Matrix> conceptors, conceptorsh;
    matrix::Matrix conceptor_matrix;
    int num_patterns;
    int num_neuron;
    int transition_source,
        transition_target;
    int current_position;

    int transition_sourceh,
        transition_targeth;
    int current_positionh;
    // ------ end conceptor related variables -------

    /// learn controller (C,h, C_update)
    virtual void learnController();

    /// neuron transfer function
    static double g(double z) {
        return tanh(z);
    };

    /// function that clips the second argument to the interval [-r,r]
    static double clip(double r, double x) {
        return min(max(x, -r), r);
    }


};

#endif
