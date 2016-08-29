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

#include "conceptordep.h"
#include <selforg/matrixutils.h>
#include <cmath>

using namespace matrix;
using namespace std;

ConceptorDEP::ConceptorDEP(const std::vector<matrix::Matrix> &conceptors_,
                           const matrix::Matrix &initialW, const matrix::Matrix &initialWOut,
                           const matrix::Matrix &initialWbias,
                           const ConceptorDEPConf &conf_, const DEPConf &depconf) :
        DEP(depconf), cconf(conf_), W(initialW), W_out(initialWOut),
        conceptor_bias(initialWbias), conceptors(conceptors_),
        Wh(initialW), W_outh(initialWOut),
        conceptor_biash(initialWbias), conceptorsh(conceptors_) {
    ;
    num_patterns = conceptors.size();
    num_neuron = conceptors.begin()->getM();

    addParameterDef("lambdaH", &(cconf.lambdaH), 1.0, 0, 1,
                    "the percentage of H from DEP model");
    addParameterDef("lambdaC", &(cconf.lambdaC), 0.5, 0, 1,
                    "the percentage of C from DEP model");

};

void ConceptorDEP::init(int sensornumber, int motornumber, RandGen *randGen) {
    DEP::init(sensornumber, motornumber, randGen);

    Matrix X(num_neuron, 1);
    //random_device rd;
    //mt19937 gen(rd());
    //normal_distribution<> d(cconf.initialXMean, cconf.initialXDeviation);

    // initialize neuron state (X vector)
    double *X_temp = new double[num_neuron];
    for (int i = 0; i < num_neuron; i++)
        X_temp[i] = 0.5 * randGen->rand() + 0.25; // - 0.5 + cconf.initialXMean;
    X.set(X_temp);
    X.read(fopen("xarray.txt", "r"));
    X = X.rows(0, 149);
    // in fact, the initial value of "X" DOES AFFECT the sign of the
    // produced value matrix
    neuron = X;
    neuronh = X;
    delete[]X_temp;

    // initialize transition source and target
    transition_source = transition_target = 0;
    transition_sourceh = transition_targeth = 0;
    current_position = cconf.transitionTime;
    current_positionh = cconf.transitionTime;

    Matrix current_conceptor = getCurrentConceptor();
    for (int i = 0; i < 10; i++)
    {
        neuron = W * neuron + conceptor_bias;
        neuron = neuron.map(g);
        neuron = current_conceptor * neuron;
    }
    Matrix y = W_out * neuron;
    Matrix y_c = y.rows(0, number_sensors * number_motors - 1)
            .reshape(number_motors, number_sensors);

    double lambdaC = getParam("lambdaC");
    double lambdaH = getParam("lambdaH");
    C = C * lambdaC + y_c * (1 - lambdaC);
    C_update = C_update * lambdaC + y_c * (1 - lambdaC);

    Matrix current_conceptorh = getCurrentConceptorh();
    for (int i = 0; i < 10; i++)
    {
        neuronh = Wh * neuronh + conceptor_biash;
        neuronh = neuronh.map(g);
        neuronh = current_conceptorh * neuronh;
    }
    h *= 0;
}


ConceptorDEP::~ConceptorDEP() {
}

void ConceptorDEP::learnController() {
    DEP::learnController();
    // learn controller states using DEP rules

    // ------------------- update C ------------------------

    double lambdaC = getParam("lambdaC");
    double lambdaH = getParam("lambdaH");

    Matrix current_conceptor = getCurrentConceptor();
    if (current_position < cconf.transitionTime)
        current_position += 1;
    neuron = W * neuron + conceptor_bias;
    neuron = neuron.map(g);
    neuron = current_conceptor * neuron;
    // update X

    Matrix y = W_out * neuron;
    Matrix y_c = y.rows(0, number_sensors * number_motors - 1)
            .reshape(number_motors, number_sensors);

    C = C * lambdaC + y_c * (1 - lambdaC);
    C_update = C_update * lambdaC + y_c * (1 - lambdaC);
    // update C
    // maybe not correct, because C need initialization

    // -------------------- update H ------------------------

    Matrix current_conceptorh = getCurrentConceptorh();
    if (current_positionh < cconf.transitionTime)
        current_positionh += 1;
    neuronh = Wh * neuronh + conceptor_biash;
    neuronh = neuronh.map(g);
    neuronh = current_conceptorh * neuronh;
    // update X

    y = W_outh * neuronh;

    if (y.getM() > number_sensors * number_motors) {
        // assert(0 && "Seems it should not be here now");
        assert(y.getM() == number_sensors * number_motors + h.getM());
        Matrix y_h = y.rows(number_sensors * number_motors,
                            y.getM() - 1);
        h = h * lambdaH + y_h * (1 - lambdaH);
    }
    // update H
}

Matrix ConceptorDEP::getCurrentConceptor() {
    double lambda = (double) current_position / cconf.transitionTime;
    Matrix current =
            conceptors[transition_source] * (1 - lambda) +
            conceptors[transition_target] * lambda;
    return current;
}

Matrix ConceptorDEP::getCurrentConceptorh() {
    double lambda = (double) current_positionh / cconf.transitionTime;
    Matrix current =
            conceptorsh[transition_sourceh] * (1 - lambda);
    current = current +
            conceptorsh[transition_targeth] * lambda;
    return current;
}

void ConceptorDEP::switchConceptor(int tgt) {
    transition_source = transition_target;
    transition_target = tgt;
    current_position = 0;
}

void ConceptorDEP::switchConceptor(int src, int tgt) {
    transition_source = src;
    transition_target = tgt;
    current_position = 0;
}


void ConceptorDEP::switchConceptorh(int tgt) {
    transition_sourceh = transition_targeth;
    transition_targeth = tgt;
    current_positionh = 0;
}

void ConceptorDEP::switchConceptorh(int src, int tgt) {
    transition_sourceh = src;
    transition_targeth = tgt;
    current_positionh = 0;
}

/* stores the controller values to a given file. */
bool ConceptorDEP::store(FILE *f) const {
    // save matrix values
    C_update.store(f);
    h.store(f);
    M.store(f);
    b.store(f);
    Configurable::print(f, 0);
    return true;
}

/* loads the controller values from a given file. */
bool ConceptorDEP::restore(FILE *f) {
    // save matrix values
    C_update.restore(f);
    h.restore(f);
    Matrix Mod;
    Mod.restore(f);
    // old versions stored the model matrix in transposed form
    if (Mod.getM() == M.getM() && Mod.getN() == M.getN())
        M = Mod;
    else if (Mod.getM() == M.getN() && Mod.getN() == M.getM())
        M = Mod ^ T;
    else
        return false;
    b.restore(f);
    Configurable::parse(f);
    t = 0; // set time to zero to ensure proper filling of buffers
    return true;
}
