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
                           const ConceptorDEPConf &conf_, const DEPConf &depconf) :
        DEP(depconf), cconf(conf_), W(initialW), W_out(initialWOut),
        conceptors(conceptors_) {
    ;
    num_patterns = conceptors.size();
    num_neuron = conceptors.begin()->getM();
    Matrix X(num_neuron, 1);
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> d(cconf.initialXMean, cconf.initialXDeviation);

    // initialize neuron state (X vector)
    double *X_temp = new double[num_neuron];
    for (int i = 0; i < num_neuron; i++)
        X_temp[i] = d(gen);
    X.set(X_temp);
    neuron = X;
    delete[]X_temp;

    // initialize transition source and target
    transition_source = transition_target = 0;
    current_position = 100;
};

ConceptorDEP::~ConceptorDEP() {
}

void ConceptorDEP::learnController() {
    Matrix current_conceptor = getCurrentConceptor();
    if (current_position < cconf.transitionTime)
        current_position += 1;
    Matrix new_neuron(neuron);
    neuron = W * neuron;
    neuron.map(g);
    // update X

    Matrix y = W_out * neuron;
    Matrix y_c = y.rows(0, number_sensors * number_motors - 1)
            .reshape(number_motors, number_sensors);
    C = C * cconf.lambdaC + y_c * (1 - cconf.lambdaC);
    // update C

    if (y.getM() > number_sensors * number_motors) {
        assert(y.getM() == number_sensors * number_motors + h.getM());
        Matrix y_h = y.rows(number_sensors * number_motors,
                            y.getM() - 1);
        h = h * cconf.lambdaH + y_h * (1 - cconf.lambdaH);
    }
    // update H

    DEP::learnController();
    // learn controller states using DEP rules
}

Matrix ConceptorDEP::getCurrentConceptor() {
    double lambda = (double) current_position / cconf.transitionTime;
    Matrix current =
            conceptors[transition_source] * (1 - lambda) +
            conceptors[transition_target] * lambda;
    return current;
}

void ConceptorDEP::switchConceptor(int src, int tgt) {
    transition_source = src;
    transition_target = tgt;
    current_position = 0;
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
