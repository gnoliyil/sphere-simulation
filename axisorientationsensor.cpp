/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <selforg/matrix.h>

#include <ode_robots/primitive.h>
#include "axisorientationsensor.h"
#include <ode_robots/mathutils.h>

namespace lpzrobots {

  AxisOrientationSensorMod::AxisOrientationSensorMod(Mode mode, short dimensions)
    : mode(mode), dimensions(dimensions) {
    own = 0;
    setBaseInfo(SensorMotorInfo("AxisOrientation").changequantity(SensorMotorInfo::Position));
    printf("###############################create\n") ;
  }

  void AxisOrientationSensorMod::init(Primitive* own, Joint* joint){
    this->own = own;
  }

  int AxisOrientationSensorMod::getSensorNumber() const{

    short n = ((dimensions & X) != 0) + ((dimensions & Y) != 0) + ((dimensions & Z) != 0);
    switch (mode) {
    case OnlyZAxis:
    case ZProjection:
      return n;
      break;
    case Axis:
      return 3*n;
      break;
    }
    return 0;
  }

  bool AxisOrientationSensorMod::sense(const GlobalData& globaldata) { return true; }

  std::list<sensor> AxisOrientationSensorMod::getList() const {
    assert(own);
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );

    switch (mode) {
    case OnlyZAxis:
      if(dimensions == (X | Y | Z)) return A.column(2).convertToList();
      else return selectrows(A.column(2),dimensions); break;
    case ZProjection:
      if(dimensions == (X | Y | Z)) return A.row(2).convertToList();
      else return selectrows(A.row(2)^(matrix::T),dimensions);  break;
    case Axis:
      if(dimensions == (X | Y | Z)) return A.convertToList();
      else return selectrows(A,dimensions); break;
    }
    return std::list<sensor>();
  }

  int AxisOrientationSensorMod::get(sensor* sensors, int length) const{
    assert(own);
    static matrix::Matrix lastTime(3, 3);
    static double time_interval = 0.01; 
     
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( own->getBody() ) );
    
    //calculate angular velocity: 
    matrix::Matrix deriv = (A - lastTime) * (1 / time_interval); // * A^(matrix::T); 
    matrix::Matrix angular = deriv * (A^(matrix::T)); 
    
    double w_x = angular.val(0, 2), 
           w_y = angular.val(1, 0), 
          w_z = angular.val(2, 1); 
    lastTime = A; 
    printf("%lf %lf %lf\n", w_x, w_y, w_z); 
    
    switch (mode) {
    case OnlyZAxis:
      if(dimensions == (X | Y | Z)) return A.column(2).convertToBuffer(sensors, length);
      else return selectrows(sensors, length, A.column(2),dimensions); break;
    case ZProjection:
      if(dimensions == (X | Y | Z)) {
        int return_value = A.row(2).convertToBuffer(sensors, length);
        //for (int i = 0; i < length; i++) {
        //  printf("%lf ", sensors[i]); 
        //}
        //printf("\n"); 
        return return_value; 
      }
      else return selectrows(sensors, length, A.row(2)^(matrix::T),dimensions);  break;
    case Axis:
      if(dimensions == (X | Y | Z)) return A.convertToBuffer(sensors, length);
      else return selectrows(sensors, length, A,dimensions); break;
    }
    return 0;
  }

}

