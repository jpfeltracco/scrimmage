/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef _SCRIMMAGE_COLORMAPS_
#define _SCRIMMAGE_COLORMAPS_

// Plugins use the Colormap functions, but this compilation unit doesn't know
// that. Disable the unused function warning.

namespace scrimmage {

typedef struct Color{
    double r,g,b;
} Color_t;

// Matlab's versions of JET to GRAYSCALE
// Matlab uses the same JET version as BlueView sonar JET
Color_t GetColor_matlab(double v, double vmin, double vmax);
double GetGray_matlab(Color_t c, double vmin, double vmax);

// "Traditional" JET conversions
Color_t GetColor(double v, double vmin, double vmax);

int GetGray(Color_t c, double vmin, double vmax);

}

#endif
