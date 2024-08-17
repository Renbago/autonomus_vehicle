/*
* MIT License
* 
* Copyright (c) 2024 Mehmet Baha Dursun
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*
 * Email: baha.dursun123@gmail.com
 */

#pragma once

#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <locale.h>  

using namespace casadi;

// Scenario names as constants
const std::string SCENARIO_PARK = "park";
const std::string SCENARIO_CROSSWALK = "crosswalk";
const std::string SCENARIO_MOTORWAY = "motorway";
const std::string SCENARIO_INITIAL = "initial_settings";

class MpcStartSetting {
public:

    typedef struct {
        double Q_x;
        double Q_y;
        double Q_theta;
        double R1;
        double R2;
        double step_horizon;
        int N;
        double rob_diam;
        double wheel_radius;
        double L_value;
        double Ly;
        double v_max;
        double v_min;
        double omega_max;
        double omega_min;
        int iteration;
    } Settings;

    Settings initial_settings_;

    typedef struct {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    } Pose;

    Pose localisation_data_;
    std::string scenerio_name_;
    void mpc_start_setting(const std::string& scenerio_name_);

    
};