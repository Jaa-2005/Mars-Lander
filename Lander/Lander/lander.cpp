// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    
    //defining autopilot local variables
    vector3d er;
    double v, e, h, Pout, delta, Kp, Kh;
    
    //set constants
    Kp = 0.9;
    Kh = 0.018;
    delta = 0.1;
    
    //define control theory equations
    h = (position.abs() - MARS_RADIUS);
    er = position.norm();
    v = velocity * er;
    e = -(0.5 + Kh*h + v);
    Pout = Kp*e;
    
    //update throttle values
    if (Pout <= -delta){
        throttle = 0;
    }else if (-delta < Pout && Pout <= (1 - delta)){
        throttle = delta + Pout;
    }else{
        throttle = 1;
    }
    
    //make file with altitude and velocity for plot.
    
    {ofstream fout;
        fout.open("/Users/jessicaallen/Documents/results14.txt", ios::app);
        
        if (!fout) {
            cerr << "Error opening file!" << endl;
        }else{
            fout<< h << ' ' << v << endl;
        }
        fout.close();}
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    
    bool debug_mode = 0;
    
    //declare local variables
    static vector3d previous_position;
    vector3d force, lander_drag, chute_drag, g_force, new_position;
    double current_mass, drag_lander_mag, g_force_mag,chute_drag_mag;
    
    //update current mass (unsure what they have defined as fuel)
    current_mass = UNLOADED_LANDER_MASS + fuel*FUEL_DENSITY*FUEL_CAPACITY;
    
    //calculate the magnitude of the forces
    drag_lander_mag =  0.5*DRAG_COEF_LANDER*atmospheric_density(position)*M_PI*LANDER_SIZE*LANDER_SIZE*velocity.abs2();
    g_force_mag = (GRAVITY*MARS_MASS*current_mass)/position.abs2();
    chute_drag_mag = 0.5*DRAG_COEF_CHUTE*atmospheric_density(position)*5.0*2.0*LANDER_SIZE*2.0*LANDER_SIZE*velocity.abs2();
    
    //make the forces into 3D vectors
    lander_drag = -drag_lander_mag*velocity.norm();
    chute_drag = -chute_drag_mag*velocity.norm();
    g_force = -g_force_mag*position.norm();
    
    // not sure to use current mass or where to place this line
    previous_position = position - velocity*delta_t + 0.5*pow(delta_t, 2)*force/current_mass;
    
    //force claculation based on parachute status
    if (parachute_status == NOT_DEPLOYED){
        force = g_force + lander_drag + thrust_wrt_world();
    }else{
        force = g_force + lander_drag + chute_drag + thrust_wrt_world();
    }
    
    //euler version of the numerical dynamics
    
    //position = position + delta_t*velocity;
    //velocity = velocity + delta_t*(force/current_mass);
    
    
    //numerical integration taking the case of t=0 as a euler integration
    if (simulation_time == 0.0){
        //euler
        position = position + delta_t*velocity;
        velocity = velocity + delta_t*(force/current_mass);
        
        if (debug_mode) cout << "Position:" << position << endl;
        if (debug_mode) cout << "Velocity:" << velocity << endl;
        if (debug_mode) cout << "g_force:"<< g_force << endl;
        if (debug_mode) cout << "lander_drag"<< lander_drag << endl;
        
    }else{
        //verlet
        new_position = 2*position - previous_position + (force/current_mass)*pow(delta_t, 2);
        velocity = (new_position - position)/delta_t;
        
        previous_position = position;
        position = new_position;
        
    }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "descent from 10km";
  scenario_description[5] = "circular orbit";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[0] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 1:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 0:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
