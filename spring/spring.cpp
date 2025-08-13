#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

using namespace std;

int main() {
    
    auto start = std::chrono::high_resolution_clock::now();
    
    // declare variables
        double m, k, x_current, v_current, t_max, dt, t, a_initial, x_previous, a, x_next, v_next;
        vector<double> t_list, x_list, v_list;
        
        // mass, spring constant, initial position and velocity
        m = 1;
        k = 1;
        x_current = 0;
        v_current = 1;
        
        // simulation time and timestep
        t_max = 100;
        dt = 0.1;
        
        a_initial = -k * x_current / m;
        x_previous = x_current - v_current * dt + 0.5 * a_initial * pow(dt, 2);
        
        // Verlet integration
    for (t = 0; t <= t_max; t = t + dt) {
        
        // append current state to trajectories
        t_list.push_back(t);
        x_list.push_back(x_current);
        v_list.push_back(v_current);
        
        // calculate new position and velocity
        a = -k * x_current / m;
        
        x_next = 2 * x_current - x_previous + a * pow(dt,2);
        
        v_next = (x_next - x_current)/dt;
        
        x_previous = x_current;
        x_current = x_next;
        v_current = v_next;
    }
        
        
        
        auto end = std::chrono::high_resolution_clock::now();
        
        // Calculate duration in milliseconds
    std::chrono::duration<double>duration = end -start;
        
        std::cout << "Execution time: " << duration.count() << " seconds" << endl;
    
        return 0;
        
        // Write the trajectories to file
        //ofstream fout;
        //fout.open("Users/Documents/trajectories.txt");
        //if (fout) { // file opened successfully
        //for (int i = 0; i < t_list.size(); i = i + 1) {
        //  fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
        //   }
        // } else { // file did not open successfully
        //    cout << "Could not open trajectory file for writing" << endl;
        //  }
        
        
        
        
        /* The file can be loaded and visualised in Python as follows:
         
         import numpy as np
         import matplotlib.pyplot as plt
         results = np.loadtxt('trajectories.txt')
         plt.figure(1)
         plt.clf()
         plt.xlabel('time (s)')
         plt.grid()
         plt.plot(results[:, 0], results[:, 1], label='x (m)')
         plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
         plt.legend()
         plt.show()
         
         */
    }
