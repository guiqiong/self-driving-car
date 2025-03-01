/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    default_random_engine gen;
    num_particles = 10;
    weights.resize(num_particles);
    
    // This line creates a normal (Gaussian) distribution for x, y, theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    for (int i = 0; i < num_particles; ++i) {
        Particle particle;
        
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1;
        
        particles.push_back(particle);
        
        weights[i] = 1.0;
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    
    default_random_engine gen;
    if (fabs(yaw_rate) < 0.0001) {
        yaw_rate = 0.0001;
    }
    
    for (int i = 0; i < num_particles; ++i) {
        double g = velocity/yaw_rate;
        double d = yaw_rate*delta_t;
        
        // This line creates a normal (Gaussian) distribution for x, y, theta
        normal_distribution<double> dist_x(0, std_pos[0]);
        normal_distribution<double> dist_y(0, std_pos[1]);
        normal_distribution<double> dist_theta(0, std_pos[2]);
        
        particles[i].x +=  g*(sin(particles[i].theta+d)-sin(particles[i].theta)) + dist_x(gen);
        particles[i].y += g*(cos(particles[i].theta)-cos(particles[i].theta+d)) + dist_y(gen);
        particles[i].theta += d + dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    for(int i=0; i<observations.size(); ++i){
        LandmarkObs closest_landmark;
        double dist=1000000.0;
        for(int j=0; j<predicted.size(); ++j){
            double dist_tmp = pow(predicted[i].x-observations[j].x,2)+pow(predicted[i].y-observations[j].y,2);
            if(dist_tmp < dist){
                dist = dist_tmp;
                closest_landmark = predicted[j];
            }
        }
        observations[i] = closest_landmark;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
    
    double sum_weights = 0;
    for(int i=0; i<particles.size(); ++i){
        
        std::vector<LandmarkObs> map_observations;
        std::vector<LandmarkObs> associate_landmarks;
        
        long double weight=1;
        
        for(int j=0; j<observations.size(); ++j){
            
            double cos_theta = cos(particles[i].theta);
            double sin_theta = sin(particles[i].theta);
            double x_map = particles[i].x + observations[j].x*cos_theta - observations[j].y*sin_theta;
            double y_map = particles[i].y + observations[j].x*sin_theta + observations[j].y*cos_theta;
            
            double min_dist = sensor_range*sensor_range;
            Map::single_landmark_s min_landmark;
            
            for(int k=0; k<map_landmarks.landmark_list.size(); ++k){
                
                double dist = pow(x_map-map_landmarks.landmark_list[k].x_f, 2) + pow(y_map-map_landmarks.landmark_list[k].y_f,2);
                
                if(dist < min_dist){
                    min_dist = dist;
                    min_landmark = map_landmarks.landmark_list[k];
                }
            }
            
            double gauss_norm = 1/(2*M_PI*std_landmark[0]*std_landmark[1]);
            double exponent = 0.5*pow(x_map-min_landmark.x_f, 2)/pow(std_landmark[0],2) + 0.5*pow(y_map-min_landmark.y_f, 2)/pow(std_landmark[1],2);
            
            weight *= gauss_norm * exp(-exponent);

        }
        particles[i].weight = weight;
        weights[i] = weight;
        sum_weights += weight;
    }
    
    for(int i=0; i<particles.size(); ++i){
        particles[i].weight /= sum_weights;
        weights[i] /= sum_weights;
        
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    std::vector<Particle> particles_new;
    
    std::vector<int> w;
    for(int i=0; i<num_particles; ++i){
        w.push_back((int)(particles[i].weight*100+0.5));
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(w.begin(), w.end());
    for(int i=0; i<num_particles; ++i){
        int res = d(gen);
        particles_new.push_back(particles[res]);
    }
    particles = particles_new;

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
