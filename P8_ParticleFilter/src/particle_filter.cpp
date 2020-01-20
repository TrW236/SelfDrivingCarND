/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#define PI 3.14159265359
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <array>
#include "map.h"
#include "particle_filter.h"

using namespace std;

#define NUM_PARTICLES 100

bool debug = false;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = NUM_PARTICLES;

	default_random_engine gen;
	double std_x, std_y, std_theta, weight;
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	for (int i = 0; i < num_particles; ++i)
	{
		x = dist_x(gen);
		y = dist_y(gen);
		theta = dist_theta(gen);
		weight = 1.0;
		Particle par;
		par = (Particle) { i, x, y, theta};
		weights.push_back(weight);
		particles.push_back(par);
		if (debug)
		{
			cout<<"Particle idx:"<<i<<"; x:"<<x<<"; y:"<<y<<"; theta:"<<theta<<endl;
		}
		

	}
	is_initialized = true;

	if (debug)
	{
		cout<<"--------initialized-------end------"<<endl;
	}
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	 for (int i = 0; i < num_particles; ++i)  // loop for every particle
	 {	
	 	if(abs(yaw_rate)<0.0001) {
 			particles[i].x += velocity * delta_t * cos(particles[i].theta);
	 		particles[i].y += velocity * delta_t * sin(particles[i].theta);
	 	} else {
	 		particles[i].x += velocity / yaw_rate *(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
	 		particles[i].y += velocity / yaw_rate *(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
	 		particles[i].theta += yaw_rate * delta_t;
 		}
	 	normal_distribution<double> dist_x(particles[i].x, std_x);
		normal_distribution<double> dist_y(particles[i].y, std_y);
		normal_distribution<double> dist_theta(particles[i].theta, std_theta);

	 	particles[i].x = dist_x(gen);
	 	particles[i].y = dist_y(gen);
	 	particles[i].theta = dist_theta(gen);
	 	if (debug)
	 	{
	 		cout<<"predicted particle idx:"<<i<<"; x:"<<particles[i].x<<"; y:"<<particles[i].y<<"; theta:"<<particles[i].theta<<endl;
	 	}
	 	
	 }
	 if (debug)
	 {
	 	cout<<"--------predicted--------end-----"<<endl;
	 }
	 
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {  // len observations equals len map_landmarks (when range = inf)
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
	double std_r, std_b;  // range and bearing deviation
	std_r = std_landmark[0];
	std_b = std_landmark[1];
	weights.clear();
	for (int i = 0; i < num_particles; ++i) // loop every particle
	{	
		double par_x = particles[i].x;
		double par_y = particles[i].y;
		double par_theta = particles[i].theta;
		// cout<<par_x<<","<<par_y<<","<<par_theta<<endl;
		// restrict sensor range (reduce landmarks to consider)
		Map selected_map;
		selected_map.landmark_list.clear();
		// std::vector<Map.single_landmark_s> possible_landmarks;
		int idx_landmark = 0;
		for (int j = 0; j < map_landmarks.landmark_list.size(); ++j)  // loop for very landmarks
		{	
			if (abs(map_landmarks.landmark_list[j].x_f - par_x) <= sensor_range && 
				abs(map_landmarks.landmark_list[j].y_f - par_y) <= sensor_range)
			{
				selected_map.landmark_list.push_back(map_landmarks.landmark_list[j]);
				if (debug)
				{
					cout<<"selected landmark(x,y)"<<map_landmarks.landmark_list[j].x_f<<", "<<map_landmarks.landmark_list[j].y_f<<"; idx:"<<map_landmarks.landmark_list[j].id_i<<endl;
					idx_landmark++;
				}
				
			}
		}

		// transform coordinate system
		std::vector<LandmarkObs> transformed_obs;
		LandmarkObs tmp_ob;
		for (int j = 0; j < observations.size(); ++j)  // loop for very observations
		{	
			tmp_ob.x = observations[j].x * cos(par_theta) - observations[j].y * sin(par_theta) + par_x;
			tmp_ob.y = observations[j].x * sin(par_theta) + observations[j].y * cos(par_theta) + par_y;


			// association

			double distance = pow(selected_map.landmark_list[0].x_f - tmp_ob.x, 2) + pow(selected_map.landmark_list[0].y_f-tmp_ob.y, 2); // first landmark
			double tmp_dist;
			tmp_ob.id = 0;
			for (int k = 1; k < selected_map.landmark_list.size(); ++k)  // loop for every possible landmarks
			{
				tmp_dist = pow(selected_map.landmark_list[k].x_f - tmp_ob.x,2) + pow(selected_map.landmark_list[k].y_f-tmp_ob.y,2);
				if (tmp_dist < distance)
				{
					distance = tmp_dist;
					tmp_ob.id = k;
				}
			}
			transformed_obs.push_back(tmp_ob);
			if (debug)
			{
				cout<<"observation(x, y)"<<observations[j].x<<", "<<observations[j].y<<"->transformed(x, y)"<<tmp_ob.x<<", "<<tmp_ob.y<<"<-associated landmark idx:"<<selected_map.landmark_list[tmp_ob.id].id_i<<endl;
			}
			
		}

		// update weights
		double new_w = 1.0;
		for (int j = 0; j < transformed_obs.size(); ++j)  // loop for every ob
		{	
			new_w *= exp(-pow(transformed_obs[j].x - selected_map.landmark_list[transformed_obs[j].id].x_f, 2)/2.0/pow(std_r, 2)
						-pow(transformed_obs[j].y - selected_map.landmark_list[transformed_obs[j].id].y_f, 2)/2.0/pow(std_b,2))/
						2.0/PI/std_r/std_b;
			if (debug)
			{
				cout<<"landmark idx: "<< selected_map.landmark_list[transformed_obs[j].id].id_i<<"; std "<<std_r<<", "<<std_b<<endl;
				cout<<"Particle "<<i<<"; observation nr. "<<j<<"; weight: "<<new_w<<endl;
			}
			
		}
		weights.push_back(new_w);
		// std::cout<<weights[i];
	}
	if (debug)
	{
		cout<<"--------updated Weights-------end------"<<endl;
	}
	
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::vector<Particle> selected_particles;
	std::default_random_engine generator;
	
	std::discrete_distribution<> distribution (weights.begin(), weights.end());

	for (int i = 0; i < num_particles; ++i)
	{
		int idx = distribution(generator);
		selected_particles.push_back(particles[idx]);
	}

	particles = selected_particles;
	if (debug)
	{
		cout<<"--------resampled-------end------"<<endl;
	}
	
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
