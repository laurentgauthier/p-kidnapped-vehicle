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

// This random engine is shared by all the random number generations
// in the particle filter below.
static default_random_engine random_engine;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // The particles first position is based on estimates of x, y, theta and
    // their uncertainties from GPS and all weights to 1.

    // We work with 100 particles
    num_particles = 100;

    // Use normal distributions to generate sensor noise
    normal_distribution<double> gaussion_noise_x(0, std[0]);
    normal_distribution<double> gaussian_noise_y(0, std[1]);
    normal_distribution<double> gaussian_noise_theta(0, std[2]);

    // Initialize individual particles
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = x;
        p.y = y;
        p.theta = theta;
        p.weight = 1.0;

        // Add random noise to each individual particle
        p.x += gaussion_noise_x(random_engine);
        p.y += gaussian_noise_y(random_engine);
        p.theta += gaussian_noise_theta(random_engine);

        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Add measurements to each particle and add random Gaussian noise.

    // Use normal distributions for sensor noise
    normal_distribution<double> gaussian_noise_x(0, std_pos[0]);
    normal_distribution<double> gaussion_noise_y(0, std_pos[1]);
    normal_distribution<double> gaussion_noise_theta(0, std_pos[2]);

    for (int i = 0; i < num_particles; i++) {
        // Update the state, but be careful when the yaw_rate is small
        // as there is both a risk of division by zero, or of numerical
        // instability if the value is very small.
        if (fabs(yaw_rate) < 0.0001) {
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
        } else {
            particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }

        // Add noise to the particle state
        particles[i].x += gaussian_noise_x(random_engine);
        particles[i].y += gaussion_noise_y(random_engine);
        particles[i].theta += gaussion_noise_theta(random_engine);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    // observed measurement to this particular landmark.
    unsigned int n_observations = observations.size();
    unsigned int n_predictions = predicted.size();

    for (unsigned int i = 0; i < n_observations; i++) {
        // Initialize min distance with a large value.
        double min_distance = std::numeric_limits<double>::max();
        int landmark_id = -1;

        for (unsigned j = 0; j < n_predictions; j++) {
            double diff_x = observations[i].x - predicted[j].x;
            double diff_y = observations[i].y - predicted[j].y;
            double distance = diff_x * diff_x + diff_y * diff_y;

            // If the "distance" is less than min, stored the id and update min.
            if (distance < min_distance) {
                min_distance = distance;
                landmark_id = predicted[j].id;
            }
        }

        // Update the landmark id for the observation.
        observations[i].id = landmark_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // Update the weights of each particle using a mult-variate Gaussian distribution.

    for (int i = 0; i < num_particles; i++) {
        std::vector<LandmarkObs> nearby_landmarks;
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

        // Just look for landmarks within sensor range.
        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            float landmark_x = map_landmarks.landmark_list[j].x_f;
            float landmark_y = map_landmarks.landmark_list[j].y_f;
            int id = map_landmarks.landmark_list[j].id_i;
            double diff_x = x - landmark_x;
            double diff_y = y - landmark_y;
            if ((diff_x*diff_x + diff_y*diff_y) <= sensor_range*sensor_range) {
                nearby_landmarks.push_back(LandmarkObs{ id, landmark_x, landmark_y });
            }
        }

        // Remap the observations in the particle referential.
        vector<LandmarkObs> remapped_observations;

        for(unsigned int j = 0; j < observations.size(); j++) {
            double remapped_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
            double remapped_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
            remapped_observations.push_back(LandmarkObs{ observations[j].id, remapped_x, remapped_y });
        }

        // Associate observations to landmarks.
        dataAssociation(nearby_landmarks, remapped_observations);

        // Re-calculate the weight of the particle.
        particles[i].weight = 1.0;
        for (unsigned int j = 0; j < remapped_observations.size(); j++) {
            double observation_x = remapped_observations[j].x;
            double observation_y = remapped_observations[j].y;
            int landmark_id = remapped_observations[j].id;

            unsigned int n_landmarks = nearby_landmarks.size();
            double landmark_x, landmark_y;
            unsigned int k = 0;
            bool found = false;
            while (!found && k < n_landmarks) {
                if (nearby_landmarks[k].id == landmark_id) {
                    found = true;
                    landmark_x = nearby_landmarks[k].x;
                    landmark_y = nearby_landmarks[k].y;
                }
                k++;
            }

            // Calculate the weight based on the distance from the landmark, and the standard deviation
            // of the sensor measurements.
            double diff_x = observation_x - landmark_x;
            double diff_y = observation_y - landmark_y;
            double distance_squared = diff_x*diff_x + diff_y*diff_y;

            double weight = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1])
                              * exp(-distance_squared/(2*std_landmark[0]*std_landmark[1]));
            if (weight == 0.0) {
                particles[i].weight *= 0.000001;
            } else {
                particles[i].weight *= weight;
            }
        }
    }
}

void ParticleFilter::resample() {
    // Resample particles with replacement with probability proportional to their weight.
    std::vector<Particle> resampled_particles;
    std::vector<double>   weights;

    // Gather the weights of the current particles
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }

    // Generate a random index for use in the resampling wheel
    std::discrete_distribution<int> random_index(0, num_particles-1);
    int index = random_index(random_engine);

    // Get the maximum weight value
    double maximum_weight = *std::max_element(weights.begin(), weights.end());

    // Uniform distribution
    std::uniform_real_distribution<double> random_beta(0.0, maximum_weight);

    // Resampling wheel algorithm
    double beta = 0.0;

    for (int i = 0; i < num_particles; i++) {
        beta += random_beta(random_engine) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

    particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
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
