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
#include "particle_filter.h"

#define NUMBER_OF_PARTICLES 50


inline double ParticleFilter::normalSample(double std_dev)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dist(0.0, std_dev);
  return dist(gen); 
}

inline int ParticleFilter::uniformIntSample(int max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, max);
  return dist(gen); 
}

inline double ParticleFilter::uniformDoubleSample(double max)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0.0, max);
  return dist(gen); 
}

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
  num_particles = NUMBER_OF_PARTICLES;
  particles.resize(num_particles);
  weights.resize(num_particles);
  for (int i=0; i<particles.size(); i++)
  {
    weights[i] = 1.0;
    particles[i].id = i;
    particles[i].x = x + normalSample(std[0]);
    particles[i].y = y + normalSample(std[1]);
    particles[i].theta = theta + normalSample(std[2]);
    particles[i].weight = 1.0;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std[], double velocity, double yaw_rate) 
{
  for (int i=0; i<particles.size(); i++)
  {
    double dist = velocity * delta_t;
    double theta = particles[i].theta;
    // Conditionally do the full prediction including yaw rate
    if (fabs(yaw_rate) < 0.001)
    {
      particles[i].x += dist * cos(theta) + normalSample(std[0]);
      particles[i].y += dist * sin(theta) + normalSample(std[1]);
    }
    else
    {
      particles[i].x += velocity * (sin(theta + yaw_rate * delta_t) - sin(theta)) / yaw_rate + normalSample(std[0]);
      particles[i].y += velocity * (cos(theta) - cos(theta + yaw_rate * delta_t)) / yaw_rate + normalSample(std[1]);
      particles[i].theta += yaw_rate * delta_t + normalSample(std[2]);
    }
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> map_landmarks_in_range, std::vector<LandmarkObs>& observations_in_map_coordinates) 
{
  std::vector<LandmarkObs> result;
  for (int i=0; i<observations_in_map_coordinates.size(); i++)
  {
    int map_index;
    double dist_comp = 1.0e99;
    for (int j=0; j<map_landmarks_in_range.size(); j++)
    {
      double delta_x = map_landmarks_in_range[j].x - observations_in_map_coordinates[i].x;
      double delta_y = map_landmarks_in_range[j].y - observations_in_map_coordinates[i].y;
      double d = delta_x * delta_x + delta_y * delta_y; 
      if (d < dist_comp)
      {
        map_index = j;
        dist_comp = d;
      } 
    }
    LandmarkObs l = {map_index, observations_in_map_coordinates[i].x, observations_in_map_coordinates[i].y};
    result.push_back(l);
  }
  observations_in_map_coordinates = result;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations, Map map_landmarks) {
  std::vector<Particle> new_particles;
  std::vector<double> new_weights;
  for (int i=0; i<particles.size(); i++)
  {
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;
    // Find map landmarks within the sensor range of the particle
    std::vector<LandmarkObs> map_landmarks_in_range;
    for (int j=0; j<map_landmarks.landmark_list.size(); j++)
    {
      int index = map_landmarks.landmark_list[j].id_i;
      double m_x = map_landmarks.landmark_list[j].x_f;
      double m_y = map_landmarks.landmark_list[j].y_f;
      if (dist(p_x, p_y, m_x, m_y) < sensor_range)
      {
        LandmarkObs l = {index, m_x, m_y};
        map_landmarks_in_range.push_back(l); 
      }
    }
    // Change observation coordinates to map coordinates
    std::vector<LandmarkObs> observations_in_map_coordinates;
    for (int j=0; j<observations.size(); j++)
    {
      int index = observations[j].id;
      double o_x = observations[j].x;
      double o_y = observations[j].y;
      double m_x = p_x + o_x * cos(p_theta) - o_y * sin(p_theta);
      double m_y = p_y + o_x * sin(p_theta) + o_y * cos(p_theta); 
      LandmarkObs l = {index, m_x, m_y};
      observations_in_map_coordinates.push_back(l); 
    }
    // Use the data association method 
    dataAssociation(map_landmarks_in_range, observations_in_map_coordinates);
    // Compute the new weight based on two vectors from point to observation and landmark
    double new_weight = 1.0; 
    for (int j=0; j<observations_in_map_coordinates.size(); j++)
    {
      double o_x = observations_in_map_coordinates[j].x - p_x;
      double o_y = observations_in_map_coordinates[j].y - p_y;
      int map_index = observations_in_map_coordinates[j].id;
      double m_x = map_landmarks_in_range[map_index].x - p_x;
      double m_y = map_landmarks_in_range[map_index].y - p_y;
      double o_length = sqrt(o_x * o_x + o_y * o_y);
      double o_angle = atan2(o_y, o_x);
      double m_length = sqrt(m_x * m_x + m_y * m_y);
      double m_angle = atan2(m_y, m_x);
      double delta_length = o_length - m_length;
      double delta_angle = o_angle - m_angle; 	
      new_weight *= gaussian(std_landmark[0], delta_length, std_landmark[1], delta_angle);
    } 
    particles[i].weight = new_weight;
    weights[i] = new_weight;
  }
}

inline double ParticleFilter::gaussian(double sigma_a, double x_a, double sigma_b, double x_b)
{
  double num_a = x_a * x_a / (2.0 * sigma_a * sigma_a);
  double num_b = x_b * x_b / (2.0 * sigma_b * sigma_b);
  double numerator = exp(-1.0 * (num_a + num_b));
  double denominator = 2.0 * M_PI * sigma_a * sigma_b;
  return numerator / denominator;
}

void ParticleFilter::resample() 
{
  // Normalize weights
  double total_weight = 0.0;
  for (int i=0; i<particles.size(); i++)
  {
    total_weight += particles[i].weight;
  }
  for (int i=0; i<particles.size(); i++)
  {
    weights[i] /= total_weight;
    particles[i].weight /= total_weight;
  }
  // Max of these weights
  double max_weight = 0.0;
  for (int i=0; i<particles.size(); i++)
  {
    if (weights[i] > max_weight)
    {
      max_weight = weights[i];
    }
  }
  // Resample
  double beta = 0.0;
  int particles_size = particles.size();
  int index = uniformIntSample(particles_size);
  std::vector<Particle> new_particles;
  std::vector<double> new_weights;
  for (int i=0; i<num_particles; i++)
  {
    beta += uniformDoubleSample(2.0 * max_weight);
    while (weights[index] < beta)
    {
      beta -= weights[index];
      index = (index + 1) % particles_size;
    }
    Particle p = {i, particles[index].x, particles[index].y, particles[index].theta, 1.0};
    new_particles.push_back(p); 
    new_weights.push_back(1.0);
  }
  particles = new_particles;
  weights = new_weights;
}

void ParticleFilter::write(std::string filename) 
{
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  for (int i=0; i<num_particles; i++) 
  {
    dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
  }
  dataFile.close();
}
