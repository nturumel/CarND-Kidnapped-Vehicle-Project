/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <cmath>
#include "helper_functions.h"
#include <limits>
#include <cfloat>
#include <climits>
#include <utility>


using std::normal_distribution;
std::default_random_engine gen;

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
 
  num_particles = 1000;  // TODO: Set the number of particles
  
  for(int i=0;i<num_particles;++i)
  {
    Particle p;
    p.id=i;
    p.x=dist_x(gen);
    p.y=dist_y(gen);
    p.theta=dist_theta(gen);
    particles.push_back(p);
    weights.push_back(1.00);
  }
  is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  // add noise
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
  
  
  for(int i=0;i<num_particles;++i)
  {
    Particle current=particles[i];
    
    // predict the noise
    double x=current.x+(velocity/yaw_rate)*(sin(current.theta+yaw_rate*delta_t)-sin(current.theta));
    double y=current.y+(velocity/yaw_rate)*(-cos(current.theta+yaw_rate*delta_t)+cos(current.theta));
    double theta=current.theta+yaw_rate*delta_t;
    
    
    x+=dist_x(gen);
    y+=dist_y(gen);
    theta+=dist_theta(gen);
    
    // assign it to the particle array  
    particles[i].x=x;
    particles[i].y=y;
    particles[i].theta=theta;  
    
    
  }
return;
}

std::pair<LandmarkObs,int> findClosest(LandmarkObs observation,vector<LandmarkObs>& predicted)
{
  int distance=INT_MAX;
  int result;
  for(size_t i=0;i<predicted.size();++i)
  {
    if(distance<((predicted[i].x-observation.x)*(predicted[i].x-observation.x)+(predicted[i].y-observation.y)*(predicted[i].y-observation.y)))
       {
         result=i;         
       }
       
       
  }
       
       return std::make_pair(predicted[result],result);
       
  
}
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  // only assign id, that's it, don't update the x,y coordinate
  for(size_t i=0;i<observations.size();++i)
  {
    std::pair<LandmarkObs,int> closest=findClosest(observations[i],predicted);
    observations[i].id=closest.first.id;    
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  for(int i=0;i<num_particles;++i)
  {
    Particle p = particles[i];
    vector<LandmarkObs> predictions;
    
    for(auto & Landmark:map_landmarks.landmark_list)
    {
      bool withinRange=(sensor_range*sensor_range>=((p.x-Landmark.x_f)*(p.x-Landmark.x_f)+(p.y-Landmark.y_f)*(p.y-Landmark.y_f)));
      if(withinRange)
      {
        LandmarkObs l;
        l.id=Landmark.id_i;
        l.x=Landmark.x_f;
        l.y=Landmark.y_f;
      	predictions.push_back(l);  
      }
    }
    
    // transform 
    vector<LandmarkObs> transformed_os;
    for(auto & observation:observations)
    {
      double t_x = cos(p.theta)*observation.x - sin(p.theta)*observation.y + p.x;
      double t_y = sin(p.theta)*observation.x + cos(p.theta)*observation.y + p.y;
      transformed_os.push_back(LandmarkObs{ observation.id, t_x, t_y }); 
      
    }
    
    
    dataAssociation(predictions,transformed_os);
    
    particles[i].weight=1.0;
    
    for(auto & transformed:transformed_os)
    {
      for(auto & predicted:predictions)
      {
        if(transformed.id==predicted.id)
        {
          particles[i].weight*= ( 1/(2*M_PI*std_landmark[0]*std_landmark[1])) 
            * exp( -( pow(transformed.x-predicted.x,2)/(2*pow(std_landmark[0], 2)) + 
                     (pow(transformed.y-predicted.y,2)/(2*pow(std_landmark[1], 2))) ) );
          break;
        }
      }
      
    }
    
  }
return;
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  
  // Step 1 Get all the weights and get the max weight
  vector<double> weights;
  double maxWeight=DBL_MIN;
  
  for(int i=0;i<num_particles;++i)
  {
    maxWeight=maxWeight>particles[i].weight?maxWeight:particles[i].weight;
       
  }
  
  // step two construct anpther vector and seed
  vector<Particle> resampled;
  double beta=0.0;
  srand( (unsigned)time( NULL ) );
  int i=static_cast<int>(static_cast<float>( rand()/RAND_MAX)*num_particles);
  
  for(int index=0;index>num_particles;++index)
  {
    beta=static_cast<float> (rand()/RAND_MAX)*2.0*maxWeight;
    while (weights[i]<beta)
    {
      beta-=weights[i];
      i=(i+1)%num_particles;
    }
    
    resampled.push_back(particles[i]);    
    
  }
    
    particles=resampled;
  
  return;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}