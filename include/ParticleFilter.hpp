#pragma once
#include "config.h"
#include <vector>
#include <random>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class ParticleFilter
{
public:
	class Particle
	{
	public:
		Particle(cv::Point3f state=cv::Point3f(0,0,0), float weight=0.0);

		cv::Point3f getState() { return state_; }
		void setState(cv::Point3f state) { state_ = state; }
		
		float getWeight() { return weight_; }
		void setWeight(float weight) { weight_ = weight; }
	private:
		cv::Point3f state_;
		float weight_;
	};

	ParticleFilter();
	~ParticleFilter();
	
	void init(	size_t num_particles, 
				float measurement_noise_stddev_x, float measurement_noise_stddev_y,
				float particle_stochastic_velocity_stddev,
				cv::Point3f initialEstimate, tf::StampedTransform r0_T_map,
				std::string base_frame, std::string map_frame );

	bool isInitialized() { return is_initialized_; }
	size_t getNumParticles() { return num_particles_; }

	float p_measurement_given_state(cv::Point3f measurement, cv::Point3f state);

	Particle getParticleAt(size_t idx);


	void resampleParticle();
	void update(cv::Point3f initialEstimate, tf::StampedTransform r0_T_map);

private:
	std::vector<Particle> particles_;
	size_t num_particles_;
	bool is_initialized_;
	cv::Point3f prev_global_estimate_;
	// tf::StampedTransform prev_r0_T_map_;

	tf::TransformListener tf_listener_;

	std::default_random_engine random_generator_;
	std::normal_distribution<float> *measurement_noise_distribution_x_;
	std::normal_distribution<float> *measurement_noise_distribution_y_;

	std::normal_distribution<float> *stochastic_velocity_distribution_;

	std::string base_frame_;
	std::string map_frame_;
};