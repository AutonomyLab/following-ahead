#pragma once

#include "config.h"
#include <vector>
#include <random>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class PredictionParticleFilter
{
public:
	class PredictionParticle
	{
	public:
		PredictionParticle(cv::Point3f state=cv::Point3f(0,0,0), float weight=0.0);

		cv::Point3f getState() { return state_; }
		void setState(cv::Point3f state) { state_ = state; }
		
		float getWeight() { return weight_; }
		void setWeight(float weight) { if (std::isfinite(weight)) weight_ = weight; }

		void sampleFromMeasurementGaussian(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
											float velocity, float velocity_stddev,
											float orientation, float orientation_stddev,
											float weight, float dt,
											std::uniform_real_distribution<float> &noise_distribution_x,
											std::uniform_real_distribution<float> &noise_distribution_y,
											std::default_random_engine &random_generator );
	private:
		cv::Point3f state_;
		float weight_;
	};

	PredictionParticleFilter();
	~PredictionParticleFilter();
	
	void init(	size_t num_particles, 
				cv::Point3f target_initial_position_estimate,
				float velocity, float velocity_stddev,
				float orientation, float orientation_stddev	);

	bool isInitialized() { return is_initialized_; }
	/**
	 * Reintialize the filter
	 */
	void reintialize() { is_initialized_ = false; }

	size_t getNumPredictionParticles() { return num_particles_; }

	// float p_measurement_given_state(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
	// 									float velocity, float velocity_stddev,
	// 									float orientation, float orientation_stddev,
	// 									cv::Point3f state, float dt	);

	float getUpdateProbabilities(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
									float velocity, float velocity_stddev,
									float orientation, float orientation_stddev,
									cv::Point3f state, float dt,
									float &p_measurement_given_state_out, float &p_measurement_out	);

	void normalizeParticleWeights();

	PredictionParticle getPredictionParticleAt(size_t idx);


	void resamplePredictionParticle();
	void update(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
					float velocity, float velocity_stddev,
					float orientation, float orientation_stddev,
					float dt	);

private:
	std::vector<PredictionParticle> particles_;
	size_t num_particles_;
	bool is_initialized_;
	
	float velocity_;
	float velocity_stddev_;

	float orientation_;
	float orientation_stddev_; 

	tf::TransformListener tf_listener_;

	std::default_random_engine random_generator_;
};