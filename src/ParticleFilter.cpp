#include "ParticleFilter.hpp"
#include "utils.hpp"
#include <cmath>

ParticleFilter::Particle::Particle(cv::Point3f state, float weight)
{
	setState(state);
	setWeight(weight);
}

ParticleFilter::ParticleFilter() 
	: is_initialized_(false)
{

}

ParticleFilter::~ParticleFilter()
{
	delete measurement_noise_distribution_x_;
	delete measurement_noise_distribution_y_;
}

ParticleFilter::Particle ParticleFilter::getParticleAt(size_t idx)
{
	assert(is_initialized_ && idx < getNumParticles()); 
	return particles_[idx]; 
}

float ParticleFilter::p_measurement_given_state(cv::Point3f measurement, cv::Point3f state)
{
	assert(is_initialized_);

	// 2D gaussian
	double sigma_x = measurement_noise_distribution_x_->stddev();
	double sigma_y = measurement_noise_distribution_y_->stddev();
	return 	1.0/(2*M_PI*sigma_x*sigma_y) *
			exp(-
				(
					pow(state.x - measurement.x, 2)/(2*pow(sigma_x, 2)) +
					pow(state.y - measurement.y, 2)/(2*pow(sigma_y, 2))
				)
			);
}

void ParticleFilter::update(cv::Point3f current_relative_estimate, tf::StampedTransform r0_T_map)
{	
	cv::Point3f curr_global_estimate = transformPoint(r0_T_map.inverse(), current_relative_estimate);
	
	// model with no sense of orientation !!!
	cv::Point3f velocity(
		curr_global_estimate.x - prev_global_estimate_.x,
		curr_global_estimate.y - prev_global_estimate_.y,
		curr_global_estimate.z - prev_global_estimate_.z
	);


	for (size_t i = 0, len = particles_.size(); i < len; i++)
	{
		cv::Point3f prev_state = particles_[i].getState();
		cv::Point3f curr_state(
			prev_state.x + velocity.x + (*stochastic_velocity_distribution_)(random_generator_),
			prev_state.y + velocity.y + (*stochastic_velocity_distribution_)(random_generator_),
			0
		);
		
		float new_weight = particles_[i].getWeight() * p_measurement_given_state(curr_global_estimate, curr_state);
		particles_[i].setWeight(new_weight);
		particles_[i].setState(curr_state);
	}

	resampleParticle();

	prev_global_estimate_ = curr_global_estimate;
}

void ParticleFilter::resampleParticle()
{
	assert(is_initialized_);
	
	float old_total = 0;
	float new_total = 0;
	std::vector<float> cumulative_weights(particles_.size(), 0);

	for (size_t i = 0; i < num_particles_; i++)
	{
		old_total += particles_[i].getWeight();
		cumulative_weights[i] = old_total;
	}

	std::vector<Particle> new_particles(num_particles_);

	for (size_t new_particle_idx = 0; new_particle_idx < num_particles_; new_particle_idx++)
	{
		float rand_choice = rand()/(float)RAND_MAX * old_total;

		// TODO: do this efficiently without going through all the particles again and again!!!
		for (size_t prev_particle_idx = 0; prev_particle_idx < num_particles_; prev_particle_idx++)
		{
			if (rand_choice <= cumulative_weights[prev_particle_idx])
			{
				// sample chosen
				new_particles[new_particle_idx] = particles_[prev_particle_idx];
				new_total += new_particles[new_particle_idx].getWeight();
				break;
			}
		}
	}

	// normalize the weights
	for (Particle &particle: new_particles)
	{
		particle.setWeight(
			particle.getWeight() / new_total
		);
	}

	particles_ = new_particles;

}

void ParticleFilter::init(	size_t num_particles, 
							float measurement_noise_stddev_x, float measurement_noise_stddev_y,
							float particle_stochastic_velocity_stddev,
							cv::Point3f initial_relative_estimate, tf::StampedTransform r0_T_map,
							std::string base_frame, std::string map_frame	)
{
	base_frame_ = base_frame;
	map_frame_ = map_frame;
	
	num_particles_ = num_particles;
	particles_.clear();
	particles_.reserve(num_particles_);

	tf::StampedTransform map_T_r0 = tf::StampedTransform(r0_T_map.inverse(), r0_T_map.stamp_, map_frame_, base_frame_);
	// TODO: propagate error in robot position to error in global estimate
	prev_global_estimate_ = transformPoint(map_T_r0, initial_relative_estimate);


	measurement_noise_distribution_x_ = new std::normal_distribution<float>(0, measurement_noise_stddev_x*PARTICLE_INIT_NOISE_FACTOR_X);
	measurement_noise_distribution_y_ = new std::normal_distribution<float>(0, measurement_noise_stddev_y*PARTICLE_INIT_NOISE_FACTOR_Y);
	stochastic_velocity_distribution_ = new std::normal_distribution<float>(0, particle_stochastic_velocity_stddev);
	// for orientation
	std::uniform_real_distribution<float> orientation_distribution(-M_PI, M_PI);

	for (size_t i = 0; i < num_particles_; i++)
	{
		cv::Point3f r0_noisy_point;

		float x_noise = (*measurement_noise_distribution_x_)(random_generator_);
        float y_noise = (*measurement_noise_distribution_y_)(random_generator_);
		r0_noisy_point.x = initial_relative_estimate.x + x_noise; 
        r0_noisy_point.y = initial_relative_estimate.y + y_noise;
        r0_noisy_point.z = 0;

		particles_.push_back(
			Particle(
				transformPoint(map_T_r0, r0_noisy_point),
				1 // can use the probability of the noise as weight
			)
		);
	}

	is_initialized_ = true;
}