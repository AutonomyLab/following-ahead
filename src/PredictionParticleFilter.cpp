#include "PredictionParticleFilter.hpp"
#include "utils.hpp"
#include <cmath>

PredictionParticleFilter::PredictionParticle::PredictionParticle(cv::Point3f state, float weight)
{
	setState(state);
	setWeight(weight);
}

void PredictionParticleFilter::PredictionParticle::sampleFromMeasurementGaussian(	
				cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
				float velocity, float velocity_stddev,
				float orientation, float orientation_stddev,
				float weight, float dt,
				std::uniform_real_distribution<float> &noise_distribution_x,
				std::uniform_real_distribution<float> &noise_distribution_y,
				std::default_random_engine &random_generator
)
{
	cv::Point3f predicted_position(
		current_target_position.x + PREDICTION_LOOKAHEAD_DISTANCE * cos(orientation),
		current_target_position.y + PREDICTION_LOOKAHEAD_DISTANCE * sin(orientation),
		0
	);

	float noise_x = noise_distribution_x(random_generator);
	float noise_y = noise_distribution_y(random_generator);

	cv::Point3f local_coord(noise_x, noise_y, 0);

	// 2D gaussian
	tf::Transform transform_global_local;
	
	transform_global_local.setOrigin( 
		tf::Vector3(
			predicted_position.x, 
			predicted_position.y, 
			0.0
		) 
	);
	tf::Quaternion q;
	q.setRPY(0, 0, orientation);
	transform_global_local.setRotation(q);

	state_ = transformPoint(transform_global_local, local_coord);
	weight_ = weight;
}

PredictionParticleFilter::PredictionParticleFilter() : is_initialized_(false)
{

}

PredictionParticleFilter::~PredictionParticleFilter()
{
	
}

PredictionParticleFilter::PredictionParticle PredictionParticleFilter::getPredictionParticleAt(size_t idx)
{
	assert(is_initialized_ && idx < getNumPredictionParticles()); 
	return particles_[idx]; 
}

// float PredictionParticleFilter::p_measurement_given_state(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
// 															float velocity, float velocity_stddev,
// 															float orientation, float orientation_stddev,
// 															cv::Point3f state, float dt	)
// {
// 	assert(is_initialized_);

// 	// standard deviation in local x and y direction due to orientation and velocity respectively
// 	float orientation_error_stddev = PREDICTION_LOOKAHEAD_DISTANCE * tan(orientation_stddev);

// 	float velocity_error_stddev = velocity_stddev * dt + 
// 							sqrt(pow(current_target_position_stddev.x, 2) + pow(current_target_position_stddev.y, 2));
		

// 	// 2D gaussian
// 	tf::Transform transform_global_local;
// 	tf::Transform transform_local_global;
	
// 	transform_global_local.setOrigin( 
// 		tf::Vector3(
// 			current_target_position.x + PREDICTION_LOOKAHEAD_DISTANCE * cos(orientation), 
// 			current_target_position.y + PREDICTION_LOOKAHEAD_DISTANCE * sin(orientation), 
// 			0.0
// 		) 
// 	);
// 	tf::Quaternion q;
// 	q.setRPY(0, 0, orientation);
// 	transform_global_local.setRotation(q);

// 	transform_local_global = transform_global_local.inverse();
// 	cv::Point3f global_coord(state.x, state.y, 0);

// 	cv::Point3f local_coord = transformPoint(transform_local_global, global_coord);
	

// 	float p_state_given_measurement = 	1.0/(2*M_PI*orientation_error_stddev*velocity_error_stddev) *
// 										exp(-
// 											(
// 												pow(local_coord.x, 2)/(2*pow(orientation_error_stddev, 2)) +
// 												pow(local_coord.y, 2)/(2*pow(velocity_error_stddev, 2))
// 											)
// 										);
// 	float p_measurement = std::min(
// 		1 - std::min( fabs(orientation - orientation_) , MAX_DEL_THETA ) / MAX_DEL_THETA + MEASUREMENT_PROBABILITY_EPSILON,
// 		1.0
// 	);

// 	return p_state_given_measurement * p_measurement;
// }

float PredictionParticleFilter::getUpdateProbabilities(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
														float velocity, float velocity_stddev,
														float orientation, float orientation_stddev,
														cv::Point3f state, float dt,
														float &p_measurement_given_state_out, float &p_measurement_out	)
{
	assert(is_initialized_);

	// standard deviation in local x and y direction due to orientation and velocity respectively
	float orientation_error_stddev = velocity_stddev * dt; // PREDICTION_LOOKAHEAD_DISTANCE * tan(orientation_stddev);

	float velocity_error_stddev = velocity_stddev * dt; 
									// + sqrt(pow(current_target_position_stddev.x, 2) + pow(current_target_position_stddev.y, 2));
		

	// 2D gaussian
	tf::Transform transform_global_local;
	tf::Transform transform_local_global;
	
	transform_global_local.setOrigin( 
		tf::Vector3(
			current_target_position.x + PREDICTION_LOOKAHEAD_DISTANCE * cos(orientation), 
			current_target_position.y + PREDICTION_LOOKAHEAD_DISTANCE * sin(orientation), 
			0.0
		) 
	);
	tf::Quaternion q;
	q.setRPY(0, 0, orientation);
	transform_global_local.setRotation(q);

	transform_local_global = transform_global_local.inverse();
	cv::Point3f global_coord(state.x, state.y, 0);

	cv::Point3f local_coord = transformPoint(transform_local_global, global_coord);
	

	p_measurement_given_state_out = 	1.0/(2*M_PI*orientation_error_stddev*velocity_error_stddev) *
										exp(-
											(
												pow(local_coord.x, 2)/(2*pow(orientation_error_stddev, 2)) +
												pow(local_coord.y, 2)/(2*pow(velocity_error_stddev, 2))
											)
										);
	p_measurement_out = std::min(
		1 - std::min( fabs(orientation - orientation_) , MAX_DEL_THETA ) / MAX_DEL_THETA + MEASUREMENT_PROBABILITY_EPSILON,
		1.0
	);	
}				

void PredictionParticleFilter::normalizeParticleWeights()
{
	float total_weight = 0;
	for (PredictionParticle &particle: particles_)
	{
		total_weight += particle.getWeight();
	}

	for (PredictionParticle &particle: particles_)
	{
		particle.setWeight( particle.getWeight() / total_weight );
	}
}						

void PredictionParticleFilter::update(	cv::Point3f current_target_position, cv::Point3f current_target_position_stddev, 
										float velocity, float velocity_stddev,
										float orientation, float orientation_stddev,
										float dt  )
{	
	if (velocity_stddev == 0)
	{
		velocity_stddev = STDDEV_EPSILON;
	}

	if (orientation_stddev == 0)
	{
		orientation_stddev = STDDEV_EPSILON;
	}

	float cos_theta = cos(orientation_);
	float sin_theta = sin(orientation_);

	float cos_theta_2 = pow(cos_theta, 2);
	float sin_theta_2 = pow(sin_theta, 2);

	float velocity_2 = pow(velocity_, 2);


	std::normal_distribution<float> velocity_noise_x(0, velocity_stddev_ * cos_theta_2);
	std::normal_distribution<float> velocity_noise_y(0, velocity_stddev_ * sin_theta_2);

	std::normal_distribution<float> orientation_noise_x(0, velocity_stddev_ * velocity_2 * sin_theta_2);
	std::normal_distribution<float> orientation_noise_y(0, velocity_stddev_ * velocity_2 * cos_theta_2);


	for (size_t i = 0, len = particles_.size(); i < len; i++)
	{
		cv::Point3f prev_state = particles_[i].getState();
		cv::Point3f curr_state(
			prev_state.x + velocity_ * cos(orientation_) + velocity_noise_x(random_generator_) + orientation_noise_x(random_generator_),
			prev_state.y + velocity_ * sin(orientation_) + velocity_noise_y(random_generator_) + orientation_noise_y(random_generator_),
			0
		);	

		float p_measurement_given_state, p_measurement;

		float new_weight;
		
		getUpdateProbabilities(
			current_target_position, current_target_position_stddev, 
			velocity, velocity_stddev,
			orientation, orientation_stddev,
			curr_state, dt,
			p_measurement_given_state, p_measurement
		);

		if ( (float)rand() / RAND_MAX < p_measurement && std::isfinite(p_measurement_given_state))
		{
			new_weight = particles_[i].getWeight() * p_measurement_given_state;
		}
		else
		{
			// the measurement was very unlikely, keep the previous weight
			new_weight = particles_[i].getWeight();
		}
		// float new_weight = particles_[i].getWeight() * p_measurement_given_state(
		// 	current_target_position, current_target_position_stddev, 
		// 	velocity, velocity_stddev,
		// 	orientation, orientation_stddev,
		// 	curr_state, dt
		// );

		if 	(
				std::isfinite(curr_state.x) &&
				std::isfinite(curr_state.y) &&
				std::isfinite(curr_state.z)
			)
		{
			particles_[i].setWeight(new_weight);
			particles_[i].setState(curr_state);
		}
		else
		{
			ROS_ERROR("Particle state %d infinite", i);
		}
	}

	normalizeParticleWeights();

	{
		// standard deviation in local x and y direction due to orientation and velocity respectively
		float orientation_error_stddev = PREDICTION_LOOKAHEAD_DISTANCE * tan(orientation_stddev);

		float velocity_error_stddev = velocity_stddev * dt;
		
		// sample from the gaussian (in local coords)
		// std::normal_distribution<float> noise_distribution_x(0, orientation_error_stddev);
		// std::normal_distribution<float> noise_distribution_y(0, velocity_error_stddev);

		std::uniform_real_distribution<float> noise_distribution_x(-velocity_error_stddev, velocity_error_stddev);
		std::uniform_real_distribution<float> noise_distribution_y(-velocity_error_stddev, velocity_error_stddev);
	
		for (size_t i = 0, len = particles_.size(); i < len; i++)
		{
			if (particles_[i].getWeight() < TRANSPORT_PARTICLE_PROBABILITY_EPSILON)
			{
				// resample from measurement
				particles_[i].sampleFromMeasurementGaussian(
					current_target_position, current_target_position_stddev, 
					velocity, velocity_stddev,
					orientation, orientation_stddev,
					1.0/num_particles_, dt,
					noise_distribution_x, noise_distribution_y, random_generator_
				);
			}
		}
	}

	resamplePredictionParticle();

	if (std::isfinite(velocity))
	{
		velocity_ = velocity;
	}
	else
	{
		ROS_ERROR("Infinite velocity");
	}

	if (std::isfinite(orientation))
	{
		orientation_ = orientation;
	}
	else
	{
		ROS_ERROR("Infinite orientation");
	}

	if (std::isfinite(velocity_stddev))
	{
		velocity_stddev_ = velocity_stddev;
	}
	else
	{
		ROS_ERROR("Infinite velocity_stddev");
	}
	if (std::isfinite(orientation_stddev))
	{
		orientation_stddev_ = orientation_stddev;
	}
	else
	{
		ROS_ERROR("Infinite orientation_stddev");
	}



}

void PredictionParticleFilter::resamplePredictionParticle()
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

	std::vector<PredictionParticle> new_particles(num_particles_);

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
	for (PredictionParticle &particle: new_particles)
	{
		particle.setWeight(
			particle.getWeight() / new_total
		);
	}

	particles_ = new_particles;

}

void PredictionParticleFilter::init(	size_t num_particles, 
										cv::Point3f target_initial_position_estimate,
										float velocity, float velocity_stddev,
										float orientation, float orientation_stddev		)
{
	num_particles_ = num_particles;
	particles_.clear();
	particles_.reserve(num_particles_);

	velocity_ = velocity;
	orientation_ = orientation;

	velocity_stddev_ = velocity_stddev;
	orientation_stddev_ = orientation_stddev;

	std::uniform_real_distribution<float> prediction_noise(PREDICTION_LOOKAHEAD_DISTANCE*.7, PREDICTION_LOOKAHEAD_DISTANCE*1.5);

	for (size_t i = 0; i < num_particles_; i++)
	{
		
		float r_noise = prediction_noise(random_generator_);
        float theta_noise = (float)rand()/RAND_MAX * 2 * M_PI;

        cv::Point3f particle_state;
		particle_state.x = 0; // target_initial_position_estimate.x + r_noise * cos(theta_noise); 
        particle_state.y = 0; // target_initial_position_estimate.y + r_noise * sin(theta_noise);
        particle_state.z = 0;

		particles_.push_back( PredictionParticle(particle_state, 1) );
	}

	is_initialized_ = true;
}