#!/usr/bin/env python

TOTAL_ITERATIONS = 10000
import random

weights = [1, 40, 5, 10, 4]

total = 0
cumulative_probabilities = []

for weight in weights:
	total += weight
	cumulative_probabilities.append(total)

sample_count = [0 for i in range(len(weights))]

for iter_idx in range(TOTAL_ITERATIONS):
	rand_choice = random.uniform(0, total)
	for weight_idx in range(len(weights)):
		current_cumulative_weight = cumulative_probabilities[weight_idx]
		if rand_choice <= current_cumulative_weight:
			# sample chosen
			sample_count[weight_idx] += 1
			break



print "sample count: ", sample_count

print "Initial distribution", map(lambda x: float(x)/total, weights)
print "Final distribution", map(lambda x: float(x)/TOTAL_ITERATIONS, sample_count)
