# Particle Filter Localization

This is my submission project for Udacity's self-driving car program related to the 
[Kidnapped Vehicle Project](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).  There
are no surprises on how to build it, and there are no surprises on how to run it.  

The general idea of the solution is as follows:

 0. Initialize particles with initial position, orientation and noise
 1. Update particle position based on current particles, their orientation and noise  
 2. For each particle, determine landmarks within sensor range
 3. Map particle observations to map coordinates
 4. Compute particle weight using vectors to the observation and landmark (distance and angle error)
 5. Resample particle weights
 6. Repeat 1 - 5 for each observation 

I found that using 50 particles is reliable in estimating the positions within the
tolerances desired for the project.  That is to say, I have run it >10 times in a row
with no failures.  10 particles does fail sporadically with my code, and there is a non-zero chance
that 50 would as well, just more remote.  My turn-in project uses 100 particles just for added 
safely.  Here is what it produces using 50 particles and some compiler optimization (noted below):

![Performance](performance.png)

The algorithm was implemented extremely simply in an imperative style without a lot of attention to 
performance since it performs well within the expected time.  However, there is 
a lot of opportunity for improvement. Some of the problems with loop unrolling, inlining, etc.,
are addressed quite respectably by using -O3 optimization.  This is easy to add to the CMake
file, like this for GCC:

```
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3") 
    set(CMAKE_EXE_LINKER_FLAGS "-s") 
endif()
```

Having decent optimization can make a big difference in compute time, easily availing the use
of 1000 or more particles.  Or you can just enjoy that a reasonable number of particles runs 
really quickly.

I decided to use the Mersenne Twister generator because it is a better linear conguence generator, 
albeit potentially slightly more expensive.  In the update step, while it is certainly more 
accurate to use the full computation including yaw rate for the update, it actually doesn't make a huge 
difference in the outcome; it speeds it up, albeit fairly negligably, and the error introduced 
into the particle's positions is small in comparison with the noise.  Probably best to leave it 
in the more correct form, but given the noise and tolerance of this project it isn't really 
required.  

I implemented the "resampling wheel" algorithm because it is clever and interesting.  The same 
effect can also be achieved by using the `discrete_distribution` like this (in place of the 
resampling code in the project):

```
std::vector<Particle> new_particles;
std::vector<double> new_weights;
std::random_device rd;
std::mt19937 gen(rd());
std::discrete_distribution<> d(weights.begin(), weights.end());
for (int i=0; i<num_particles; i++)
{
  int index = d(gen);
  Particle p = {i, particles[index].x, particles[index].y, particles[index].theta, 1.0};
  new_particles.push_back(p);
  new_weights.push_back(1.0);
}
```

This is faster than the resampling wheel, and the code is more concise.
