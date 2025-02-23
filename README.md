
This repository contains code for the controllers to be cross-compiled for the Khepera IV.

Things that are simulated:
- degradation of ground sensor readings (the actual readings are still being read from the physical sensor)

Things that aren't simulated:
- communication, via WiFi
- obstacle avoidance
- led (may consider turning this off to improve robot battery life)

### Docker build instructions
Do it from the project's root directory

First build the base (containing ARGoS, Buzz, and the `argos3-kheperaiv` plugin)
```
docker image build -t nestlab/kheperaiv --file docker/DockerfileBase .
```

Then build the controller 

### dev notes
what changed:
- std::accumulate for std::reduce (only supported in c++17)
- convert double to float