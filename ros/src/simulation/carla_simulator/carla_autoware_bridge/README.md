# How to Setup Carla with Autoware

1. install nvidia-docker

2. build docker images of carla-bridge
```
cd $(rospack find carla_autoware_bridge)/docker
sh build.sh
```

3. Download carla simulator binary from Github Release Page(https://github.com/carla-simulator/carla/releases)  
Ver 0.9.3 is recomended.  

4. Extract it into $(carla_autoware_bridge)/bin directory  

```
cd $(rospack find carla_autoware_bridge)/bin
tar -xvzf CARLA_0.9.3.tar.gz
```