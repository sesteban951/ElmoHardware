# Dependencies

### Simple Open EtherCAT Master Library

Install ```soem``` from ```https://github.com/OpenEtherCATsociety/SOEM```

### Eigen

Install ```Eigen3``` from ```eigen.tuxfamily.org```

### Yaml-CPP

Install ```yaml-cpp``` from ```https://github.com/jbeder/yaml-cpp```

# Compatibility with ROS
The simplest way to run this code with ROS wrapped around it is to run your ROS file with ```root```. That is, go into root mode, ```sudo -s``` and then ```rosrun``` or ```roslaunch```. 

An alternative is using ```ethercat_grant``` to remove the ```sudo``` or ```root``` requirements in any environment.