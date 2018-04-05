#eece5698-final

## Setup Notes 

### Remote PC Side
- Need to be running the NTP server to allow the TurtleBot to synchronize its 
timing when offline
- Ensure `$ROS_MASTER_URI` is set to the ip address of your machine
- Run `roscore` on the remote pc side. The remote machine is the master node.

### TurtleBot Side
- Ensure `$ROS_MASTER_URI` is set to the ip address of your remote machine
- Ensure the date and time are correclty set (i.e. synchronized) with the master
node

