#!/bin/bash

export USERNAME=user

# Set UID and GID if not already set
UID=${UID:-$(id -u)}
GID=${GID:-$(id -g)}

# Check if UID and GID variables are set
if [ -z "$UID" ] || [ -z "$GID" ]; then
    echo "UID and GID environment variables must be set."
    exit 1
fi

echo "Matching docker and host user permissions for shared volumes..."

# Modify the group ID of the group "user"
groupmod -o -g "$GID" "$USERNAME"

# Modify the user ID of the user "user"
usermod -u "$UID" "$USERNAME"

# Modify the permissions of /home/"$USERNAME" and its content to the new user ID and group ID of the user "user"
chown -R "$USERNAME":"$USERNAME" /home/"$USERNAME"

echo "Done."

# Start TypeDB server (terminal 1)
typedb server &

# Start DT environment (terminal 3)
cd /tmp/dt && python3 DT_Simulation.py &

# Launch pick_place_uc (terminal 5)
source /home/user/sit-aw-anchoring/colcon_ws/install/setup.sh
ros2 launch pick_place_uc pick_place_uc_launch.py &

# Setup the anchoring process (terminal 6)
ros2 lifecycle set /anchoring_process configure
ros2 lifecycle set /anchoring_process activate

ros2 action send_goal /anchoring_process/set_ontology anchoring_process_interfaces/action/SetOntology "{knowledge_domain: 'CubesWorld'}"
ros2 action send_goal /anchoring_process/populate_instances anchoring_process_interfaces/action/PopulateInstances "{knowledge_domain: 'CubesWorld', instances: '/tmp/dt/setup.json'}"

# Export DT data (terminal 4)
cd /tmp/dt
python3 update_json.py &

# Exectution phase (terminal 6) (must be done manually for now)
ros2 action send_goal /anchoring_process/update_state anchoring_process_interfaces/action/UpdateState "{knowledge_domain: 'CubesWorld', instances: '/tmp/dt/runtime.json'}"

# Start TypeDB studio (terminal 2)
cd /opt/typedb-studio/bin/
./typedb-studio 

# Execute the main command as the user using su
#if [ -z "$@" ]; then
#  exec su user -P -c "/bin/bash"
#else
#  exec su user -P -c "exec \"$@\""
#fi
