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


# Execute the main command as the user using su
if [[ -z "$@" ]]; then
  exec su user -P -c "/bin/bash"
else
  exec su user -P -c "exec $@"
fi
