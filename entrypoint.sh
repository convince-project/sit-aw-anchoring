#!/bin/bash

export USERNAME=user

# Check if UID and GID variables are set
if [ -z "$UID" ] || [ -z "$GID" ]; then
    echo "UID and GID environment variables must be set."
    exit 1
fi

# Modify the group ID of the group "user"
groupmod -o -g "$GID" "$USERNAME"

# Modify the user ID of the user "user"
usermod -u "$UID" "$USERNAME"

echo "User '"$USERNAME"' UID and GID changed to $UID and $GID."

# Modify the permissions of /home/"$USERNAME" and its content to the new user ID and group ID of the user "user"
chown -R "$USERNAME":"$USERNAME" /home/"$USERNAME"
echo "Permissions of path '/home/"$USERNAME"' changed to $UID and $GID."

# Step down to the non-privileged user
exec su "$USERNAME" -P -c "exec \"$@\""
#exec "$@"
