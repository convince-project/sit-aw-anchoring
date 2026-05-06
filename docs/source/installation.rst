Run
===

SIT-AW-ANCHORING is packaged as docker images for ROS 2 `jazzy` and `humble` distributions at the `sit-aw-anchoring repository <https://github.com/convince-project/sit-aw-anchoring/pkgs/container/sit-aw-anchoring>`_

Assuming that `<path-to-sit-aw-anchoring>` is the path to the root folder of SIT-AW-ANCHORING on your filesystem, use the following command to start it:

.. code-block:: bash

    cd <path-to-sit-aw-anchoring>/docker
    export UID
    export GID="$(id -g)"
    docker compose up sit-aw-anchoring -d

To connect to the container from new terminals on the host, use these commands:

.. code-block:: bash

    cd <path-to-sit-aw-anchoring>/docker
    export UID
    export GID="$(id -g)"
    docker compose exec -u $UID:$GID -it sit-aw-anchoring /bin/bash


