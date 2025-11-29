CONVINCE sit-aw-anchoring, a semantic anchoring module
======================================================

sit-aw-anchoring is a ROS 2 based implementation of the semantic anchoring process. Semantic anchoring is a process that integrates numerical data coming from perception with information in digital twin(s) to generate symbolic predicates in a planning language (PDDL, HTN, etc.) for a task planner, which can then compute an action plan for a given goal. It is a crucial process in a robot situation awareness pipeline.

In this documentation we introduce the generic principles of semantic anchoring. Then, we describe how to build and run sit-aw-anchoring. Finally, we provide a tutorial to see sit-aw-anchoring in action in a simple simulated scenario of a robot arm moving cubes.

Contents
--------

.. toctree::

   generic_principles
   installation
   tutorials
