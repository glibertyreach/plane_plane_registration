"""planereg: gathering and analysis of the sample set that tests the registration of a
fixed 3D sensor into the robot base frame from plane-plane correspondences.

Subpackages:
    core      shared mathematics: plane algebra, the registration solver, target-plane
              segmentation (used by both of the others; imports neither)
    capture   the data-gathering tools run on the capture computer (pose plan, bootstrap,
              quick check); imports core only
    analysis  the engineer's tools (registration, residual maps, report, simulator);
              imports core only

The two tool subpackages never import each other, so either can be shipped alone with core.
"""
