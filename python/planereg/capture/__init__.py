"""
The data-gathering tools of planereg, run on the capture computer (code design, Section 7):

    plan_poses       plan the robot poses of a registration session, with the approach pose of
                     every target pose for the "minimizing backlash" sub-procedure
    bootstrap        find the sensor roughly from a few hand-jogged board captures
    check_captures   quick check of a capture set, with the report of the joint motion signs

Each is run as ``python3 -m planereg.capture.<tool>`` and exposes ``main(argv) -> int`` with the exit
codes 0 (ok), 1 (something flagged) and 2 (an input has to be fixed). The helpers they share are in
``planereg.capture.common``. This subpackage imports ``planereg.core`` and sphcal only, never
``planereg.analysis``.
"""
