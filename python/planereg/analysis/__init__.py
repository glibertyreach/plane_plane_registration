"""The engineer's tools of planereg: registration of a capture session, residual figures,
comparison of two sessions, the Markdown report and the synthetic session generator.

Every tool is a command-line module with ``main(argv) -> int``:

    python3 -m planereg.analysis.register       manifest (+ plan: held-out poses) -> registration.json,
                                                segmentation.npz, figures
    python3 -m planereg.analysis.residual_maps  registration.json -> figures (also called by register)
    python3 -m planereg.analysis.compare        two registration.json files -> comparison.json / .png
    python3 -m planereg.analysis.report         registration.json (+ comparison.json) -> report.md
    python3 -m planereg.analysis.simulate       a synthetic session in the layout of a real one

Exit codes: 0 success, 1 the result is flagged (a registration that did not pass its acceptance
thresholds), 2 an input that must be fixed.

This package imports ``planereg.core`` and sphcal only; it never imports ``planereg.capture``.
"""
