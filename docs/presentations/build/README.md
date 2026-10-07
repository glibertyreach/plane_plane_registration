# Building the registration decks

`make_registration_deck.js` builds two decks from two content files (all slide
text and speaker notes) and the figure panels in `../assets/`:

| Content file | Deck |
|---|---|
| `registration_build_deck_content.json` | `../registration_procurement_build.pptx` (10 slides) |
| `registration_procedure_deck_content.json` | `../registration_test_procedure.pptx` (16 slides) |

Slides are built by the "id" field of each slide in the JSON; each id has one
builder in the script (tables share one helper that sizes the rows to their text;
`approach`, `board` and `residuals` share the figure-beside-a-list layout, and
`fixtures`, `board_build` and `runout` share the figure-card-with-icon-cards
layout). Edit a JSON to change wording; edit the constants at the top of the
script to change layout, sizes or colors. Figure pixel sizes are read when the
script runs, so a re-cropped figure needs no code change. The generator was
adapted from the stage-1 generator of `depth_calibration_from_spherical_target`;
the layout constants that differ from it (title size, column widths, row gaps)
carry a comment saying why.

1. Install the Node dependencies once, in this folder: `npm install`
2. From the repository root, build both decks:

   ```
   NODE_PATH=docs/presentations/build/node_modules \
   PPTX_SKILL_SCRIPTS=<folder holding apply_theme.js> \
       node docs/presentations/build/make_registration_deck.js
   ```

   or one deck, giving the content file and the output file:

   ```
   NODE_PATH=docs/presentations/build/node_modules \
   PPTX_SKILL_SCRIPTS=<folder holding apply_theme.js> \
       node docs/presentations/build/make_registration_deck.js \
           docs/presentations/build/registration_procedure_deck_content.json \
           docs/presentations/registration_test_procedure.pptx
   ```

   The script prints a WARNING when a table or a column of cards needs more
   room than its slide gives it.

3. Check the text: `python3 docs/presentations/archive/check_registration_content.py`
   (needs `markitdown[pptx]` and `python-pptx`) compares both decks with their
   JSON files (every string on its slide, notes equal to the JSON notes) and
   then checks the build deck's cost figures against
   `docs/procedures/build/costs.py`, their single source. It must report
   0 problems and a passing cost check.

4. Check the layout without rendering:
   `python3 docs/presentations/archive/check_registration_layout.py` warns about
   overlapping text boxes and text that the generator's own text-width model
   says will not fit its box. It estimates; a render is the final judge.
   Render checks (PDF, images) and other intermediate files go in `../archive/`.

`apply_theme.js` comes from the pptx skill used to write this generator; it
writes the theme's colors and fonts into the finished file. The script needs
it to run: point `PPTX_SKILL_SCRIPTS` at the folder holding a copy, or keep
the script's default path. The same skill's `scripts/office/validate.py` checks
a finished deck's package structure.

## Regenerating the figure panels

The panels in `../assets/` are crops of the procedure's figures in
`docs/procedures/figures/`. Regenerate them, from the repository root, with

```
python3 docs/presentations/build/make_assets.py
```

(needs `Pillow` and `numpy`). Regenerate the figures first if a figure changes.
The crop boxes are named constants at the top of the script, in pixels of the
source figures; the script erases each panel title, crops the panel to its
content and restores a 20 px margin on every side. `plan_example.png` is the
plan figure used whole. If a figure's layout changes, view the five outputs and
adjust the constants.
