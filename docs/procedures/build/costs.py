"""Single source of the cost figures quoted in the registration capture procedure and its decks.

Every estimate is a range in US dollars, as of the date below, taken from the stage-1 procedure
of depth_calibration_from_spherical_target (suppliers' list prices where these exist, otherwise
typical United States job-shop rates of about 80 to 150 dollars per hour). Treat every figure as
plus or minus 50 percent and get quotes.

Used by fill_procedure.py (markers {{COST_TABLE_BUILD}}, {{COST_TABLE_BUY}}, {{COST_PARAGRAPH}})
and, as JSON, by the deck content; run it directly to print the tables:

    python3 docs/procedures/build/costs.py            # Markdown tables and paragraph
    python3 docs/procedures/build/costs.py --json     # the same figures as JSON
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass

ESTIMATE_DATE = "October 2026"
UNCERTAINTY_PERCENT = 50
"""Every figure is plus or minus this much."""
JOB_SHOP_RATE_USD_PER_HOUR = (80, 150)
"""Basis of the machining estimates."""


@dataclass(frozen=True)
class CostItem:
    name: str
    low: int
    high: int
    purpose: str
    in_hand_from_stage_1: bool = False
    """True for an item the stage-1 session already produced; it is listed, but costs nothing more
    when stage 1 has been run."""


BUILD_ITEMS = (
    CostItem("Flat target (board)", 50, 400, "200 x 150 mm plate, specification in 1d", True),
    CostItem("Board adapter", 250, 600, "Flange plate with pads, edge pins and clamp fingers, drawing SC1-05", True),
    CostItem("Run-out fixture", 240, 570, "Dial indicator on a magnetic base on a bolted steel plate, section 1e", True),
)
BUY_ITEMS = (
    CostItem("Calipers with depth rod, 150 mm", 30, 150, "Board distance D; board size", True),
    CostItem("Precision straightedge, 300 mm, and feeler gauges", 75, 240, "Board flatness after painting and mounting", True),
    CostItem("Thermometer", 15, 40, "Room temperature in the session notes", True),
    CostItem("Machinist's square", 30, 80, "Board x axis against the adapter when the tool frame is set", True),
    CostItem("Consumables", 20, 60, "Isopropyl alcohol and wipes, shim stock for the adapter", False),
)


def total(items) -> tuple[int, int]:
    return sum(i.low for i in items), sum(i.high for i in items)


def total_if_stage_1_done(items) -> tuple[int, int]:
    rest = [i for i in items if not i.in_hand_from_stage_1]
    return total(rest)


def usd(value: int) -> str:
    return f"${value:,}"


def usd_range(low: int, high: int) -> str:
    return f"{usd(low)} to {usd(high)}"


def build_table() -> str:
    rows = ["| Item | Quantity | Description | Drawing | Estimated cost (USD) | From stage 1 |",
            "|---|---|---|---|---|---|"]
    drawing = {"Flat target (board)": "SC1-05 (outline)", "Board adapter": "SC1-05", "Run-out fixture": "none needed"}
    for i in BUILD_ITEMS:
        rows.append(f"| {i.name} | 1 | {i.purpose} | {drawing[i.name]} | {usd_range(i.low, i.high)} | "
                    f"{'yes' if i.in_hand_from_stage_1 else 'no'} |")
    return "\n".join(rows)


def buy_table() -> str:
    rows = ["| Item | Purpose | Estimated cost (USD) | From stage 1 |", "|---|---|---|---|"]
    for i in BUY_ITEMS:
        rows.append(f"| {i.name} | {i.purpose} | {usd_range(i.low, i.high)} | "
                    f"{'yes' if i.in_hand_from_stage_1 else 'no'} |")
    rows.append("| Phone camera | Setup photos for the deliverables (section 9) | in hand | yes |")
    return "\n".join(rows)


def cost_paragraph() -> str:
    b_lo, b_hi = total(BUILD_ITEMS)
    y_lo, y_hi = total(BUY_ITEMS)
    r_lo, r_hi = total_if_stage_1_done(BUILD_ITEMS + BUY_ITEMS)
    return (f"Cost estimates ({ESTIMATE_DATE}): if the stage-1 session has been run, everything above is in hand "
            f"and the new spending is {usd_range(r_lo, r_hi)} for consumables. Built from nothing, the items "
            f"come to {usd_range(b_lo, b_hi)} to build and {usd_range(y_lo, y_hi)} to buy, "
            f"{usd_range(b_lo + y_lo, b_hi + y_hi)} in all. The figures are the stage-1 procedure's, from "
            f"suppliers' list prices where these exist (appendix A) and otherwise from typical United States "
            f"job-shop rates of about {usd(JOB_SHOP_RATE_USD_PER_HOUR[0])} to {usd(JOB_SHOP_RATE_USD_PER_HOUR[1])} "
            f"per hour. Treat them as plus or minus {UNCERTAINTY_PERCENT} percent and get quotes; the board adapter "
            f"is the least certain figure.")


def as_json() -> dict:
    b = total(BUILD_ITEMS); y = total(BUY_ITEMS); r = total_if_stage_1_done(BUILD_ITEMS + BUY_ITEMS)
    return {"estimate_date": ESTIMATE_DATE, "uncertainty_percent": UNCERTAINTY_PERCENT,
            "build": [i.__dict__ for i in BUILD_ITEMS], "buy": [i.__dict__ for i in BUY_ITEMS],
            "build_total": usd_range(*b), "buy_total": usd_range(*y), "total": usd_range(b[0] + y[0], b[1] + y[1]),
            "new_spending_if_stage_1_done": usd_range(*r)}


if __name__ == "__main__":
    if "--json" in sys.argv:
        print(json.dumps(as_json(), indent=1))
    else:
        print(build_table()); print(); print(buy_table()); print(); print(cost_paragraph())
