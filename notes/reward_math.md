# Reward Function Math: Overcut and Overfill

This note explains the mathematical symmetry of the `r2` and `r4` reward components used in the DozeRL environment.

### Definitions
* `H_goal`: The target goal heightmap (`grid_G`)
* `H_minus`: The terrain height before the action
* `H_plus`: The terrain height after the action
* `dH = H_plus - H_minus`: The change in height caused by the action (Negative = Cut, Positive = Fill)
* `H_cfm = H_minus - H_goal`: How far the previous height was from the goal
* `H_cfp = H_plus - H_goal`: How far the current height is from the goal

### Overcut / Bad Cut (`r2`)
**Equation:** `r2 += w_oc * min(0, max(dH, H_cfp))`

This equation mathematically handles **any action that puts the terrain below the goal state**. 
It perfectly covers two distinct scenarios:

1. **Digging too deep in a Cut region (Overcut):**
   * *Scenario:* You are 2m ABOVE the goal (`H_cfm = +2`), so you are supposed to cut. You accidentally cut 3m (`dH = -3`), leaving you 1m BELOW the goal (`H_cfp = -1`).
   * *Math:* `max(-3, -1)` is `-1`. `min(0, -1)` is `-1`. 
   * *Result:* You get penalized exactly `w_oc * -1` for the 1m you overcut.

2. **Digging in a Fill region (Wrong-way action):**
   * *Scenario:* You are 2m BELOW the goal (`H_cfm = -2`), so you are supposed to fill. You accidentally cut 1m (`dH = -1`), leaving you 3m BELOW the goal (`H_cfp = -3`).
   * *Math:* `max(-1, -3)` is `-1`. `min(0, -1)` is `-1`. 
   * *Result:* You get penalized exactly `w_oc * -1` for the 1m you wrongly cut.

Because it automatically penalizes any cut that leaves the terrain below the goal, `r2` single-handedly covers all cutting-related errors across the entire map.

### Overfill / Bad Fill (`r4`)
Because `r2` handles all bad cuts, `r4` acts as the exact mirror image to handle all bad fills.

**Equation:** `r4 += w_oc * min(0, max(-dH, -H_cfp))`

This equation mathematically handles **any action that puts the terrain above the goal state**.
It perfectly covers the opposite two scenarios:

1. **Adding too much dirt to a pile (Overfill):**
   * *Scenario:* You are 2m BELOW the goal (`H_cfm = -2`), so you are supposed to fill. You accidentally dump 3m of dirt (`dH = +3`), leaving you 1m ABOVE the goal (`H_cfp = +1`).
   * *Math:* `max(-3, -1)` is `-1`. `min(0, -1)` is `-1`.
   * *Result:* You get penalized exactly `w_oc * -1` for the 1m you overfilled.

2. **Dumping dirt in a Cut region (Wrong-way action):**
   * *Scenario:* You are 2m ABOVE the goal (`H_cfm = +2`), so you are supposed to cut. You accidentally dump 1m of dirt (`dH = +1`), leaving you 3m ABOVE the goal (`H_cfp = +3`).
   * *Math:* `max(-1, -3)` is `-1`. `min(0, -1)` is `-1`.
   * *Result:* You get penalized exactly `w_oc * -1` for the 1m you wrongly dumped.

Together, `r2` and `r4` form a mathematically symmetrical penalty system that prevents over-compensation and wrong-way actions everywhere in the environment.
