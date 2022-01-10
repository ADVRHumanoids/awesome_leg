# URDF description(s) of the leg

Files:

- `awesome_leg_fixed.xacro` &rarr Imports awesome_leg.urdf 

- `awesome_leg_sliding_all.xacro` &rarr Imports awesome_leg.urdf and adds the test rig and its PRISMATIC joint. In this case, the test rig is connected to the world with an additional PRISMATIC horizontal joint

- `awesome_leg.xacro` &rarr Imports awesome_leg.urdf and adds the test rig and its PRISMATIC joint

- `awesome_leg_complete_fixed.urdf` &rarr Complete URDF with fixed hip (no plugins). Used by the RViz launch file/s
 
- `awesome_leg_complete_sliding_all.urdf` &rarr Complete URDF with sliding hip and sliding test rig (no plugins). No model plugins loaded. Used by the RViz launch file/s

- `awesome_leg_complete.urdf` &rarr Complete URDF of the leg (+ test rig and world link). No model plugins loaded. Used by the RViz launch file/s

- `awesome_leg.urdf` &rarr Leg base URDF (no external links and model plugins). No model plugins loaded

