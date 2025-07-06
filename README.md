# MakSafe: Haskell Prototype & Experiment (SEFM 2025)

**MakSafe** provides a formal framework for defining and evaluating spatiotemporal relations concerning road vehicles within a road network, enabling the specification and verification of road vehicle safety rules.

## Prototype Structure

The implementation comprises two conceptual layers:

- **` Abstract.hs `** – Specifies the *abstract layer*, formalizing the core components of the MakSafe framework.
- **` Concrete.hs `** – Instantiates the abstract layer for a concrete case study focused on vehicles. This layer encodes domain-specific specifications, including:
  - Front vehicle identification;
  - Time-to-collision (TTC) threshold verification regarding front vehicles.

## Experimental Setup

We define three lanes: `firstLane`, which splits into `leftLane` and `rightLane`. Two linkage transitions connect `firstLane` to `leftLane` and `rightLane`, respectively.

The road network contains three vehicles with the following initial positions:  
- The **ego vehicle** is on `firstLane`, approaching the split endpoint.
- The **second vehicle** (left) is near the start of `leftLane`.
- The **third vehicle** (right) is near the start of `rightLane`.

The second and third vehicles are equidistant from the ego vehicle; however, the second vehicle moves slower than the third.

### Verification Goal

We verify if the ego vehicle violates the **time-to-collision threshold** of **2 seconds** relative to its front vehicles.

### Steps

1. **Front Vehicle Identification**

   Using the framework implementation, the system identifies front vehicles for the ego vehicle within the 2-second threshold. Both the second and third vehicles qualify as front vehicles, each corresponding to different ego vehicle trajectories:
   - The second vehicle (left) is a front vehicle if the ego assumes the left lane.
   - The third vehicle (right) is a front vehicle if the ego assumes the right lane.

2. **Time-to-Collision Verification**

   The time-to-collision threshold is violated for the second vehicle but not the third, as the third vehicle moves faster. This identifies the action trace leading the ego vehicle to violate the threshold and specifies the corresponding front vehicle.

## Experiment Implementation

The experiment’s code is contained in **`Main.hs`**. To run the executable, **`test`** execute the following command from the experiment directory (tested on Ubuntu 24.04 LTS):

```bash
./test
