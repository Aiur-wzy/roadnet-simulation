# TODO: Time Repairing and Waiting-Buffer Length Trimming

This document records two unresolved future simulation-design issues:

1. Time repairing / arrival compensation for vehicles entering a downstream road.
2. Lane-level waiting-buffer road-length trimming.

This document is intentionally a TODO only. No simulation behavior is changed in this commit.

## Context

In the current cycle-aware simulation, when a vehicle is discharged from an upstream movement into a downstream road, the simulation estimates the vehicle's `arrivalTime` at the downstream waiting buffer using the current travel-time logic.

That estimate is only an approximation. At the moment the vehicle enters the downstream road, the downstream waiting queue has some observed length. Before the vehicle actually reaches the waiting buffer, vehicles already in that downstream waiting queue may be discharged by the traffic signal. This shortens the queue and moves the effective tail of the waiting buffer forward. As a result, the entering vehicle may need to travel a longer distance than originally estimated, so the originally computed `arrivalTime` may be too early.

We are intentionally not implementing a new compensation mechanism now because the proposed mechanism may be numerically equivalent to the current temporary logic in many cases. This design should be revisited only after the basic simulation, movement-based dispatch, and model-based travel-time prediction paths are stable.

## Problem 1: Time Repairing / Arrival Compensation

### Issue

A vehicle can be discharged into a downstream road at time `t_release` and assigned an estimated waiting-buffer arrival time `t1`. This estimate may be based on the queue length observed at road entry. However, before the vehicle reaches the waiting buffer, the queue ahead may shrink. Therefore, the actual distance the vehicle needs to travel before reaching the waiting buffer can increase.

This creates an `arrivalTime` underestimation problem. A future `Time_repairing` mechanism may be needed before the vehicle is actually allowed to pass the next signal.

### Possible Vehicle-Level Fields

Future vehicle-level fields may include:

- `needsTimeRepairing` / `repairFlag`:
  - `1` means the vehicle must go through one compensation check before it can be discharged from the next waiting buffer.
  - `0` means it can be discharged normally if other signal, capacity, and downstream conditions allow it.
- `travel_length`:
  - The distance already considered or traveled when estimating arrival at the waiting buffer.
  - This is used to compute the remaining compensation distance.
- `repairedArrivalTime` or `compensatedArrivalTime`:
  - Optional field used to record the corrected arrival label.

### Possible Future Logic

When a vehicle enters a new downstream road:

1. Set `repairFlag = 1`.
2. Record `travel_length` for the initially assumed travel distance before reaching the waiting buffer.
3. The vehicle may still be inserted into the downstream waiting-buffer ordering structure according to the current simulation design, but it cannot be truly discharged while `repairFlag == 1`.

When the vehicle is considered for discharge at time `t`:

1. If `repairFlag == 0`, process normal signal, capacity, and downstream checks, then discharge normally.
2. If `repairFlag == 1`, compute the remaining compensation distance:

   ```text
   remaining_distance = road_length - travel_length
   ```

3. Compute the compensation travel time:

   ```text
   t2 = travelTime(remaining_distance)
   ```

4. Let `t1` be the originally estimated waiting-buffer arrival time.
5. Compare the current time `t` with `t1 + t2`.

#### Case A: Compensation Already Satisfied

If `t >= t1 + t2`:

1. Set `repairFlag = 0`.
2. Allow the vehicle to continue the normal discharge logic immediately.

#### Case B: Compensation Still Needed

If `t < t1 + t2`:

1. Set `repairFlag = 0`.
2. Update the vehicle's `arrivalTime` label to `t1 + t2`.
3. Reinsert or reschedule the corresponding movement/road candidate in the movement-based dispatch priority queue.
4. Do not discharge the vehicle yet.

The movement should be rescheduled based on:

```text
max(movementTimeLabel, vehicle.arrivalTime)
```

### Constraints

Any future implementation must preserve the following constraints:

- The real FIFO waiting-buffer queue must not be broken.
- The vehicle must not be removed from the real queue unless it is truly discharged.
- Compensation must interact cleanly with the existing movement-based dispatch priority queue.
- The mechanism must not reintroduce vehicle-based global priority-queue scheduling.

## Problem 2: Waiting-Buffer Road-Length Trimming

### Issue

The current travel-time calculation may treat the downstream road as if the vehicle always travels the full road length. If the downstream waiting buffer already occupies part of the road, the effective running distance before joining the waiting queue should be shorter than the full road length:

```text
effective_travel_length = road_length - waiting_buffer_occupied_length
```

This should not be road-level only. It should be lane-level.

### Why Lane-Level Trimming Is Needed

Different lanes can have different waiting-buffer lengths because:

- Different movements use different lane groups.
- Left, straight, and right movements may have separate queues.
- Some lanes may be full while others are not.
- Lane-level storage and lane-level waiting queues are already relevant to the simulation.

### Future Direction

A future implementation should support lane-level waiting-buffer trimming:

1. For each road/lane or lane group, maintain the current waiting-buffer occupied length.
2. When a vehicle enters a downstream road and is assigned a target lane, compute:

   ```text
   effective_length_for_lane = road_length - waiting_buffer_length[lane]
   ```

3. Use this lane-specific effective length for the predicted travel time when entering the road.
4. If the waiting queue shrinks before the vehicle reaches the buffer, `Time_repairing` may need to compensate for the increased effective travel distance.
5. Integrate this with the selected downstream lane from `chooseLeastOccupiedAvailableLane` or with the movement's allowed lane group.

### Possible Future Fields

Possible future fields include:

- `laneWaitingBufferLength`
- `laneWaitingVehicleCount`
- `vehicle.travel_length`
- `vehicle.repairFlag`
- `vehicle.repairedArrivalTime`
- Movement/lane discharge logs, if needed

### Potential Alternative Approach

Instead of per-vehicle repairing, maintain movement/lane-level discharge logs:

- Record discharge time.
- Record released queue length.
- Record lane index or lane group.
- When a vehicle first becomes eligible for discharge, scan or query the logs to repair `arrivalTime`.

This may be more complex and should not be implemented until needed.

## Open TODOs

- Decide whether `Time_repairing` should be per-vehicle flag based or movement/lane discharge-log based.
- Decide how to define `travel_length` precisely.
- Decide whether `t2` should be computed by current formula-based travel time, kinematic mode, or future model-based travel time.
- Decide how lane-level waiting-buffer length should be represented.
- Decide how this interacts with `BasicRoadModelFeatures` and future model-based prediction.
- Ensure any future solution preserves FIFO queue semantics.
- Ensure any future solution does not convert the global dispatch priority queue back into a vehicle-based priority queue.
- Add tests for:
  - Queue shrinking before vehicle arrival.
  - Lane-level waiting-buffer trimming.
  - Compensation delaying a vehicle that was previously estimated too early.
  - No compensation needed when `t >= t1 + t2`.
  - No vehicle is removed from the waiting buffer on failed compensation.
