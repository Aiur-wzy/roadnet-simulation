# TODO: Distinguish Running vs Waiting-Buffer Flow Features

## Purpose

The simulation does not yet have a clean implementation for distinguishing vehicles that are still physically running on a road segment from vehicles that have already reached the downstream waiting buffer.

This distinction may be important for future model-based travel-time prediction.

## Current context

The current simulation maintains basic flow features through `RoadSegment` state, including:

- `road_flow`
- `lane_flow`

These values are now used to build `BasicRoadModelFeatures` for future travel-time model prediction.

For now, `road_flow` and `lane_flow` represent total road/lane occupancy. That total occupancy may include both vehicles still running on the road segment and vehicles that have already arrived at the downstream waiting buffer.

## Unresolved design issue

For future model prediction, we may want to distinguish between:

1. vehicles physically running on the road segment;
2. vehicles that have already arrived at the downstream waiting buffer;
3. total occupancy.

Vehicles already queued in the waiting buffer still occupy storage, but their effect on the travel speed of a newly entering vehicle may not be the same as the effect of vehicles actively running on the road segment.

A future travel-time model may therefore want separate features such as:

- `road_running_flow`
- `lane_running_flow`
- `road_waiting_buffer_flow`
- `lane_waiting_buffer_flow`
- `effective_road_flow`
- `effective_lane_flow`

We have not yet found a satisfactory implementation for this separation.

## Rejected or unsatisfactory options

### Option 1: Scan `waitingBuffer.vehicleQueue` during feature construction

For each prediction, iterate over vehicles in the buffer and count vehicles with `arrivalTime <= current_time`.

Problems:

- This is `O(n)` per prediction, which is expensive and inelegant.
- It mixes model-feature construction with waiting-buffer queue internals.
- The current `waitingBuffer` may contain both actually arrived vehicles and future-arrival vehicles used for scheduling order.

### Option 2: Maintain an additional sorted arrival-time array per buffer

Maintain a separate sorted arrival-time structure for each buffer so feature construction can binary-search for vehicles with `arrivalTime <= current_time`.

Problems:

- It requires extra insert, delete, and lazy-deletion logic.
- It risks inconsistency with the real FIFO queue.
- It is hard to maintain correctly when vehicles move, finish, or have repaired arrival times.

### Option 3: Global `WaitingArrivalEvent` priority queue with lazy validation

When a vehicle enters a road, schedule an event at its predicted waiting-buffer arrival time. When the event fires, convert running flow into waiting-buffer flow.

Problems:

- It introduces another event queue and another state-transition layer.
- It may be acceptable later, but feels too heavy before the running/pending road-state design is finalized.

### Option 4: Introduce explicit `RunningOnRoad` / `RoadArrivalEvent` state

This is probably the cleanest long-term design. Vehicles would not be inserted into the waiting buffer until they actually arrive. Then `waitingBuffer.vehicleQueue` would contain only physically arrived vehicles.

Problems:

- It requires a larger simulation refactor.
- It may affect dispatch scheduling, FIFO assumptions, and arrival-time repair logic.

## Current decision

Do not implement this separation yet.

Keep the current `BasicRoadModelFeatures` as-is:

- `road_length`
- `turn_type`
- `road_flow`
- `lane_flow`
- necessary simulation parameters

Keep `road_flow` and `lane_flow` defined as total occupancy for now.

## Future direction

Revisit this after the travel-time model integration is working with the basic features.

If model performance indicates that total flow is too crude, design a cleaner state model to separate running flow from waiting-buffer flow.

## TODO

- Decide whether to keep the current early-insertion `waitingBuffer` design or refactor to explicit `RoadArrivalEvent` state.
- Decide whether model features should use total flow, running flow, waiting-buffer flow, or weighted effective flow.
- Ensure any future solution does not require frequent expensive search/delete operations on `waitingBuffer`.
- Ensure any future solution preserves FIFO and movement-based dispatch semantics.
- Add tests for flow-accounting invariants if this separation is implemented.
