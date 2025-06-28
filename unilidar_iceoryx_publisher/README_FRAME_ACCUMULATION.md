# Frame Accumulation Feature

## Overview

The iceoryx publisher has been enhanced with frame accumulation capabilities, similar to Point LIO's `con_frame` mechanism. This allows combining multiple small point clouds (~5k points each) into larger accumulated clouds before sending them via iceoryx.

## Key Changes

### 1. Frame Accumulation Parameters
```cpp
constexpr bool ACCUMULATE_FRAMES = true;   // Enable/disable frame accumulation  
constexpr int ACCUMULATION_COUNT = 5;      // Number of frames to accumulate
```

### 2. Increased Point Cloud Capacity
- `MAX_POINTS_PER_CLOUD`: Increased from 6,000 to 30,000 points
- `IceoryxPointCloud.points[]`: Array size increased to 30,000

### 3. Temporal Coherence
The accumulator adjusts timestamps to maintain temporal coherence across accumulated frames, similar to Point LIO's approach.

## Configuration Options

### Enable/Disable Frame Accumulation
- Set `ACCUMULATE_FRAMES = true` to enable frame accumulation
- Set `ACCUMULATE_FRAMES = false` to use original behavior (send individual frames)

### Adjust Accumulation Count
- Modify `ACCUMULATION_COUNT` to change how many frames are accumulated
- Examples:
  - `ACCUMULATION_COUNT = 3`: ~15k points per cloud
  - `ACCUMULATION_COUNT = 5`: ~25k points per cloud  
  - `ACCUMULATION_COUNT = 10`: ~50k points per cloud (may exceed buffer)

## Expected Results

With the default settings (`ACCUMULATION_COUNT = 5`):
- **Before**: ~5,000 points per cloud, high frequency
- **After**: ~25,000 points per cloud, 5x lower frequency

This should match the denser point clouds you see in the unilidar 2 visualization software.

## Usage

1. Compile the publisher:
   ```bash
   cd unilidar_iceoryx_publisher && make
   ```

2. Run the publisher:
   ```bash
   ./unilidar_publisher
   ```

3. Check output messages:
   - With accumulation: "Accumulated 5 frames, total points: 25000"
   - Without accumulation: "publish_count: 5000"

## Performance Considerations

- **Memory**: Larger point clouds use more memory in iceoryx shared memory
- **Latency**: Higher latency due to accumulation delay (5x frame period)
- **Throughput**: Same total point throughput, but in larger batches

## Comparison with Point LIO

This implementation mirrors Point LIO's frame concatenation:
- Point LIO: `con_frame: true`, `con_frame_num: 5`
- Iceoryx Publisher: `ACCUMULATE_FRAMES = true`, `ACCUMULATION_COUNT = 5`

Both approaches create larger, denser point clouds by combining multiple sensor frames. 