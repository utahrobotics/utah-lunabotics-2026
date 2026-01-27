# Lunabot Ai
*image may be a bit out of date, but you can re generate it with the .get_graphviz function on the behavior tree*
<img width="6542" height="2422" alt="graphviz" src="https://github.com/user-attachments/assets/7b810fd6-5ea8-4093-b305-eca13cd12ab1" />



## Behaviors
* located at ai/behaviors
* Should not contain actual action implementations, only functions that return a ```Behavior<LunabotAction>```.
* **teleop_behavior() creates the root of the behavior tree, and everything branches from there.**

### The Big 3

1. **Manual** <br>
Sets motors and actuators to whatever the lunabase says.
2. **Autonomy** <br>
Contains behaviors for navigate, dig, and dump operations. (or at least it eventually will)
3. **Software Stop** <br>
Continuously commands motors and actuators to set speeds to 0, the starting state of the robot and usually where you get dumped if something goes wrong.

<br>

These 3 branches are activated from the teleop_behavior based on the ```IsSoftStop IsAutonomy IsManual``` actions which check the ```current_mission``` field of the bt which gets updated if the lunabase asks to switch modes, or if the mode is manually switched with the ```SetStage``` action. 


### Helper behaviors
* located at ai/behaviors/helper_nodes.rs

I have taken inspiration from [nav2](https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html)'s behavior trees and implemented some helper nodes for autonomy that will be useful. 
_(currently just a retry node but more to come if we need them)._

## Actions
* located in ai/action.rs
### A Few Caveats
1. Each action when polled needs to return instantly, you should not make a complicated synchronous action that takes a while to run.

Example of what not to do:
```rust
LunabotAction::IsObstacleMapReady => {
    thread::sleep(Duration::from_secs(5)); // think for a while
    if blackboard.latest_local_map.is_some() {
        Success
    } else {
        Failure
    }
}
```


Instead if you have an action that will run for a long time it should be an asynchronous, easily cancelable Job, more on that later.

2. Zero-tick logic is weird. If an action immediately returns Success, then in the eyes of the behavior tree no time has passed. Each tick of bt is  greedy so you can end up with situations where an action gets called over and over in an infinite loop because the bt doesn't know to re check a condition because *technically* no time has passed. If infinite greedy 0-tick logic becomes a problem you can use the LunabotAction::Yield.


## Long running Jobs
* located in ai/jobs
* long running Jobs are launched by actions in ```actions.rs```

Theoretically we could implement all the autonomy purely with a bt where all the actions are extremely simple unit blocks like set steering, but that would be really annoying, hence the long running job concept.
<br>

Long running actions are created with the ```Job::spawn``` method where you pass an async ```Future```, and the resulting struct has helper methods for polling the jobs status, and getting any output information. The long running job's status (Success, Running, or Failure) is determined by a tokio::watch channel, and the outputs are also a thread channel like so:


```rust
/// outputs forward steering commands for n seconds
pub fn forward_for(seconds: f32) -> Job<Steering>{
    // create status channel with initial value of Running
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    // output steering commands sent through this channel
    let (output_tx, output_rx) = mpsc::channel(5);

    // you pass the rx side of the channels to the spawn function, and the tx sides are used in the async body
    Job::spawn(async move {

        let _ = tokio::time::timeout(Duration::from_secs_f32(seconds), async {
            loop {
                output_tx.send(Steering::new(1.0, 1.0, 1200.0)).await.ok();
                // add a little sleep to avoid backpressure
                tokio::time::sleep(Duration::from_millis(20)).await;
                // we could update the status here but it isnt actually necessary because our initial value was running
                status_tx.send(Running).ok();
            }
        }).await;

        // I made it so the async body must return a Status to prevent logic bugs where the status gets stuck in running if the developer forgets to set it.
        Success
    }, status_rx, output_rx)
}
```

And then in our behavior tree tick we can handle the action like so:


```rust
// --- snip ---
    LunabotAction::HardcodedForward(seconds) => {
        if let Some(ref mut path_follower) = blackboard.path_follower {
            // if the job is running grab the output and store it in the blackboard.
            blackboard.outgoing_steering_msg = path_follower.get_output();
            let status = path_follower.get_status();
            if status == Success || status == Failure {
                println!("Follow path job completed with status: {:?}", status);
                // call cancel just in case
                // it is helpful for the long running jobs to be async because we can just abort the thread body if we need to.
                // in software stop, we actually cancel any zombie jobs that happen to be running just in case.
                path_follower.cancel();
                blackboard.path_follower = None;
            }
            status
        } else {
            let mut job = forward_for(*seconds);
            let initial_status = job.get_status();
            blackboard.path_follower = Some(job);
            initial_status
        }
    }
// --- snip ---
```

that code will start a new async path follower if the old one has exited, and if one already exists then it grabs the output and sets ```blackboard.outgoing_steering_msg``` where the outgoing steering msgs will be consumed in the process() function of the lunabot ai task.

One must be careful to avoid spamming too many steering messages or they will not be consumed fast enough.

### You don't always need a long running job: 
A common pattern is to have something that runs for a certain amount of time, which can be achieved with the tokio::select! macro as shown in the hardcoded forward above, but you can easily achieve similar things just using normal behaviors and actions:

```rust
pub fn hardcoded_forward() -> Behavior<LunabotAction> {
    While(
        Box::new(Behavior::Wait(2.0)), // while loop body executes over and over so long as this behavior reports 'Running'
        vec![
            Action(LunabotAction::SetSteering(Steering::new(-0.5, 0.5, 1.0))),
            Action(LunabotAction::Yield),
        ],
    )
}
```
