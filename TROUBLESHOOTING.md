# Trouble Shooting

Before spending a million years debugging, first check which git branch you are on and run:

1. ```git pull```
2. ```cargo update```



## Common issues

#### Weird error related to Iceoryx node or service creation

Sometimes there are artifacts from iceoryx that get left behind if a program crashes during data transfer, to fix it run ```rm /tmp/iceoryx2```.


#### Rerun window not opening

1. Check main.rs/sim.rs/resim.rs depending on the target you are running, and look for the call to init_rerun. Ensure that the parameters are correct for your situation.
2. Open a terminal and type rerun then press enter, if nothing happens that means the rerun binary is not in your PATH, follow operating system specific instructions to put the rerun binary in your path.
3. Try starting rerun manually, and then run ```make sim/resim/prod```, sometimes the rerun spawn doesn't work on windows but it will still connect if you spawn rerun yourself.


#### Other Rerun error messages and unexpected behavior

1. Ensure that the rerun sdk version in Cargo.toml matches the version that is reported by running ```rerun --version```


#### Log replay (resim.rs) says "No such File or directory"

1. Are there valid logs in ```lunabot-cu/logs```?


#### Other log replay (resim.rs) errors

1. Ensure that the logs you downloaded are compatible with the commit hash you are currently on, trying to replay logs recorded with different datatypes passed between tasks will result in weird obscure error messages. Logs downloaded from the lunabot-logs channel will always specify the commit hash that those logs are known to replay correctly with.


#### Invalid copperconfig.ron

The copper runtime has pretty useless error messages for invalid copper configs which can be frustrating, so if you see something like this ensure that all tasks have an input and output, and the connections between tasks are all defined.
You cannot have a task with an output that isn't connected to the input of some other task.
```rust
error: custom attribute panicked
  --> lunabot-cu/src/resim.rs:24:1
   |
24 | #[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
   | ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   |
   = help: message: index out of bounds: the len is 0 but the index is 0
```
