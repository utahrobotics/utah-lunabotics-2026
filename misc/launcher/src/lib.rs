use std::process::{Command, Stdio};
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug, Clone)]
pub struct ProcessConfig {
    pub working_directory: Option<String>,
    pub suppress_stdout: bool,
    pub suppress_stderr: bool,
    pub detach: bool, // Run process in background without waiting
}

impl Default for ProcessConfig {
    fn default() -> Self {
        Self {
            working_directory: None,
            suppress_stdout: false,
            suppress_stderr: false,
            detach: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ProcessCommand {
    pub command: String,
    pub args: Vec<&'static str>,
    pub config: ProcessConfig,
}

impl ProcessCommand {
    pub fn new(command: impl Into<String>) -> Self {
        Self {
            command: command.into(),
            args: Vec::new(),
            config: ProcessConfig::default(),
        }
    }

    pub fn with_args(mut self, args: Vec<&'static str>) -> Self {
        self.args = args;
        self
    }

    pub fn with_working_directory(mut self, dir: impl Into<String>) -> Self {
        self.config.working_directory = Some(dir.into());
        self
    }

    pub fn with_suppress_stdout(mut self, suppress: bool) -> Self {
        self.config.suppress_stdout = suppress;
        self
    }

    pub fn with_suppress_stderr(mut self, suppress: bool) -> Self {
        self.config.suppress_stderr = suppress;
        self
    }

    pub fn with_suppress_output(mut self, suppress: bool) -> Self {
        self.config.suppress_stdout = suppress;
        self.config.suppress_stderr = suppress;
        self
    }

    pub fn with_detach(mut self, detach: bool) -> Self {
        self.config.detach = detach;
        self
    }
}

pub struct ProcessLauncher {
    commands: HashMap<String, ProcessCommand>,
}

impl ProcessLauncher {
    pub fn new() -> Self {
        Self {
            commands: HashMap::new(),
        }
    }

    pub fn add_command(&mut self, name: impl Into<String>, command: ProcessCommand) -> &mut Self {
        self.commands.insert(name.into(), command);
        self
    }

    pub fn launch_all(&self) -> Result<(), Box<dyn std::error::Error>> {
        for (name, command) in &self.commands {
            self.launch_process(command)?;
        }
        Ok(())
    }

    pub fn launch_command(&self, name: &str) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(command) = self.commands.get(name) {
            self.launch_process(command)?;
        } else {
            return Err(format!("Command '{}' not found", name).into());
        }
        Ok(())
    }

    fn launch_process(&self, process_command: &ProcessCommand) -> Result<(), Box<dyn std::error::Error>> {
        let mut cmd = Command::new(&process_command.command);
        
        // Add arguments
        if !process_command.args.is_empty() {
            cmd.args(&process_command.args);
        }

        // Set working directory
        if let Some(working_dir) = &process_command.config.working_directory {
            cmd.current_dir(working_dir);
        }

        // Configure stdout
        if process_command.config.suppress_stdout {
            cmd.stdout(Stdio::null());
        }

        // Configure stderr
        if process_command.config.suppress_stderr {
            cmd.stderr(Stdio::null());
        }

        // Launch the process
        if process_command.config.detach {
            // Spawn and detach (don't wait for completion)
            cmd.spawn()?;
        } else {
            // Wait for completion
            let status = cmd.status()?;
            if !status.success() {
                return Err(format!("Command failed with exit code: {:?}", status.code()).into());
            }
        }

        Ok(())
    }

    pub fn list_commands(&self) -> Vec<&String> {
        self.commands.keys().collect()
    }

    pub fn remove_command(&mut self, name: &str) -> Option<ProcessCommand> {
        self.commands.remove(name)
    }

    pub fn clear_commands(&mut self) {
        self.commands.clear();
    }
}