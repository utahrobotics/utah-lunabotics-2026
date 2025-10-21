//! This tool does not guarantee that a config will actually compile, it just checks for cycles and nodes with unused inputs or outputs.
//! Takes from core/cu29_runtime/src/config.rs to parse a copper config and give useful error messages on if the copperconfig is a valid directed acyclic graph.
//! The only place copper task graphs differ from normal DAGs is that a task can have multiple input types, but only one output type.
//!     There can be multiple output edges, i.e. multiple sink tasks can subscribe to the same source task, but that source task cannot have multiple output types.
//! All tasks must have their inputs and outputs used, dangling edges are an error.
//! This tool does not check that the types passed tasks are valid, as the error messages reported by the copper runtime for that are useful enough.
//!
//! Invalid DAG error message from the copper runtime:
//! error: custom attribute panicked
//!   --> lunabot-cu/src/resim.rs:24:1
//!    |
//! 24 | #[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
//!    | ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//!    |
//!    = help: message: index out of bounds: the len is 0 but the index is 0
//!
//!
//! Invalid digraph error message using this validator:
//! Validation failed: Node 'l2_imu' is isolated (check that its inputs are supplied and outputs are utilized)
//!
//! Another example output:
//! Validation failed: Cycle detected in graph: localizer -> new_ai -> v3_pico -> localizer
//!
//! Todo:
//! I have not tested this tool on copper configs with missions

use clap::Parser;
use colored::Colorize;
use petgraph::algo::is_cyclic_directed;
use petgraph::stable_graph::{NodeIndex, StableDiGraph};
use petgraph::visit::{EdgeRef, IntoEdgeReferences};
use ron::Options;
use ron::extensions::Extensions;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

#[derive(Parser)]
#[command(
    long_about = "Validates that a copperconfig is a valid directed acyclic graph. Does not check types."
)]
pub struct ConfigValidator {
    /// Path to copperconfig.ron
    #[arg(short, long)]
    pub config_path: String,

    /// Generate SVG visualization of the task graph and save to this path (outputs as .svg)
    #[arg(short, long)]
    pub output_svg: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Value(ron::value::Value);

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ComponentConfig(pub HashMap<String, Value>);

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct NodeLogging {
    enabled: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Node {
    id: String,

    #[serde(rename = "type", skip_serializing_if = "Option::is_none")]
    type_: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,

    #[serde(skip_serializing_if = "Option::is_none")]
    missions: Option<Vec<String>>,

    #[serde(skip_serializing_if = "Option::is_none")]
    background: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    run_in_sim: Option<bool>,

    #[serde(skip_serializing_if = "Option::is_none")]
    logging: Option<NodeLogging>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Cnx {
    src: String,

    dst: String,

    pub msg: String,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct MonitorConfig {
    #[serde(rename = "type")]
    type_: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    config: Option<ComponentConfig>,
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct LoggingConfig {
    #[serde(default, skip_serializing_if = "Clone::clone")]
    pub enable_task_logging: bool,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub slab_size_mib: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub section_size_mib: Option<u64>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub keyframe_interval: Option<u32>,
}

#[derive(Serialize, Deserialize, Default, Debug, Clone)]
pub struct RuntimeConfig {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate_target_hz: Option<u64>,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MissionsConfig {
    pub id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct IncludesConfig {
    pub path: String,
    pub params: HashMap<String, Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub missions: Option<Vec<String>>,
}

#[derive(Serialize, Deserialize, Default, Debug)]
struct CuConfigRepresentation {
    tasks: Option<Vec<Node>>,
    cnx: Option<Vec<Cnx>>,
    monitor: Option<MonitorConfig>,
    logging: Option<LoggingConfig>,
    runtime: Option<RuntimeConfig>,
    missions: Option<Vec<MissionsConfig>>,
    includes: Option<Vec<IncludesConfig>>,
}

pub type ValidationGraph = StableDiGraph<Node, Cnx, u32>;

pub type ValidationResult<T> = Result<T, ValidationError>;

#[derive(Debug)]
pub enum ValidationError {
    IoError(String),
    ParseError(String),
    CycleDetected(Vec<String>),
    DanglingSource(String),
    DanglingSink(String),
    IsolatedNode(String),
    SourceNotFound(String),
    DestinationNotFound(String),
    InvalidLoggingConfig(String),
    MultipleOutputTypes(String, Vec<String>),
}

impl std::fmt::Display for ValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ValidationError::IoError(msg) => write!(f, "{}", msg),
            ValidationError::ParseError(msg) => write!(f, "{}", msg),
            ValidationError::CycleDetected(cycle) => {
                write!(f, "Cycle detected in graph: {}", cycle.join(" -> "))
            }
            ValidationError::DanglingSource(node) => {
                write!(f, "Node '{}' has no outgoing connections", node.red())
            }
            ValidationError::DanglingSink(node) => {
                write!(f, "Node '{}' has no incoming connections", node.red())
            }
            ValidationError::IsolatedNode(node) => {
                write!(
                    f,
                    "Node '{}' is isolated (check that its inputs are supplied and outputs are utilized)",
                    node.red()
                )
            }
            ValidationError::SourceNotFound(src) => {
                write!(f, "Source node '{}' not found", src)
            }
            ValidationError::DestinationNotFound(dst) => {
                write!(f, "Destination node '{}' not found", dst.red())
            }
            ValidationError::InvalidLoggingConfig(msg) => write!(f, "{}", msg.red()),
            ValidationError::MultipleOutputTypes(node, types) => {
                write!(
                    f,
                    "Node '{}' has multiple output types: [{}]. Tasks can only have one output type.",
                    node.red(),
                    types.join(", ")
                )
            }
        }
    }
}

impl std::error::Error for ValidationError {}

impl ConfigValidator {
    pub fn validate(&self) -> ValidationResult<()> {
        let config = self.read_config()?;

        if let Some(missions) = &config.missions {
            for mission in missions {
                println!("  • {}", mission.id.bright_yellow());
            }
            println!();
            self.validate_missions(&config, missions)?;
        } else {
            self.validate_simple(&config)?;
        }

        self.validate_logging(&config)?;

        println!("{}", "[*] Configuration is (likely) valid!".green().bold());

        if let Some(output_path) = &self.output_svg {
            if let Some(missions) = &config.missions {
                for mission in missions {
                    let mission_output = output_path.to_string() + &format!("_{}.svg", mission.id);
                    let graph = self.build_graph(&config, Some(&mission.id))?;
                    self.render_svg(&graph, &mission_output, Some(&mission.id))?;
                    println!("{}", format!("[*] Graph saved to {mission_output}").green())
                }
            } else {
                let graph = self.build_graph(&config, None)?;
                self.render_svg(&graph, output_path, None)?;
            }
            println!("{}", format!("[*] Graph saved to {output_path}").green())
        }

        Ok(())
    }

    fn read_config(&self) -> ValidationResult<CuConfigRepresentation> {
        let path = Path::new(&self.config_path);
        let content = fs::read_to_string(path).map_err(|e| {
            ValidationError::IoError(format!("Failed to read file '{}': {}", self.config_path, e))
        })?;

        let options = Options::default()
            .with_default_extension(Extensions::IMPLICIT_SOME)
            .with_default_extension(Extensions::UNWRAP_NEWTYPES)
            .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES);

        let representation: CuConfigRepresentation = options.from_str(&content).map_err(|e| {
            ValidationError::ParseError(format!(
                "Failed to parse configuration: Error: {} at position {}",
                e.code, e.span
            ))
        })?;

        self.process_includes(&self.config_path, representation, &mut Vec::new())
    }

    fn process_includes(
        &self,
        file_path: &str,
        base_representation: CuConfigRepresentation,
        processed_files: &mut Vec<String>,
    ) -> ValidationResult<CuConfigRepresentation> {
        processed_files.push(file_path.to_string());

        let mut result = base_representation;

        if let Some(includes) = result.includes.take() {
            for include in includes {
                let include_path = if include.path.starts_with('/') {
                    include.path.clone()
                } else {
                    let current_dir = Path::new(file_path)
                        .parent()
                        .unwrap_or_else(|| Path::new(""))
                        .to_string_lossy()
                        .to_string();

                    format!("{}/{}", current_dir, include.path)
                };

                let include_content = fs::read_to_string(&include_path).map_err(|e| {
                    ValidationError::IoError(format!(
                        "Failed to read include file '{}': {}",
                        include_path, e
                    ))
                })?;

                let processed_content =
                    self.substitute_parameters(&include_content, &include.params);

                let options = Options::default()
                    .with_default_extension(Extensions::IMPLICIT_SOME)
                    .with_default_extension(Extensions::UNWRAP_NEWTYPES)
                    .with_default_extension(Extensions::UNWRAP_VARIANT_NEWTYPES);

                let mut included_representation: CuConfigRepresentation =
                    options.from_str(&processed_content).map_err(|e| {
                        ValidationError::ParseError(format!(
                            "Failed to parse include file '{}': Error: {} at position {}",
                            include_path, e.code, e.span
                        ))
                    })?;

                included_representation =
                    self.process_includes(&include_path, included_representation, processed_files)?;

                if let Some(included_tasks) = included_representation.tasks {
                    if result.tasks.is_none() {
                        result.tasks = Some(included_tasks);
                    } else {
                        let mut tasks = result.tasks.take().unwrap();
                        for included_task in included_tasks {
                            if !tasks.iter().any(|t| t.id == included_task.id) {
                                tasks.push(included_task);
                            }
                        }
                        result.tasks = Some(tasks);
                    }
                }

                if let Some(included_cnx) = included_representation.cnx {
                    if result.cnx.is_none() {
                        result.cnx = Some(included_cnx);
                    } else {
                        let mut cnx = result.cnx.take().unwrap();
                        for included_c in included_cnx {
                            if !cnx
                                .iter()
                                .any(|c| c.src == included_c.src && c.dst == included_c.dst)
                            {
                                cnx.push(included_c);
                            }
                        }
                        result.cnx = Some(cnx);
                    }
                }

                if result.monitor.is_none() {
                    result.monitor = included_representation.monitor;
                }
                if result.logging.is_none() {
                    result.logging = included_representation.logging;
                }
                if result.runtime.is_none() {
                    result.runtime = included_representation.runtime;
                }

                if let Some(included_missions) = included_representation.missions {
                    if result.missions.is_none() {
                        result.missions = Some(included_missions);
                    } else {
                        let mut missions = result.missions.take().unwrap();
                        for included_mission in included_missions {
                            if !missions.iter().any(|m| m.id == included_mission.id) {
                                missions.push(included_mission);
                            }
                        }
                        result.missions = Some(missions);
                    }
                }
            }
        }

        Ok(result)
    }

    fn substitute_parameters(&self, content: &str, params: &HashMap<String, Value>) -> String {
        let mut result = content.to_string();

        for (key, value) in params {
            let pattern = format!("{{{{{key}}}}}");
            let value_str = match &value.0 {
                ron::value::Value::String(s) => s.clone(),
                ron::value::Value::Number(n) => format!("{:?}", n),
                ron::value::Value::Bool(b) => b.to_string(),
                _ => format!("{:?}", value.0),
            };
            result = result.replace(&pattern, &value_str);
        }

        result
    }

    fn validate_simple(&self, config: &CuConfigRepresentation) -> ValidationResult<()> {
        let graph = self.build_graph(config, None)?;
        self.validate_graph(&graph, None)?;
        Ok(())
    }

    fn validate_missions(
        &self,
        config: &CuConfigRepresentation,
        missions: &[MissionsConfig],
    ) -> ValidationResult<()> {
        for mission in missions {
            let graph = self.build_graph(config, Some(&mission.id))?;
            self.validate_graph(&graph, Some(&mission.id))?;
        }

        Ok(())
    }

    fn build_graph(
        &self,
        config: &CuConfigRepresentation,
        mission_id: Option<&str>,
    ) -> ValidationResult<ValidationGraph> {
        let mut graph = StableDiGraph::new();
        let mut node_map: HashMap<String, NodeIndex> = HashMap::new();

        if let Some(tasks) = &config.tasks {
            for task in tasks {
                if let Some(mid) = mission_id {
                    if let Some(task_missions) = &task.missions {
                        if !task_missions.contains(&mid.to_string()) {
                            continue;
                        }
                    }
                }

                let idx = graph.add_node(task.clone());
                node_map.insert(task.id.clone(), idx);
            }
        }

        if let Some(connections) = &config.cnx {
            for cnx in connections {
                if let Some(mid) = mission_id {
                    if let Some(cnx_missions) = &cnx.missions {
                        if !cnx_missions.contains(&mid.to_string()) {
                            continue;
                        }
                    }
                }

                let src_idx = node_map.get(&cnx.src).ok_or_else(|| {
                    ValidationError::SourceNotFound(format!(
                        "Source '{}' in connection not found",
                        cnx.src
                    ))
                })?;

                let dst_idx = node_map.get(&cnx.dst).ok_or_else(|| {
                    ValidationError::DestinationNotFound(format!(
                        "Destination '{}' in connection not found",
                        cnx.dst
                    ))
                })?;

                graph.add_edge(*src_idx, *dst_idx, cnx.clone());
            }
        }

        Ok(graph)
    }

    fn validate_graph(
        &self,
        graph: &ValidationGraph,
        mission_id: Option<&str>,
    ) -> ValidationResult<()> {
        let mission_str = mission_id
            .map(|m| format!(" (mission: {})", m))
            .unwrap_or_default();

        if is_cyclic_directed(graph) {
            let cycle = self.find_cycle(graph);
            return Err(ValidationError::CycleDetected(cycle));
        }

        // tasks cant have multiple output types
        self.validate_single_output_type(graph)?;

        let (sources, sinks, middle_nodes) = self.categorize_nodes(graph);

        println!("[*] {} source(s)", sources.len());
        println!("[*] {} sink(s)", sinks.len());
        println!("[*] {} middle node(s)", middle_nodes.len());

        for node_idx in graph.node_indices() {
            let incoming = graph
                .edges_directed(node_idx, petgraph::Direction::Incoming)
                .count();
            let outgoing = graph
                .edges_directed(node_idx, petgraph::Direction::Outgoing)
                .count();

            if incoming == 0 && outgoing == 0 {
                let node = &graph[node_idx];
                return Err(ValidationError::IsolatedNode(node.id.to_owned()));
            }
        }

        Ok(())
    }

    fn categorize_nodes(
        &self,
        graph: &ValidationGraph,
    ) -> (Vec<NodeIndex>, Vec<NodeIndex>, Vec<NodeIndex>) {
        let mut sources = Vec::new();
        let mut sinks = Vec::new();
        let mut middle_nodes = Vec::new();

        for node_idx in graph.node_indices() {
            let incoming = graph
                .edges_directed(node_idx, petgraph::Direction::Incoming)
                .count();
            let outgoing = graph
                .edges_directed(node_idx, petgraph::Direction::Outgoing)
                .count();

            match (incoming, outgoing) {
                (0, 0) => {} // isolated
                (0, _) => sources.push(node_idx),
                (_, 0) => sinks.push(node_idx),
                _ => middle_nodes.push(node_idx),
            }
        }

        (sources, sinks, middle_nodes)
    }

    fn validate_single_output_type(&self, graph: &ValidationGraph) -> ValidationResult<()> {
        for node_idx in graph.node_indices() {
            let mut output_types = HashSet::new();

            for edge in graph.edges_directed(node_idx, petgraph::Direction::Outgoing) {
                output_types.insert(edge.weight().msg.clone());
            }

            if output_types.len() > 1 {
                let node = &graph[node_idx];
                let types: Vec<String> = output_types.into_iter().collect();
                return Err(ValidationError::MultipleOutputTypes(node.id.clone(), types));
            }
        }

        Ok(())
    }

    fn find_cycle(&self, graph: &ValidationGraph) -> Vec<String> {
        use petgraph::algo::toposort;

        if let Err(cycle_node) = toposort(graph, None) {
            let mut visited = HashSet::new();
            let mut path = Vec::new();
            let mut cycle = Vec::new();

            self.dfs_find_cycle(
                graph,
                cycle_node.node_id(),
                &mut visited,
                &mut path,
                &mut cycle,
            );

            return cycle;
        }

        unreachable!(
            "is_cyclic_directed returned true but no cycle was found. (This is a bug please open an issue)"
        );
    }

    fn dfs_find_cycle(
        &self,
        graph: &ValidationGraph,
        node: NodeIndex,
        visited: &mut HashSet<NodeIndex>,
        path: &mut Vec<NodeIndex>,
        cycle: &mut Vec<String>,
    ) -> bool {
        if path.contains(&node) {
            let cycle_start = path.iter().position(|&n| n == node).unwrap();
            *cycle = path[cycle_start..]
                .iter()
                .map(|&idx| graph[idx].id.clone())
                .collect();
            cycle.push(graph[node].id.clone());
            return true;
        }

        if visited.contains(&node) {
            return false;
        }

        visited.insert(node);
        path.push(node);

        for edge in graph.edges(node) {
            if self.dfs_find_cycle(graph, edge.target(), visited, path, cycle) {
                return true;
            }
        }

        path.pop();
        false
    }

    fn validate_logging(&self, config: &CuConfigRepresentation) -> ValidationResult<()> {
        if let Some(logging) = &config.logging {
            if let (Some(section_size), Some(slab_size)) =
                (logging.section_size_mib, logging.slab_size_mib)
            {
                if section_size > slab_size {
                    return Err(ValidationError::InvalidLoggingConfig(format!(
                        "Section size ({} MiB) cannot be larger than slab size ({} MiB)",
                        section_size, slab_size
                    )));
                }
            }
        }
        Ok(())
    }

    fn render_svg(
        &self,
        graph: &ValidationGraph,
        output_path: &str,
        _mission_id: Option<&str>,
    ) -> ValidationResult<()> {
        use layout::backends::svg::SVGWriter;
        use layout::gv::GraphBuilder;
        use layout::gv::parser::DotParser;

        if graph.node_count() == 0 {
            return Err(ValidationError::IoError(
                "Cannot render empty graph".to_string(),
            ));
        }

        let dot_content = self.graph_to_dot(graph);

        let dot_path = output_path.to_string() + ".dot";
        fs::write(&dot_path, &dot_content)
            .map_err(|e| ValidationError::IoError(format!("Failed to write DOT file: {}", e)))?;

        let mut parser = DotParser::new(&dot_content);
        let tree = parser
            .process()
            .map_err(|e| ValidationError::IoError(format!("Failed to parse DOT format: {}", e)))?;

        let mut graph_builder = GraphBuilder::new();
        graph_builder.visit_graph(&tree);
        let mut visual_graph = graph_builder.get();

        let mut svg_writer = SVGWriter::new();
        visual_graph.do_it(false, false, false, &mut svg_writer);
        let svg_content = svg_writer.finalize();
        fs::write(output_path, svg_content)
            .map_err(|e| ValidationError::IoError(format!("Failed to write SVG file: {}", e)))?;

        Ok(())
    }

    // adapted from the render() function in copper, doesnt use html to stay compatible with the layout-rs
    // crate svg renderer
    fn graph_to_dot(&self, graph: &ValidationGraph) -> String {
        let mut dot = String::from("digraph G {\n");

        fn wrap_text(text: &str, max_width: usize) -> String {
            let mut result = Vec::new();
            let mut current_line = String::new();

            for word in text.split_whitespace() {
                if current_line.len() + word.len() + 1 > max_width && !current_line.is_empty() {
                    result.push(current_line.clone());
                    current_line.clear();
                }

                if !current_line.is_empty() {
                    current_line.push(' ');
                }
                current_line.push_str(word);
            }

            if !current_line.is_empty() {
                result.push(current_line);
            }

            result.join("\\n")
        }

        for node_idx in graph.node_indices() {
            let node = &graph[node_idx];

            let config_str = match &node.config {
                Some(config) => {
                    let config_entries = config
                        .0
                        .iter()
                        .map(|(k, v)| {
                            let value = ron::to_string(v)
                                .expect("couldn't convert RON Value to string")
                                .replace("\"", "\\\"");
                            let full_entry = format!("  {} = {}", k, &value[1..value.len() - 1]);

                            if full_entry.len() > 100 {
                                wrap_text(&full_entry, 100)
                            } else {
                                full_entry
                            }
                        })
                        .collect::<Vec<String>>()
                        .join("\\n");
                    if config_entries.is_empty() {
                        String::new()
                    } else {
                        format!("\\n{}\\n{}", "--------------------", config_entries)
                    }
                }
                None => String::new(),
            };

            dot.push_str(&format!("{} [\n", node_idx.index()));
            dot.push_str("shape=box,\n");
            dot.push_str("style=\"rounded,filled\",\n");
            dot.push_str("fontname=\"Noto Sans\",\n");

            let incoming = graph
                .edges_directed(node_idx, petgraph::Direction::Incoming)
                .count();
            let outgoing = graph
                .edges_directed(node_idx, petgraph::Direction::Outgoing)
                .count();

            let is_src = incoming == 0 && outgoing > 0;
            let is_sink = outgoing == 0 && incoming > 0;

            if is_src {
                dot.push_str("fillcolor=lightgreen,\n");
            } else if is_sink {
                dot.push_str("fillcolor=lightblue,\n");
            } else {
                dot.push_str("fillcolor=lightgrey,\n");
            }
            dot.push_str("color=grey,\n");

            let type_str = node.type_.as_ref().map(|s| s.as_str()).unwrap_or("?");

            let escaped_id = node.id.replace('\\', "\\\\").replace('"', "\\\"");
            let escaped_type = type_str.replace('\\', "\\\\").replace('"', "\\\"");

            dot.push_str(&format!(
                "label=\"{}\\n[{}]{}\"\n",
                escaped_id, escaped_type, config_str
            ));
            dot.push_str("];\n");
        }

        for edge in graph.edge_references() {
            let src_idx = edge.source().index();
            let dst_idx = edge.target().index();
            let edge_data = edge.weight();

            let escaped_msg = edge_data.msg.replace('\\', "\\\\").replace('"', "\\\"");

            // has to be plain text
            dot.push_str(&format!(
                "{} -> {} [label=\"{}\", fontsize=10, labeldistance=1.5, labelangle=0];\n",
                src_idx, dst_idx, escaped_msg
            ));
        }

        dot.push_str("}\n");
        dot
    }
}
