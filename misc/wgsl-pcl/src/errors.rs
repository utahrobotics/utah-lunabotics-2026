#[derive(thiserror::Error, Debug)]
pub enum WgslPclError {
    #[error("Shader compilation error: {0}")]
    CompilationError(String),
}
