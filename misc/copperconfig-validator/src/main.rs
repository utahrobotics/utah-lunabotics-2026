use clap::Parser;
use colored::Colorize;
use copperconfig_validator::ConfigValidator;
use std::process;

fn main() {
    let validator = ConfigValidator::parse();

    if let Err(e) = validator.validate() {
        eprint!("{}", "\nValidation failed: ".red());
        eprint!("{} \n", e);
        process::exit(1);
    }
}
