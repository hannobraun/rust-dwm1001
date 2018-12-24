#!/usr/bin/env bash

# Build all crate items
cargo build --verbose --examples --all-features

# Build all template items
cargo build --manifest-path templates/Cargo.toml --all
