//! Shared test support.
//!
//! Each integration-test target compiles this module separately and uses a
//! different subset of it, so dead-code analysis fires on whatever that target
//! doesn't touch. Allowed here once rather than annotated item by item.

#[allow(dead_code)]
pub mod fake_agent;
#[allow(dead_code)]
pub mod platform_double;
