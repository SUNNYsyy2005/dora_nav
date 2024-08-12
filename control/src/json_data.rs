use serde::{Deserialize, Serialize};

use crate::enums::CommandType;

#[derive(Debug, Serialize, Deserialize)]
pub struct JsonData {
    pub sleep_second: u64,
    pub command: CommandType,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ImuData {
    pub angular: Angular,
    pub linear: Linear,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Angular {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct Linear {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
