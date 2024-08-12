use serde::{Deserialize, Serialize};

// 命令类型
#[derive(Debug, Serialize, Deserialize)]
#[serde(tag = "command_type")]
pub enum CommandType {
    DifferSpeed { x: f64, y: f64, w: f64 }, // 差速命令
}
