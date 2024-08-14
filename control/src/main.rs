mod command;
mod config;
mod enums;
mod error;
mod json_data;

use error::*;

use dora_node_api::arrow::array::PrimitiveArray;
use dora_node_api::arrow::datatypes::UInt8Type;
use dora_node_api::{self, DoraNode, Event};

use core::str;
use dora_node_api::arrow::buffer::ScalarBuffer;
use json_data::ImuData;
use serial::SerialPort;
use std::io::Write;
use tokio::sync::mpsc;
use std::time::Duration;

// serial port
static SERIAL_PORT: &str = "/dev/ttyUSB1";

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // 连接串口
    const COM_SETTINGS: serial::PortSettings = serial::PortSettings {
        baud_rate: serial::Baud115200,
        char_size: serial::Bits8,
        parity: serial::ParityNone,
        stop_bits: serial::Stop1,
        flow_control: serial::FlowNone,
    };

    let mut com = serial::open(SERIAL_PORT).map_err(|_| Error::Connect)?;
    com.configure(&COM_SETTINGS)
        .map_err(|_| Error::SettingsSet)?;
    com.set_timeout(Duration::from_millis(1000))
        .map_err(|_| Error::SetTimeout)?;

    // 消息通道
    let (tx, mut rx) = mpsc::channel::<Vec<u8>>(100);

    tokio::spawn(async move {
        while let Some(data) = rx.recv().await {
            com.write_all(&data).ok();
        }
    });

    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id: _,
                data,
                metadata: _,
            } => {
                // check if new size bracket
                let array = data
                    .as_any()
                    .downcast_ref::<PrimitiveArray<UInt8Type>>()
                    .expect("Expected a PrimitiveArray of UInt8Type");

                let buffer: &ScalarBuffer<u8> = array.values();

                let bytes: &[u8] =
                    unsafe { std::slice::from_raw_parts(buffer.as_ptr(), buffer.len()) };

                match str::from_utf8(bytes) {
                    Ok(s) => {
                        if let Ok(imu) = serde_json::from_str::<ImuData>(s) {
                            let data =
                                command::send_speed_to_x4chassis(imu.linear.x, 0.0, imu.angular.z);
                            tx.send(data).await .ok();
                        }
                    }
                    Err(e) => println!("failed: {}", e),
                }
            }
            Event::InputClosed { id } => {
                println!("Input `{id}` was closed");
            }
            other => eprintln!("Receive d unexpected input: {other:?}"),
        }
    }

    println!("finished");
    Ok(())
}
