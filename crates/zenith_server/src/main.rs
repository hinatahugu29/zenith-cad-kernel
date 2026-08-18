use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, Mutex};

const DEFAULT_ADDR: &str = "127.0.0.1:8080";

#[derive(Debug, Default)]
struct ServerState {
    stacks: HashMap<i64, StackState>,
}

#[derive(Debug, Default)]
struct StackState {
    last_payload_len: usize,
}

#[derive(Debug, Deserialize)]
struct RequestHead {
    action: String,
    #[serde(default)]
    stack_ptr: i64,
}

fn main() -> std::io::Result<()> {
    let addr = std::env::var("ZENITH_SERVER_ADDR").unwrap_or_else(|_| DEFAULT_ADDR.to_string());
    let listener = TcpListener::bind(&addr)?;
    eprintln!("zenith_server listening on {addr}");

    let state = Arc::new(Mutex::new(ServerState::default()));
    let next_stack = Arc::new(AtomicI64::new(1));

    for incoming in listener.incoming() {
        let mut stream = match incoming {
            Ok(stream) => stream,
            Err(err) => {
                eprintln!("accept error: {err}");
                continue;
            }
        };

        let state = Arc::clone(&state);
        let next_stack = Arc::clone(&next_stack);
        if let Err(err) = handle_client(&mut stream, &state, &next_stack) {
            let _ = write_error(&mut stream, &err);
        }
    }

    Ok(())
}

fn handle_client(
    stream: &mut TcpStream,
    state: &Arc<Mutex<ServerState>>,
    next_stack: &Arc<AtomicI64>,
) -> Result<(), String> {
    let mut len_bytes = [0_u8; 4];
    stream
        .read_exact(&mut len_bytes)
        .map_err(|e| format!("failed to read request length: {e}"))?;
    let msg_len = u32::from_le_bytes(len_bytes) as usize;
    let mut msg = vec![0_u8; msg_len];
    stream
        .read_exact(&mut msg)
        .map_err(|e| format!("failed to read request payload: {e}"))?;

    let (request_json, binary_payload) = decode_request(&msg)?;
    let head: RequestHead = serde_json::from_value(request_json.clone())
        .map_err(|e| format!("invalid request header: {e}; json={request_json}"))?;

    match head.action.as_str() {
        "create_stack" => {
            let ptr = next_stack.fetch_add(1, Ordering::Relaxed);
            state
                .lock()
                .map_err(|_| "server state poisoned".to_string())?
                .stacks
                .insert(ptr, StackState::default());
            write_success(stream)?;
            stream
                .write_all(&ptr.to_le_bytes())
                .map_err(|e| format!("failed to write create_stack response: {e}"))?;
        }
        "delete_stack" => {
            state
                .lock()
                .map_err(|_| "server state poisoned".to_string())?
                .stacks
                .remove(&head.stack_ptr);
            write_success(stream)?;
        }
        "update" => {
            if let Some(payload) = binary_payload {
                let mut guard = state
                    .lock()
                    .map_err(|_| "server state poisoned".to_string())?;
                guard
                    .stacks
                    .entry(head.stack_ptr)
                    .or_default()
                    .last_payload_len = payload.len();
            }
            write_update_empty(stream)?;
        }
        "generate_mesh" => {
            write_generate_mesh_empty(stream)?;
        }
        "measure_stack" => {
            write_success(stream)?;
            let values = [0.0_f64; 11];
            for value in values {
                stream
                    .write_all(&value.to_le_bytes())
                    .map_err(|e| format!("failed to write measure_stack response: {e}"))?;
            }
        }
        "measure_entity" => {
            write_success(stream)?;
            let values = [0.0_f64; 10];
            for value in values {
                stream
                    .write_all(&value.to_le_bytes())
                    .map_err(|e| format!("failed to write measure_entity response: {e}"))?;
            }
        }
        "import_step" | "import_svg" => {
            write_json_response(stream, &json!([]))?;
        }
        "export_step"
        | "export_stack_to_step"
        | "export_parts_to_step"
        | "export_stack_to_stl"
        | "export_stack_to_iges"
        | "csg_preview_begin"
        | "csg_preview_end" => {
            write_success(stream)?;
        }
        other => {
            return Err(format!("unsupported action: {other}"));
        }
    }

    Ok(())
}

fn decode_request(msg: &[u8]) -> Result<(Value, Option<&[u8]>), String> {
    if msg.is_empty() {
        return Err("empty request payload".to_string());
    }

    if msg[0] == b'{' {
        let value: Value =
            serde_json::from_slice(msg).map_err(|e| format!("invalid json request: {e}"))?;
        return Ok((value, None));
    }

    if msg.len() < 4 {
        return Err("short framed update request".to_string());
    }

    let mut len_bytes = [0_u8; 4];
    len_bytes.copy_from_slice(&msg[..4]);
    let json_len = u32::from_le_bytes(len_bytes) as usize;
    let json_end = 4 + json_len;
    if msg.len() < json_end {
        return Err(format!(
            "short update json: need {json_end} bytes, got {}",
            msg.len()
        ));
    }

    let value: Value = serde_json::from_slice(&msg[4..json_end])
        .map_err(|e| format!("invalid framed json request: {e}"))?;
    Ok((value, Some(&msg[json_end..])))
}

fn write_success(stream: &mut TcpStream) -> Result<(), String> {
    stream
        .write_all(&[1])
        .map_err(|e| format!("failed to write success status: {e}"))
}

fn write_error(stream: &mut TcpStream, message: &str) -> Result<(), String> {
    stream
        .write_all(&[0])
        .map_err(|e| format!("failed to write error status: {e}"))?;
    let bytes = message.as_bytes();
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .map_err(|e| format!("failed to write error length: {e}"))?;
    stream
        .write_all(bytes)
        .map_err(|e| format!("failed to write error body: {e}"))
}

fn write_json_response(stream: &mut TcpStream, value: &Value) -> Result<(), String> {
    write_success(stream)?;
    let bytes = serde_json::to_vec(value).map_err(|e| format!("failed to encode json: {e}"))?;
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .map_err(|e| format!("failed to write json length: {e}"))?;
    stream
        .write_all(&bytes)
        .map_err(|e| format!("failed to write json body: {e}"))
}

fn write_update_empty(stream: &mut TcpStream) -> Result<(), String> {
    write_success(stream)?;

    let empty_f32: [f32; 0] = [];
    let empty_i32: [i32; 0] = [];
    let meta = json!({
        "use_mmap": false,
        "edge_lineages": [],
        "mesh_face_ids": [],
        "perf_bool": 0.0,
        "perf_edge": 0.0,
        "perf_mesh": 0.0,
        "perf_prim": 0.0
    });
    let meta_bytes =
        serde_json::to_vec(&meta).map_err(|e| format!("failed to encode meta: {e}"))?;

    let lengths = [0_u32, 0_u32, 0_u32, 0_u32, 0_u32];
    for length in lengths {
        stream
            .write_all(&length.to_le_bytes())
            .map_err(|e| format!("failed to write update lengths: {e}"))?;
    }
    stream
        .write_all(&(meta_bytes.len() as u32).to_le_bytes())
        .map_err(|e| format!("failed to write update meta length: {e}"))?;
    stream
        .write_all(&meta_bytes)
        .map_err(|e| format!("failed to write update meta: {e}"))?;

    write_f32_slice(stream, &empty_f32)?;
    write_i32_slice(stream, &empty_i32)?;
    write_f32_slice(stream, &empty_f32)?;
    write_i32_slice(stream, &empty_i32)?;
    write_i32_slice(stream, &empty_i32)
}

fn write_generate_mesh_empty(stream: &mut TcpStream) -> Result<(), String> {
    write_success(stream)?;
    let meta = json!({ "face_ids": [] });
    let meta_bytes =
        serde_json::to_vec(&meta).map_err(|e| format!("failed to encode meta: {e}"))?;

    for length in [0_u32, 0_u32, 0_u32] {
        stream
            .write_all(&length.to_le_bytes())
            .map_err(|e| format!("failed to write mesh lengths: {e}"))?;
    }
    stream
        .write_all(&(meta_bytes.len() as u32).to_le_bytes())
        .map_err(|e| format!("failed to write mesh meta length: {e}"))?;
    stream
        .write_all(&meta_bytes)
        .map_err(|e| format!("failed to write mesh meta: {e}"))
}

fn write_f32_slice(stream: &mut TcpStream, values: &[f32]) -> Result<(), String> {
    for value in values {
        stream
            .write_all(&value.to_le_bytes())
            .map_err(|e| format!("failed to write f32 slice: {e}"))?;
    }
    Ok(())
}

fn write_i32_slice(stream: &mut TcpStream, values: &[i32]) -> Result<(), String> {
    for value in values {
        stream
            .write_all(&value.to_le_bytes())
            .map_err(|e| format!("failed to write i32 slice: {e}"))?;
    }
    Ok(())
}
