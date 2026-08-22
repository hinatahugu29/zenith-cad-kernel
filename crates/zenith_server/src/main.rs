use serde::Deserialize;
use serde_json::Value;
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
        // ここから下は、プロトコルの枠だけがあって中身が無い。
        //
        // **もっともらしい成功を返すのをやめた。** 以前はこうだった:
        //
        // - `generate_mesh` は三角形0枚のメッシュを返す。クライアントからは
        //   「モデルが空」と区別が付かない。
        // - `measure_stack` / `measure_entity` は 0.0 を 11 個・10 個返す。
        //   体積 0、面積 0、重心 (0,0,0) が表示される。**空ではなく誤答**。
        // - `export_*` は**ファイルを1バイトも書かずに**成功を返す。
        //   保存したつもりの人が、あとで何も無いことに気づく。
        //
        // 実装していないことは、実装していないと言う。名前を挙げて断れば、
        // クライアントは失敗として扱えるし、次に手を付ける人は何が空白かを
        // 一覧できる。
        other => {
            if let ("update", Some(payload)) = (other, binary_payload) {
                // 受け取ったことだけは控えておく。中身の解釈はまだ無い。
                let mut guard = state
                    .lock()
                    .map_err(|_| "server state poisoned".to_string())?;
                guard
                    .stacks
                    .entry(head.stack_ptr)
                    .or_default()
                    .last_payload_len = payload.len();
            }
            return Err(unimplemented_message(other));
        }
    }

    Ok(())
}

/// まだ書かれていない動作に対する返事。
///
/// `SEAMLESS_PROTOCOL.md` に載っている動作のうち、`create_stack` と
/// `delete_stack` 以外はまだ中身がない。この crate はプロトコルの枠であって、
/// カーネルには繋がっていない（`zenith_algo` を依存に持ってはいるが、呼んで
/// いない）。Blender との連携は `zenith_py` のインプロセス経路のほうが先に
/// 動いているので、こちらは保留のままである。
fn unimplemented_message(action: &str) -> String {
    const KNOWN: &[&str] = &[
        "update",
        "generate_mesh",
        "measure_stack",
        "measure_entity",
        "import_step",
        "import_svg",
        "export_step",
        "export_stack_to_step",
        "export_parts_to_step",
        "export_stack_to_stl",
        "export_stack_to_iges",
        "csg_preview_begin",
        "csg_preview_end",
        "face_ids",
        "mesh_face_ids",
        "edge_lineages",
        "perf_bool",
        "perf_edge",
        "perf_mesh",
    ];
    if KNOWN.contains(&action) {
        format!(
            "zenith_server does not implement '{action}' yet. Only create_stack and \
             delete_stack are wired up; this crate is the protocol shell and does not \
             call the kernel. Use the in-process binding (zenith_py) instead."
        )
    } else {
        format!("unsupported action: {action}")
    }
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

// ここには、空の応答を組み立てる関数が5本ありました
// （`write_json_response`、`write_update_empty`、`write_generate_mesh_empty`、
// `write_f32_slice`、`write_i32_slice`）。三角形0枚のメッシュ、長さ0の配列、
// 空のメタ情報を、**成功として**返すためのものです。
//
// 実装していない動作を名前で断るようにしたので、組み立てる相手がいなく
// なりました。残しておくと「いつでも空を返せる」という誘惑になるので、
// 消してあります。
