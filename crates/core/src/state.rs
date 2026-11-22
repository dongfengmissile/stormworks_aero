//! Process-wide runtime state shared by the bootstrap, hooks and overlay.

use std::path::PathBuf;
use std::sync::atomic::AtomicUsize;
use std::sync::Mutex;

pub static MODULE_BASE: AtomicUsize = AtomicUsize::new(0);
pub static SELF_MODULE: AtomicUsize = AtomicUsize::new(0);
pub static LOG_PATH: Mutex<Option<PathBuf>> = Mutex::new(None);
