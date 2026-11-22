//! The game's own UI sounds (knowledge/audio.md "UI sounds", `22e792753489`).

use std::sync::atomic::{AtomicBool, Ordering};

use crate::editor::ptr_ok;
use crate::logging::log_line;
use crate::memory::{bytes_match, read_u32_at, read_usize_at, PTR_MIN};

/// `sfx_bank_play(bank, &handle, index, volume, loop, pitch)` and `sound_handle_release(&handle)`.
const RVA_SFX_BANK_PLAY: usize = 0x0028_9870;
const SFX_BANK_PLAY_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xec, 0x50, 0x4c, 0x8b, 0xfa, 0x48, 0x8b,
    0xd9, 0x33, 0xed, 0x89, 0x6c, 0x24,
];
const RVA_SOUND_HANDLE_RELEASE: usize = 0x001a_d270;
const SOUND_HANDLE_RELEASE_PROLOGUE: [u8; 32] = [
    0x48, 0x89, 0x5c, 0x24, 0x10, 0x48, 0x89, 0x4c, 0x24, 0x08, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48, 0x8b, 0xd9, 0x48, 0x8b, 0x01, 0x48, 0x85, 0xc0, 0x74, 0x32,
    0xff, 0x48, 0x08, 0x48, 0x8b, 0x39,
];
/// The editor's UI sound bank (pointer) and the volume it plays them at (f32).
const OFF_EDITOR_SFX_BANK: usize = 0x2120;
const OFF_EDITOR_SFX_VOLUME: usize = 0x2128;
/// `audio/ui/sfx_gen_badinput.ogg`: the sound of a refused sub-grid merge.
const SFX_BADINPUT: i32 = 3;

static PENDING: AtomicBool = AtomicBool::new(false);
static WARNED: AtomicBool = AtomicBool::new(false);

/// The game's error sound (`sfx_gen_badinput`), as when two unjoined sub-grids will not merge.
/// Any thread: it plays on the next editor tick.
pub(crate) fn error() {
    PENDING.store(true, Ordering::Relaxed);
}

/// Editor tick (the game's thread, as its own calls): plays a queued sound.
pub(crate) unsafe fn on_editor_tick(editor: usize) {
    if !PENDING.swap(false, Ordering::Relaxed) {
        return;
    }
    let base = crate::state::MODULE_BASE.load(Ordering::SeqCst);
    let (play, release) = (base + RVA_SFX_BANK_PLAY, base + RVA_SOUND_HANDLE_RELEASE);
    if base == 0 || !bytes_match(play, &SFX_BANK_PLAY_PROLOGUE) || !bytes_match(release, &SOUND_HANDLE_RELEASE_PROLOGUE) {
        if !WARNED.swap(true, Ordering::Relaxed) {
            log_line("sound: sfx_bank_play not found (game update?): no error sound");
        }
        return;
    }
    if !ptr_ok(editor + OFF_EDITOR_SFX_BANK, 0x10) {
        return;
    }
    let bank = read_usize_at(editor + OFF_EDITOR_SFX_BANK);
    // The bank's sound manager, and its entries (24 bytes each from +0x20) up to ours.
    if bank < PTR_MIN || !ptr_ok(bank, 0x20 + 0x18 * (SFX_BADINPUT as usize + 1)) || read_usize_at(bank + 8) < PTR_MIN {
        return;
    }
    let volume = f32::from_bits(read_u32_at(editor + OFF_EDITOR_SFX_VOLUME));
    type Play = unsafe extern "system" fn(usize, *mut u64, i32, f32, u8, f32) -> *mut u64;
    type Release = unsafe extern "system" fn(*mut u64);
    let mut handle: u64 = 0;
    std::mem::transmute::<usize, Play>(play)(bank, &mut handle, SFX_BADINPUT, if volume.is_finite() { volume } else { 1.0 }, 0, 1.0);
    std::mem::transmute::<usize, Release>(release)(&mut handle);
}
