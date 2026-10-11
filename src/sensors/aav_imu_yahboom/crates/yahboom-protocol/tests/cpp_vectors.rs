//! The C++ driver's answers, kept after it was removed.
//!
//! `cpp_vectors.txt` holds a byte stream and every sample `yahboom_imu_cpp` published for
//! it, recorded through a pseudo-terminal by `test/capture_cpp_vectors.py` (removed with the
//! C++ driver: PROVENANCE.md says where it is). The same bytes through [`Decoder`] must
//! give the same samples, chunk by chunk and bit for bit.

use yahboom_protocol::{Decoder, Event, Sample};

/// One chunk of the stream, as it was written to the port, and what the C++ published.
struct Chunk {
    /// Where the chunk is in `cpp_vectors.txt`, for a failure message.
    line: usize,
    bytes: Vec<u8>,
    samples: Vec<Sample>,
}

fn chunks() -> Vec<Chunk> {
    let mut chunks: Vec<Chunk> = Vec::new();
    for (index, line) in include_str!("cpp_vectors.txt").lines().enumerate() {
        if let Some(hex) = line.strip_prefix("bytes ") {
            chunks.push(Chunk {
                line: index + 1,
                bytes: hex
                    .split_whitespace()
                    .map(|b| u8::from_str_radix(b, 16).expect("a byte in hex"))
                    .collect(),
                samples: Vec::new(),
            });
        } else if let Some(values) = line.strip_prefix("sample ") {
            let chunk = chunks.last_mut().expect("a sample follows its bytes");
            chunk.samples.push(sample(values));
        }
    }
    chunks
}

/// Reads "ax ay az | gx gy gz | mx my mz".
fn sample(text: &str) -> Sample {
    let groups: Vec<[f64; 3]> = text
        .split('|')
        .map(|group| {
            let values: Vec<f64> = group
                .split_whitespace()
                .map(|v| v.parse().expect("a number"))
                .collect();
            values.try_into().expect("three numbers in a group")
        })
        .collect();
    match groups[..] {
        [accel, gyro, mag] => Sample { accel, gyro, mag },
        _ => panic!("expected accel | gyro | mag, got {text:?}"),
    }
}

/// The samples as raw bits, so that -0.0 and 0.0 differ, as they did on the wire.
fn bits(samples: &[Sample]) -> Vec<[u64; 9]> {
    samples
        .iter()
        .map(|s| {
            let mut out = [0; 9];
            let all = s.accel.iter().chain(&s.gyro).chain(&s.mag);
            for (slot, v) in out.iter_mut().zip(all) {
                *slot = v.to_bits();
            }
            out
        })
        .collect()
}

#[test]
fn the_vectors_are_whole() {
    // As capture_cpp_vectors.py reported: 46 chunks, and 46 samples from the C++ driver.
    let chunks = chunks();
    assert_eq!(chunks.len(), 46);
    assert_eq!(chunks.iter().map(|c| c.samples.len()).sum::<usize>(), 46);
}

#[test]
fn the_decoder_publishes_what_the_cpp_driver_published() {
    // One decoder for the whole stream, as one port: the latest readings carry over.
    let mut decoder = Decoder::default();
    let mut bad_frames = 0;
    for chunk in chunks() {
        let mut got = Vec::new();
        for &byte in &chunk.bytes {
            match decoder.push(byte) {
                Some(Event::Sample(s)) => got.push(s),
                Some(Event::BadChecksum) => bad_frames += 1,
                None => {}
            }
        }
        assert_eq!(
            bits(&got),
            bits(&chunk.samples),
            "the chunk on line {} of cpp_vectors.txt",
            chunk.line
        );
    }
    assert_eq!(
        bad_frames, 1,
        "the stream has one frame with a broken checksum"
    );
}
