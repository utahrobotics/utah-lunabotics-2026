use bincode::de::Decoder;
use bincode::de::read::Reader;
use bincode::enc::Encoder;
use bincode::enc::write::Writer;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use cu29::prelude::{ArrayLike, CuHandle};
#[allow(unused_imports)]
use cu29::{CuError, CuResult};
use serde::{Deserialize, Serialize, Serializer};

#[derive(Default, Debug, Clone, Copy, Encode, Decode, Serialize, Deserialize)]
pub struct CuDepthFrameFormat {
    pub width: u32,
    pub height: u32,
    pub depth_scale: f32,
    pub focal_len: (f32, f32),
}

impl CuDepthFrameFormat {
    #[inline]
    pub fn element_count(&self) -> usize {
        self.width as usize * self.height as usize
    }
}

#[derive(Debug, Default, Clone)]
pub struct CuDepthFrame<A>
where
    A: ArrayLike<Element = u16>,
{
    pub seq: u64,
    pub format: CuDepthFrameFormat,
    pub buffer_handle: CuHandle<A>,
}

impl<A> Encode for CuDepthFrame<A>
where
    A: ArrayLike<Element = u16>,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), bincode::error::EncodeError> {
        self.seq.encode(encoder)?;
        self.format.encode(encoder)?;
        self.buffer_handle.with_inner(|buf| {
            let len = buf.len();
            len.encode(encoder)?;
            // u16 is plain data; reinterpreting as bytes is safe.
            let bytes = unsafe {
                core::slice::from_raw_parts(buf.as_ptr() as *const u8, len * 2)
            };
            encoder.writer().write(bytes)
        })
    }
}

impl Decode<()> for CuDepthFrame<Vec<u16>> {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let seq = u64::decode(decoder)?;
        let format = CuDepthFrameFormat::decode(decoder)?;
        let len = usize::decode(decoder)?;
        let mut bytes = vec![0u8; len * 2];
        decoder.reader().read(&mut bytes)?;
        let buffer: Vec<u16> = bytes
            .chunks_exact(2)
            .map(|c| u16::from_ne_bytes([c[0], c[1]]))
            .collect();
        Ok(Self { seq, format, buffer_handle: CuHandle::new_detached(buffer) })
    }
}

impl<A> Serialize for CuDepthFrame<A>
where
    A: ArrayLike<Element = u16>,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("CuDepthFrame", 3)?;
        s.serialize_field("seq", &self.seq)?;
        s.serialize_field("format", &self.format)?;
        s.serialize_field("handle", &Vec::<u16>::new())?;
        s.end()
    }
}

impl<'de> Deserialize<'de> for CuDepthFrame<Vec<u16>> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire {
            seq: u64,
            format: CuDepthFrameFormat,
            handle: Vec<u16>,
        }
        let wire = Wire::deserialize(deserializer)?;
        Ok(Self { seq: wire.seq, format: wire.format, buffer_handle: CuHandle::new_detached(wire.handle) })
    }
}

impl<A> CuDepthFrame<A>
where
    A: ArrayLike<Element = u16>,
{
    pub fn new(format: CuDepthFrameFormat, buffer_handle: CuHandle<A>) -> Self {
        assert!(
            format.element_count() <= buffer_handle.with_inner(|i| i.len()),
            "Buffer must hold at least width × height u16 elements."
        );
        Self { seq: 0, format, buffer_handle }
    }

    /// Returns depth in metres, or `None` if the sample is invalid (raw == 0).
    pub fn depth_at_metres(&self, col: u32, row: u32) -> Option<f32> {
        assert!(col < self.format.width && row < self.format.height, "Out of bounds");
        let idx = row as usize * self.format.width as usize + col as usize;
        self.buffer_handle.with_inner(|buf| {
            let raw = buf[idx];
            if raw == 0 { None } else { Some(raw as f32 * self.format.depth_scale) }
        })
    }
}