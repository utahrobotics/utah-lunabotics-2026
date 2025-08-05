use bytes::Buf;
use cobs::decode_vec;
use tokio_util::codec::Decoder;

pub struct CobsCodec;

impl Decoder for CobsCodec {
    type Item = Vec<u8>;
    type Error = std::io::Error;
    fn decode(&mut self, src: &mut bytes::BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        if let Some(pos) = src.iter().position(|b| *b == 0) {
            let frame = src.split_to(pos);
            src.advance(1); // drop sentinel
            return Ok(Some(decode_vec(&frame).map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::Other, "failed to decode")
            })?));
        }
        Ok(None)
    }
}
