use cu29::prelude::CuPool;
use cu29::prelude::{CuHandle, CuHostMemoryPool};
use std::convert::TryInto;
use std::time::Duration;
use std::{io, mem, sync::Arc};
use v4l::buffer::{Metadata, Type};
use v4l::device::Handle;
use v4l::io::traits::{CaptureStream, Stream};
use v4l::memory::Memory;
use v4l::v4l_sys::{v4l2_buffer, v4l2_buffer__bindgen_ty_1, v4l2_format, v4l2_requestbuffers};
use v4l::{v4l2, Device};

// A specialized V4L stream that uses Copper Buffers for memory management.
pub struct CuV4LStream {
    v4l_handle: Arc<Handle>,
    v4l_buf_type: Type,
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    // Arena matching the vl42 metadata and the Copper Buffers
    arena: Vec<(Metadata, Option<CuHandle<Vec<u8>>>)>,
    arena_last_freed_up_index: usize,
    timeout: Option<i32>,
    active: bool,
}

use std::fs;
use std::os::fd::RawFd;
use std::path::PathBuf;

fn get_original_dev_path(fd: RawFd) -> Option<PathBuf> {
    let link_path = format!("/proc/self/fd/{fd}");

    if let Ok(path) = fs::read_link(link_path) {
        if path.to_string_lossy().starts_with("/dev/video") {
            return Some(path);
        }
    }
    None
}

impl CuV4LStream {
    
    pub fn try_next(&mut self) -> io::Result<Option<(&CuHandle<Vec<u8>>, &Metadata)>> {
        if !self.active {
            return Err(io::Error::other("Stream not active"));
        }

        // Use existing timeout instead of forcing non-blocking
        match self.dequeue() {
            Ok(index) => {
                self.arena_last_freed_up_index = index;
                let (meta, handle) = &self.arena[index];
                let handle = handle.as_ref().ok_or_else(|| 
                    io::Error::new(io::ErrorKind::Other, "Missing buffer handle")
                )?;
                Ok(Some((handle, meta)))
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e),
        }
    }

    #[allow(dead_code)]
    pub fn new(dev: &Device, buf_size: usize, buf_type: Type) -> io::Result<Self> {
        let original_path = get_original_dev_path(dev.handle().fd()).unwrap();
        let pool = CuHostMemoryPool::new(
            format!("V4L Host Pool {}", original_path.display()).as_str(),
            4,
            || vec![0; buf_size],
        )
        .map_err(io::Error::other)?;

        CuV4LStream::with_buffers(dev, buf_type, 4, pool)
    }

    pub fn with_buffers(
        dev: &Device,
        buf_type: Type,
        buf_count: u32,
        pool: Arc<CuHostMemoryPool<Vec<u8>>>,
    ) -> io::Result<Self> {
        let mut arena = Vec::new();
        arena.resize(buf_count as usize, (Metadata::default(), None));

        let mut result = CuV4LStream {
            v4l_handle: dev.handle(),
            pool,
            arena,
            arena_last_freed_up_index: 0,
            v4l_buf_type: buf_type,
            active: false,
            timeout: None,
        };
        result.allocate_request_buffers(buf_count)?;
        Ok(result)
    }

    /// Returns the raw device handle
    #[allow(dead_code)]
    pub fn handle(&self) -> Arc<Handle> {
        self.v4l_handle.clone()
    }

    /// Sets a timeout of the v4l file handle.
    pub fn set_timeout(&mut self, duration: Duration) {
        self.timeout = Some(duration.as_millis().try_into().unwrap());
    }

    /// Clears the timeout of the v4l file handle.
    #[allow(dead_code)]
    pub fn clear_timeout(&mut self) {
        self.timeout = None;
    }

    fn buffer_desc(&self) -> v4l2_buffer {
        v4l2_buffer {
            type_: self.v4l_buf_type as u32,
            memory: Memory::UserPtr as u32,
            ..unsafe { mem::zeroed() }
        }
    }

    // From v4l::Arena

    fn requestbuffers_desc(&self) -> v4l2_requestbuffers {
        v4l2_requestbuffers {
            type_: self.v4l_buf_type as u32,
            memory: Memory::UserPtr as u32,
            ..unsafe { mem::zeroed() }
        }
    }

    pub fn allocate_request_buffers(&mut self, count: u32) -> io::Result<u32> {
        // we need to get the maximum buffer size from the format first
        let mut v4l2_fmt = v4l2_format {
            type_: self.v4l_buf_type as u32,
            ..unsafe { mem::zeroed() }
        };
        unsafe {
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_G_FMT,
                &mut v4l2_fmt as *mut _ as *mut std::os::raw::c_void,
            )?;
        }

        let mut v4l2_reqbufs = v4l2_requestbuffers {
            count,
            ..self.requestbuffers_desc()
        };
        unsafe {
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_REQBUFS,
                &mut v4l2_reqbufs as *mut _ as *mut std::os::raw::c_void,
            )?;
        }

        Ok(v4l2_reqbufs.count)
    }

    #[allow(dead_code)]
    pub fn release(&mut self) -> io::Result<()> {
        // free all buffers by requesting 0
        let mut v4l2_reqbufs = v4l2_requestbuffers {
            count: 0,
            ..self.requestbuffers_desc()
        };
        unsafe {
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_REQBUFS,
                &mut v4l2_reqbufs as *mut _ as *mut std::os::raw::c_void,
            )
        }
    }
}

impl Drop for CuV4LStream {
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            if let Some(code) = e.raw_os_error() {
                // ENODEV means the file descriptor wrapped in the handle became invalid, most
                // likely because the device was unplugged or the connection (USB, PCI, ..)
                // broke down. Handle this case gracefully by ignoring it.
                if code == 19 {
                    /* ignore */
                    return;
                }
            }

            panic!("{e:?}")
        }
    }
}

impl Stream for CuV4LStream {
    type Item = CuHandle<Vec<u8>>;

    fn start(&mut self) -> io::Result<()> {
        // Enqueue all buffers once on stream start
        // -1 to leave one buffer unqueued as temporary storage
        for index in 1..self.arena.len() {
            self.queue(index)?;
        }

        self.arena_last_freed_up_index = 0;

        let res = unsafe {
            let mut type_ = self.v4l_buf_type as u32;
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_STREAMON,
                &mut type_ as *mut _ as *mut std::os::raw::c_void,
            )
        };

        res?;
        self.active = true;
        Ok(())
    }

    fn stop(&mut self) -> io::Result<()> {
        let res = unsafe {
            let mut type_ = self.v4l_buf_type as u32;
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_STREAMOFF,
                &mut type_ as *mut _ as *mut std::os::raw::c_void,
            )
        };

        res?;
        self.active = false;
        Ok(())
    }
}

impl CaptureStream<'_> for CuV4LStream {
    fn queue(&mut self, index: usize) -> io::Result<()> {
        // Use existing buffer if available
        let handle = match self.arena[index].1.as_ref() {
            Some(h) => h,
            None => {
                // Allocate new buffer if missing
                let new_handle = self.pool.acquire()
                    .ok_or(io::Error::new(io::ErrorKind::Other, "failed to aquire pool"))?;
                self.arena[index] = (Metadata::default(), Some(new_handle));
                self.arena[index].1.as_ref().unwrap()
            }
        };

        handle.with_inner_mut(|inner| {
            let mut v4l2_buf = v4l2_buffer {
                index: index as u32,
                m: v4l2_buffer__bindgen_ty_1 {
                    userptr: inner.as_ptr() as u64,
                },
                length: inner.len() as u32,
                ..self.buffer_desc()
            };

            unsafe {
                v4l2::ioctl(
                    self.v4l_handle.fd(),
                    v4l2::vidioc::VIDIOC_QBUF,
                    &mut v4l2_buf as *mut _ as *mut _,
                )
            }
        })
    }

    fn dequeue(&mut self) -> io::Result<usize> {
        let mut v4l2_buf = self.buffer_desc();

        if self
            .v4l_handle
            .poll(libc::POLLIN, self.timeout.unwrap_or(-1))?
            == 0
        {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "VIDIOC_DQBUF"));
        }

        let res = unsafe {
            v4l2::ioctl(
                self.v4l_handle.fd(),
                v4l2::vidioc::VIDIOC_DQBUF,
                &mut v4l2_buf as *mut _ as *mut std::os::raw::c_void,
            )
        };
        if let Err(ref e) = res {
        }
        res?;
        let index = v4l2_buf.index as usize;

        let (metadata, _) = &mut self.arena[index];

        *metadata = Metadata {
            bytesused: v4l2_buf.bytesused,
            flags: v4l2_buf.flags.into(),
            field: v4l2_buf.field,
            timestamp: v4l2_buf.timestamp.into(),
            sequence: v4l2_buf.sequence,
        };

        Ok(index)
    }


    fn next(&mut self) -> io::Result<(&Self::Item, &Metadata)> {
        if !self.active {
            return Err(io::Error::other(
                "Stream is not active. Call start() first.",
            ));
        }

        // Dequeue a buffer
        let dequeued_index = self.dequeue()?;
        self.arena_last_freed_up_index = dequeued_index;
        let buffer = self.arena[dequeued_index].1.as_ref().expect("Buffer handle should exist");
        let meta = &self.arena[dequeued_index].0;
        Ok((buffer, meta))
    }
}

impl CuV4LStream {
    /// Call this after processing a frame to re-queue the buffer for streaming
    pub fn requeue(&mut self, index: usize) -> io::Result<()> {
        self.queue(index)
    }

    /// Returns the index of the last dequeued buffer for requeueing
    pub fn last_dequeued_index(&self) -> usize {
        self.arena_last_freed_up_index
    }
}